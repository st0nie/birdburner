//! Application state + routes. HTTP framing/parsing is done by picoserve.
use alloc::{format, rc::Rc, string::String};
use core::cell::RefCell;
use serde::{Deserialize, Serialize};
use picoserve::{
    extract::{FromRequest, State},
    io::Read,
    request::{RequestBody, RequestParts},
    response::{IntoResponse, Response, StatusCode},
    routing::{get, post, PathRouter},
};
use crate::{
    heater_control::{Fault, HeaterControl, PidConfig, SafetyLimits, LIMIT_RANGE_MC, MAX_OUTPUT_RANGE_PCT, PWM_WINDOW_MS, SAMPLE_WINDOW_MS, STEP_MC, TARGET_RANGE_MC},
    persist::Settings,
    sensor_display::{DisplayTemperature, SensorDisplay},
};

pub type Shared = Rc<RefCell<AppState>>;
const MAX_BODY: usize = 128;
const INDEX_HTML: &str = include_str!("index.html");

pub struct AppState {
    pub control: HeaterControl,
    pub sensor: SensorDisplay,
    pub temperature_mc: Option<i32>,
    pub wifi_connected: bool,
    pub ip: Option<String>,
    pub relay_closed: bool,
    pub now_ms: u64,
    pub sample_seq: u64,
    pub sample_hz: f64,
    /// Set whenever persisted settings change; the storage task clears it.
    pub settings_dirty: bool,
    pub storage_ok: bool,
    // Monotonic counters for /metrics (reset only by reboot; see uptime).
    pub sensor_errors_total: u64,
    /// Failed reads by cause: no_presence, crc, bad_data, sensor_reset, timing, power.
    pub sensor_errors_by_kind: [(&'static str, u64); 6],
    pub relay_on_ms_total: u64,
    pub relay_switches_total: u64,
    pub storage_writes_total: u64,
    pub storage_failures_total: u64,
    last_tick_ms: Option<u64>,
    /// Current averaging window: start time, sum/count of good reads, failed reads.
    window_start_ms: Option<u64>,
    window_sum_mc: i64,
    window_ok: u32,
    window_err: u32,
    /// Good / failed raw reads in the last completed window.
    pub last_window: (u32, u32),
}

impl AppState {
    pub fn new(limits: Option<SafetyLimits>, saved: Settings) -> Self {
        let mut control = HeaterControl::new(PidConfig::default(), limits);
        // Target first: it always fits under the default limit, and the saved
        // limit was validated against the saved target when it was stored.
        control.set_target(saved.target_mc);
        control.set_max_temperature(saved.max_temperature_mc);
        control.set_max_output(saved.max_output_pct);
        control.restore_desired(saved.desired_enabled);
        Self {
            control, sensor: SensorDisplay::default(), temperature_mc: None,
            wifi_connected: false, ip: None, relay_closed: false, now_ms: 0,
            sample_seq: 0, sample_hz: 0.0, settings_dirty: false, storage_ok: true,
            sensor_errors_total: 0, relay_on_ms_total: 0,
            sensor_errors_by_kind: [("no_presence", 0), ("crc", 0), ("bad_data", 0), ("sensor_reset", 0), ("timing", 0), ("power", 0)], relay_switches_total: 0,
            storage_writes_total: 0, storage_failures_total: 0, last_tick_ms: None,
            window_start_ms: None, window_sum_mc: 0, window_ok: 0, window_err: 0, last_window: (0, 0),
        }
    }

    pub fn settings(&self) -> Settings {
        Settings {
            target_mc: self.control.target_mc(),
            desired_enabled: self.control.desired_enabled(),
            max_temperature_mc: self.control.max_temperature_mc().unwrap_or(crate::persist::DEFAULT_MAX_MC),
            max_output_pct: self.control.max_output_pct(),
        }
    }

    /// Run a control mutation and mark settings dirty if they changed.
    fn mutate<R>(&mut self, f: impl FnOnce(&mut HeaterControl) -> R) -> R {
        let before = self.settings();
        let result = f(&mut self.control);
        if self.settings() != before { self.settings_dirty = true; }
        result
    }

    /// One successful raw DS18B20 read. Only feeds the current window.
    pub fn raw_reading(&mut self, mc: i32, ms: u64) {
        self.roll_window(ms);
        self.window_sum_mc += mc as i64;
        self.window_ok += 1;
        self.sample_seq += 1;
    }

    /// One failed raw read. Ignored for control unless the whole window fails.
    pub fn raw_error(&mut self, ms: u64, kind: &str) {
        self.roll_window(ms);
        self.window_err += 1;
        self.sensor_errors_total += 1;
        if let Some(slot) = self.sensor_errors_by_kind.iter_mut().find(|(k, _)| *k == kind) { slot.1 += 1; }
    }

    /// Close the window once it is `SAMPLE_WINDOW_MS` old: its average becomes
    /// the control sample; a window with no good read counts as one failure.
    /// Also called from `tick`, so a hung sensor still produces failures.
    fn roll_window(&mut self, ms: u64) {
        let start = *self.window_start_ms.get_or_insert(ms);
        if ms.saturating_sub(start) < SAMPLE_WINDOW_MS { return; }
        let n = self.window_ok as i64;
        if n > 0 {
            let half = n / 2;
            let sum = self.window_sum_mc;
            let avg = if sum >= 0 { (sum + half) / n } else { (sum - half) / n };
            self.sample(avg as i32, ms);
        } else {
            self.window_failed(ms);
        }
        self.last_window = (self.window_ok, self.window_err);
        self.window_start_ms = Some(ms);
        self.window_sum_mc = 0;
        self.window_ok = 0;
        self.window_err = 0;
    }

    /// Apply one control sample (a window average).
    pub fn sample(&mut self, mc: i32, ms: u64) {
        self.now_ms = ms;
        self.temperature_mc = Some(mc);
        self.sensor.success(mc);
        self.mutate(|c| c.sample(mc, ms));
    }

    /// A whole window without a single good read.
    pub fn window_failed(&mut self, ms: u64) {
        self.now_ms = ms;
        self.temperature_mc = None;
        self.sensor.error();
        self.mutate(|c| c.sensor_error());
    }

    /// Called by the relay task; returns the SSR command.
    pub fn tick(&mut self, ms: u64) -> bool {
        self.roll_window(ms);
        self.now_ms = ms;
        if let Some(last) = self.last_tick_ms {
            if self.relay_closed { self.relay_on_ms_total += ms.saturating_sub(last); }
        }
        self.last_tick_ms = Some(ms);
        let closed = self.mutate(|c| c.tick(ms));
        if closed != self.relay_closed { self.relay_switches_total += 1; }
        self.relay_closed = closed;
        closed
    }

    /// Prometheus text exposition format (version 0.0.4).
    pub fn metrics(&self) -> String {
        use core::fmt::Write;
        let c = &self.control;
        let mut out = String::with_capacity(3072);
        let mut metric = |name: &str, kind: &str, help: &str, value: Option<f64>| {
            let _ = write!(out, "# HELP birdburner_{name} {help}\n# TYPE birdburner_{name} {kind}\n");
            if let Some(v) = value { let _ = write!(out, "birdburner_{name} {v}\n"); }
        };
        let flag = |b: bool| Some(if b { 1.0 } else { 0.0 });
        let celsius = |mc: i32| mc as f64 / 1000.0;
        // Absent (not 0) when there is no valid reading, so alerts/graphs are not fooled.
        metric("temperature_celsius", "gauge", "Cage-centre temperature, 5 s average of good reads.", self.temperature_mc.map(celsius));
        metric("target_temperature_celsius", "gauge", "Target temperature.", Some(celsius(c.target_mc())));
        metric("max_temperature_celsius", "gauge", "Software over-temperature cutoff.", c.max_temperature_mc().map(celsius));
        metric("sensor_ok", "gauge", "1 if the last 5 s window had at least one good read.", flag(self.temperature_mc.is_some()));
        metric("window_ok_reads", "gauge", "Good raw reads in the last 5 s window.", Some(self.last_window.0 as f64));
        metric("window_failed_reads", "gauge", "Failed raw reads in the last 5 s window.", Some(self.last_window.1 as f64));
        metric("sensor_consecutive_errors", "gauge", "Consecutive 5 s windows without a good read.", Some(self.sensor.consecutive_errors() as f64));
        metric("sample_rate_hertz", "gauge", "Successful readings per second (5 s window).", Some(self.sample_hz));
        metric("pid_output_percent", "gauge", "PID computed output; absent when not computed.", c.pid_output_pct());
        metric("heater_duty_percent", "gauge", "Duty actually commanded to the SSR.", Some(c.commanded_duty_pct(self.now_ms)));
        metric("max_output_percent", "gauge", "User cap on heater duty.", Some(c.max_output_pct() as f64));
        metric("heater_on", "gauge", "1 while the SSR is commanded closed.", flag(self.relay_closed));
        metric("control_enabled", "gauge", "1 while PID is driving the heater.", flag(c.enabled()));
        metric("control_desired_enabled", "gauge", "1 if the user wants heating on (persisted).", flag(c.desired_enabled()));
        metric("wifi_connected", "gauge", "1 if associated to WiFi.", flag(self.wifi_connected));
        metric("storage_ok", "gauge", "1 if the last settings flash write succeeded.", flag(self.storage_ok));
        metric("uptime_seconds", "counter", "Seconds since boot; a drop means a reboot/power cut.", Some(self.now_ms as f64 / 1000.0));
        metric("sensor_readings_total", "counter", "Successful DS18B20 readings since boot.", Some(self.sample_seq as f64));
        metric("sensor_errors_total", "counter", "Failed DS18B20 readings since boot.", Some(self.sensor_errors_total as f64));
        metric("heater_on_seconds_total", "counter", "Time the SSR was commanded closed since boot.", Some(self.relay_on_ms_total as f64 / 1000.0));
        metric("heater_switches_total", "counter", "SSR command transitions since boot.", Some(self.relay_switches_total as f64));
        metric("storage_writes_total", "counter", "Settings flash writes attempted since boot.", Some(self.storage_writes_total as f64));
        metric("storage_failures_total", "counter", "Settings flash writes that failed verification.", Some(self.storage_failures_total as f64));
        if let Some(w) = c.heater_watts() {
            metric("heater_rated_watts", "gauge", "Configured heater power rating.", Some(w as f64));
        }
        let _ = write!(out, "# HELP birdburner_sensor_errors_by_kind_total Failed DS18B20 reads by cause.\n# TYPE birdburner_sensor_errors_by_kind_total counter\n");
        for (kind, n) in self.sensor_errors_by_kind {
            let _ = write!(out, "birdburner_sensor_errors_by_kind_total{{kind=\"{kind}\"}} {n}\n");
        }
        // Enum-style state sets: exactly one series is 1.
        let _ = write!(out, "# HELP birdburner_mode Control mode.\n# TYPE birdburner_mode gauge\n");
        for m in ["pid", "standby", "stopped", "fault"] {
            let _ = write!(out, "birdburner_mode{{mode=\"{m}\"}} {}\n", (c.mode() == m) as u8);
        }
        let _ = write!(out, "# HELP birdburner_fault Active fault (none when healthy).\n# TYPE birdburner_fault gauge\n");
        let active = c.fault().map_or("none", Fault::label);
        for f in ["none", "sensor_error", "sensor_stale", "invalid_sample", "over_temperature", "invalid_config"] {
            let _ = write!(out, "birdburner_fault{{fault=\"{f}\"}} {}\n", (active == f) as u8);
        }
        out
    }

    pub fn temp_text(&self) -> String {
        if self.sensor.show_error() { return String::from("Sensor Error"); }
        match self.sensor.last_good_mc() {
            Some(mc) => format!("{} C{}", DisplayTemperature(mc), if self.sensor.stale() { "*" } else { "" }),
            None => format!("Reading...{}", if self.sensor.stale() { "*" } else { "" }),
        }
    }

    pub fn status(&self) -> Status {
        let c = &self.control;
        Status {
            temperature_c: self.temperature_mc.map(|mc| mc as f64 / 1000.0),
            display: self.temp_text(),
            target_c: c.target_mc() as f64 / 1000.0,
            sensor: if self.sensor.stale() { "error" } else if self.temperature_mc.is_some() { "ok" } else { "reading" },
            last_good_temperature_c: self.sensor.last_good_mc().map(|mc| mc as f64 / 1000.0),
            consecutive_sensor_errors: self.sensor.consecutive_errors(),
            wifi_connected: self.wifi_connected, ip: self.ip.clone(),
            ssr_command: if self.relay_closed { "Closed" } else { "Open" },
            mode: c.mode(), pid_enabled: c.enabled(), desired_enabled: c.desired_enabled(),
            recovery_samples: c.recovery_samples(),
            pid_output_pct: c.pid_output_pct(), commanded_duty_pct: c.commanded_duty_pct(self.now_ms),
            pwm_window_ms: PWM_WINDOW_MS, fault: c.fault().map(Fault::label),
            safety_configured: c.safety_configured(),
            max_temperature_c: c.max_temperature_mc().map(|mc| mc as f64 / 1000.0),
            max_output_pct: c.max_output_pct(),
            start_blocked_by: c.check_start(self.now_ms).err().map(|e| e.label()),
            storage_ok: self.storage_ok, sample_seq: self.sample_seq, sample_hz: self.sample_hz,
            window_ok_reads: self.last_window.0, window_failed_reads: self.last_window.1,
            sample_window_ms: SAMPLE_WINDOW_MS,
        }
    }
}

#[derive(Serialize)]
pub struct Status {
    pub temperature_c: Option<f64>, pub display: String, pub target_c: f64, pub sensor: &'static str,
    pub last_good_temperature_c: Option<f64>, pub consecutive_sensor_errors: u32,
    pub wifi_connected: bool, pub ip: Option<String>, pub ssr_command: &'static str,
    pub mode: &'static str, pub pid_enabled: bool, pub desired_enabled: bool, pub recovery_samples: u8,
    pub pid_output_pct: Option<f64>, pub commanded_duty_pct: f64, pub pwm_window_ms: u64,
    pub fault: Option<&'static str>,
    pub safety_configured: bool, pub max_temperature_c: Option<f64>, pub max_output_pct: u8,
    pub start_blocked_by: Option<&'static str>, pub storage_ok: bool,
    pub sample_seq: u64, pub sample_hz: f64,
    pub window_ok_reads: u32, pub window_failed_reads: u32, pub sample_window_ms: u64,
}

type JsonResponse = Response<picoserve::response::ContentHeaders, picoserve::response::ContentBody<String>>;

fn json(code: StatusCode, value: &impl Serialize) -> JsonResponse {
    let body = serde_json::to_string(value).unwrap_or_else(|_| String::from("{\"error\":\"serialization_failed\"}"));
    Response::new(code, body).with_content_type("application/json")
}

fn error(code: StatusCode, message: &str) -> JsonResponse {
    json(code, &serde_json::json!({ "error": message }))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TargetRequest { target_c: f64 }

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LimitRequest { max_temperature_c: f64 }

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MaxOutputRequest { max_output_pct: u8 }

/// Converts a Celsius value to milli-C if it is finite, on the 0.5 C grid and in range.
fn grid_mc(celsius: f64, range: core::ops::RangeInclusive<i32>) -> Option<i32> {
    if !celsius.is_finite() { return None; }
    let half_steps = celsius * 2.0;
    if half_steps != (half_steps as i32) as f64 { return None; }
    let mc = half_steps as i32 * STEP_MC;
    range.contains(&mc).then_some(mc)
}

/// Body extractor: content-type/size check + strict JSON (unknown fields rejected).
struct JsonBody<T>(T);
impl<'r, T: serde::de::DeserializeOwned> FromRequest<'r, Shared> for JsonBody<T> {
    type Rejection = JsonResponse;
    async fn from_request<R: Read>(_: &'r Shared, parts: RequestParts<'r>, body: RequestBody<'r, R>) -> Result<Self, Self::Rejection> {
        if body.content_length() > MAX_BODY { return Err(error(StatusCode::PAYLOAD_TOO_LARGE, "body_too_large")); }
        let is_json = parts.headers().get("content-type").and_then(|h| h.as_str().ok())
            .is_some_and(|s| s.split(';').next().is_some_and(|v| v.trim().eq_ignore_ascii_case("application/json")));
        if !is_json { return Err(error(StatusCode::UNSUPPORTED_MEDIA_TYPE, "application_json_required")); }
        let bytes = body.read_all().await.map_err(|_| error(StatusCode::BAD_REQUEST, "incomplete_body"))?;
        serde_json::from_slice(bytes).map(Self).map_err(|_| error(StatusCode::BAD_REQUEST, "invalid_json"))
    }
}

async fn status(State(shared): State<Shared>) -> JsonResponse {
    let s = shared.borrow().status();
    json(StatusCode::OK, &s)
}

async fn target(State(shared): State<Shared>, JsonBody(input): JsonBody<TargetRequest>) -> JsonResponse {
    let Some(mc) = grid_mc(input.target_c, TARGET_RANGE_MC) else {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "target_must_be_15_to_30_in_0_5_steps");
    };
    let mut app = shared.borrow_mut();
    if !app.mutate(|c| c.set_target(mc)) {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "target_too_close_to_safety_limit");
    }
    json(StatusCode::OK, &app.status())
}

async fn limit(State(shared): State<Shared>, JsonBody(input): JsonBody<LimitRequest>) -> JsonResponse {
    let Some(mc) = grid_mc(input.max_temperature_c, LIMIT_RANGE_MC) else {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "limit_must_be_25_to_40_in_0_5_steps");
    };
    let mut app = shared.borrow_mut();
    if !app.mutate(|c| c.set_max_temperature(mc)) {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "limit_must_be_at_least_target_plus_2");
    }
    let now = app.now_ms;
    app.tick(now); // a lowered limit may have cut the heater; apply it now
    json(StatusCode::OK, &app.status())
}

async fn start(State(shared): State<Shared>) -> JsonResponse {
    let mut app = shared.borrow_mut();
    let now = app.now_ms;
    match app.mutate(|c| c.start(now)) {
        Ok(()) => json(StatusCode::OK, &app.status()),
        Err(e) => error(StatusCode::CONFLICT, e.label()),
    }
}

async fn stop(State(shared): State<Shared>) -> JsonResponse {
    let mut app = shared.borrow_mut();
    app.mutate(|c| c.stop());
    let now = app.now_ms;
    app.tick(now); // drive the SSR command low now, not on the next relay tick
    json(StatusCode::OK, &app.status())
}

async fn max_output(State(shared): State<Shared>, JsonBody(input): JsonBody<MaxOutputRequest>) -> JsonResponse {
    if !MAX_OUTPUT_RANGE_PCT.contains(&input.max_output_pct) {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "max_output_must_be_10_to_100");
    }
    let mut app = shared.borrow_mut();
    app.mutate(|c| c.set_max_output(input.max_output_pct));
    let now = app.now_ms;
    app.tick(now); // a lower cap shortens the current PWM window now
    json(StatusCode::OK, &app.status())
}

async fn metrics(State(shared): State<Shared>) -> impl IntoResponse {
    let body = shared.borrow().metrics();
    Response::ok(body).with_content_type("text/plain; version=0.0.4; charset=utf-8")
}

async fn home() -> impl IntoResponse {
    Response::ok(INDEX_HTML).with_content_type("text/html; charset=utf-8").with_header("Cache-Control", "no-store")
}

pub fn router(shared: Shared) -> picoserve::Router<impl PathRouter, ()> {
    picoserve::Router::new()
        .route("/", get(home))
        .route("/api/status", get(status))
        .route("/metrics", get(metrics))
        .route("/api/target", post(target))
        .route("/api/limit", post(limit))
        .route("/api/max_output", post(max_output))
        .route("/api/start", post(start))
        .route("/api/stop", post(stop))
        .with_state(shared)
}
