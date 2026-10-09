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
    heater_control::{Fault, HeaterControl, PidConfig, SafetyLimits, LIMIT_RANGE_MC, MAX_OUTPUT_RANGE_PCT, KD_RANGE_MILLI, KI_RANGE_MILLI, KP_RANGE_MILLI, PWM_WINDOW_MS, SAMPLE_WINDOW_MS, STEP_MC, TARGET_RANGE_MC},
    persist::Settings,
    sensor_display::{DisplayTemperature, SensorDisplay},
    sensor_recovery::{Event, PowerCycle, SensorRecovery},
};

pub type Shared = Rc<RefCell<AppState>>;
const MAX_BODY: usize = 128;
const INDEX_HTML: &str = include_str!("index.html");

pub struct AppState {
    pub control: HeaterControl,
    pub sensor: SensorDisplay,
    /// Probe power-cycle ladder (see `sensor_recovery`); decides when a silent probe is a fault.
    pub recovery: SensorRecovery,
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
    /// Flash save in progress (dirty remains true during debounce).
    settings_saving: bool,
    /// Recovery produced a raw reading, but its 5 s control average is not ready yet.
    awaiting_average: bool,
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
        control.set_pid_gains(saved.kp_milli, saved.ki_milli, saved.kd_milli);
        control.restore_desired(saved.desired_enabled);
        Self {
            control, sensor: SensorDisplay::default(), recovery: SensorRecovery::new(), temperature_mc: None,
            wifi_connected: false, ip: None, relay_closed: false, now_ms: 0,
            sample_seq: 0, sample_hz: 0.0, settings_dirty: false, storage_ok: true,
            settings_saving: false, awaiting_average: false,
            sensor_errors_total: 0, relay_on_ms_total: 0,
            sensor_errors_by_kind: [("no_presence", 0), ("crc", 0), ("bad_data", 0), ("sensor_reset", 0), ("timing", 0), ("power", 0)], relay_switches_total: 0,
            storage_writes_total: 0, storage_failures_total: 0, last_tick_ms: None,
            window_start_ms: None, window_sum_mc: 0, window_ok: 0, window_err: 0, last_window: (0, 0),
        }
    }

    pub fn settings(&self) -> Settings {
        let (kp_milli, ki_milli, kd_milli) = self.control.pid_gains_milli();
        Settings {
            target_mc: self.control.target_mc(),
            desired_enabled: self.control.desired_enabled(),
            max_temperature_mc: self.control.max_temperature_mc().unwrap_or(crate::persist::DEFAULT_MAX_MC),
            max_output_pct: self.control.max_output_pct(),
            kp_milli, ki_milli, kd_milli,
        }
    }

    /// Change the PID gains (thousandths); marks the settings dirty if they changed.
    pub fn mutate_gains(&mut self, kp: u32, ki: u32, kd: u32) -> bool {
        self.mutate(|c| c.set_pid_gains(kp, ki, kd))
    }

    /// Called after storage debounce. Pending stays true until verification completes.
    pub fn take_settings_to_save(&mut self) -> Option<Settings> {
        if !self.settings_dirty || self.settings_saving { return None; }
        self.settings_dirty = false;
        self.settings_saving = true;
        Some(self.settings())
    }

    /// Flash writer reports read-back verification; a failure schedules another attempt.
    pub fn finish_settings_save(&mut self, ok: bool) {
        self.settings_saving = false;
        self.storage_ok = ok;
        self.storage_writes_total += 1;
        if !ok {
            self.storage_failures_total += 1;
            self.settings_dirty = true;
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
    /// Returns true if it ended a probe reset or sensor fault (the probe answers again).
    pub fn raw_reading(&mut self, mc: i32, ms: u64) -> bool {
        self.roll_window(ms);
        self.window_sum_mc += mc as i64;
        self.window_ok += 1;
        self.sample_seq += 1;
        self.mutate(|c| c.recovery_raw_reading(mc));
        let back = self.recovery.good_read().is_some();
        self.awaiting_average |= back;
        back
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
        self.awaiting_average = false;
        self.sensor.success(mc);
        self.mutate(|c| c.sample(mc, ms));
    }

    /// A whole window without a single good read. The probe is power-cycled first (see
    /// `sensor_recovery`): a running heater holds its last duty for bounded recovery; standby
    /// or stopped heaters stay off. No new sensor fault until the attempts fail or age expires.
    pub fn window_failed(&mut self, ms: u64) {
        self.now_ms = ms;
        self.temperature_mc = None;
        self.awaiting_average = false;
        self.sensor.error();
        let event = self.recovery.window_failed(ms);
        if event.is_none() {
            // Empty windows must interrupt startup/fault recovery's consecutive-good count too.
            // A running heater already in bounded recovery keeps its duty and original deadline.
            self.mutate(|c| c.sensor_resetting());
        }
        self.apply_recovery(event);
    }

    fn apply_recovery(&mut self, event: Option<Event>) {
        match event {
            Some(Event::AttemptStarted(_)) => self.mutate(|c| c.sensor_resetting()),
            Some(Event::GaveUp) => self.mutate(|c| c.sensor_error()),
            // A running heater continues at its held duty until the next average. Actual faults
            // still recover through the control's 3-good-windows rule.
            Some(Event::Recovered) | None => {}
        }
    }

    /// Sensor task: the next probe power cycle, if one is due.
    pub fn take_power_cycle(&mut self) -> Option<PowerCycle> { self.recovery.take_power_cycle() }

    pub fn power_cycle_pending(&self) -> bool { self.recovery.power_cycle_pending() }

    /// Called by the relay task; returns the SSR command.
    pub fn tick(&mut self, ms: u64) -> bool {
        self.roll_window(ms);
        self.now_ms = ms;
        let event = self.recovery.poll(ms);
        self.apply_recovery(event);
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
        metric("sensor_resetting", "gauge", "1 while the probe is being power-cycled after it stopped answering (not a fault yet).", flag(self.recovery.resetting()));
        metric("sample_rate_hertz", "gauge", "Successful readings per second (5 s window).", Some(self.sample_hz));
        metric("pid_output_percent", "gauge", "PID computed output; absent when not computed.", c.pid_output_pct());
        metric("heater_duty_percent", "gauge", "Duty actually commanded to the SSR.", Some(c.commanded_duty_pct(self.now_ms)));
        metric("max_output_percent", "gauge", "User cap on heater duty.", Some(c.max_output_pct() as f64));
        let (kp, ki, kd) = c.pid_gains_milli();
        metric("pid_kp", "gauge", "PID proportional gain (percent per C).", Some(kp as f64 / 1000.0));
        metric("pid_ki", "gauge", "PID integral gain (percent per C per second).", Some(ki as f64 / 1000.0));
        metric("pid_kd", "gauge", "PID derivative gain (percent seconds per C).", Some(kd as f64 / 1000.0));
        metric("heater_on", "gauge", "1 while the SSR is commanded closed.", flag(self.relay_closed));
        metric("heater_output_held", "gauge", "1 while last output is held during bounded probe recovery.", flag(c.holding_through_reset()));
        metric("control_enabled", "gauge", "1 while control is enabled (including bounded output hold).", flag(c.enabled()));
        metric("control_desired_enabled", "gauge", "1 if the user wants heating on (persisted).", flag(c.desired_enabled()));
        metric("wifi_connected", "gauge", "1 if associated to WiFi.", flag(self.wifi_connected));
        metric("storage_ok", "gauge", "1 if the last settings flash write succeeded.", flag(self.storage_ok));
        metric("uptime_seconds", "counter", "Seconds since boot; a drop means a reboot/power cut.", Some(self.now_ms as f64 / 1000.0));
        metric("sensor_readings_total", "counter", "Successful DS18B20 readings since boot.", Some(self.sample_seq as f64));
        metric("sensor_errors_total", "counter", "Failed DS18B20 readings since boot.", Some(self.sensor_errors_total as f64));
        metric("sensor_power_cycles_total", "counter", "Probe power cycles performed since boot (VDD switched off and on).", Some(self.recovery.power_cycles_total() as f64));
        metric("sensor_recoveries_total", "counter", "Times the probe answered again after a reset or a fault.", Some(self.recovery.recoveries_total() as f64));
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
        // While the probe is being reset the last good value stays up with `*`; "Sensor Error"
        // is for a probe that is still silent after every reset.
        if self.sensor.show_error() && !self.recovery.resetting() && !self.awaiting_average { return String::from("Sensor Error"); }
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
            sensor: if self.recovery.resetting() { "resetting" } else if self.sensor.stale() && self.awaiting_average { "recovering" }
                else if self.sensor.stale() { "error" } else if self.temperature_mc.is_some() { "ok" } else { "reading" },
            sensor_reset_attempt: self.recovery.attempt(),
            sensor_power_cycles: self.recovery.power_cycles_total(),
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
            pid_kp: c.pid_gains_milli().0 as f64 / 1000.0,
            pid_ki: c.pid_gains_milli().1 as f64 / 1000.0,
            pid_kd: c.pid_gains_milli().2 as f64 / 1000.0,
            start_blocked_by: c.check_start(self.now_ms).err().map(|e| e.label()),
            storage_ok: self.storage_ok, settings_pending: self.settings_dirty || self.settings_saving,
            heater_output_held: c.holding_through_reset(), sample_seq: self.sample_seq, sample_hz: self.sample_hz,
            window_ok_reads: self.last_window.0, window_failed_reads: self.last_window.1,
            sample_window_ms: SAMPLE_WINDOW_MS,
        }
    }
}

#[derive(Serialize)]
pub struct Status {
    pub temperature_c: Option<f64>, pub display: String, pub target_c: f64, pub sensor: &'static str,
    pub sensor_reset_attempt: u8, pub sensor_power_cycles: u32,
    pub last_good_temperature_c: Option<f64>, pub consecutive_sensor_errors: u32,
    pub wifi_connected: bool, pub ip: Option<String>, pub ssr_command: &'static str,
    pub mode: &'static str, pub pid_enabled: bool, pub desired_enabled: bool, pub recovery_samples: u8,
    pub pid_output_pct: Option<f64>, pub commanded_duty_pct: f64, pub pwm_window_ms: u64,
    pub fault: Option<&'static str>,
    pub safety_configured: bool, pub max_temperature_c: Option<f64>, pub max_output_pct: u8,
    pub pid_kp: f64, pub pid_ki: f64, pub pid_kd: f64,
    pub start_blocked_by: Option<&'static str>, pub storage_ok: bool, pub settings_pending: bool,
    pub heater_output_held: bool,
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

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PidRequest { kp: f64, ki: f64, kd: f64 }

/// Gain to thousandths: finite, rounded to 0.001, inside the allowed range.
pub fn gain_milli(value: f64, range: core::ops::RangeInclusive<u32>) -> Option<u32> {
    if !value.is_finite() || value < f64::from(*range.start()) / 1000.0
        || value > f64::from(*range.end()) / 1000.0 { return None; }
    // Non-negative, bounded value: truncation implements floor and works in no_std.
    let milli = (value * 1000.0 + 0.5) as u32;
    range.contains(&milli).then_some(milli)
}

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

async fn pid(State(shared): State<Shared>, JsonBody(input): JsonBody<PidRequest>) -> JsonResponse {
    let (Some(kp), Some(ki), Some(kd)) = (
        gain_milli(input.kp, KP_RANGE_MILLI), gain_milli(input.ki, KI_RANGE_MILLI), gain_milli(input.kd, KD_RANGE_MILLI),
    ) else {
        return error(StatusCode::UNPROCESSABLE_ENTITY, "pid_gains_out_of_range_kp_0_100_ki_0_2_kd_0_200");
    };
    let mut app = shared.borrow_mut();
    app.mutate_gains(kp, ki, kd);
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
        .route("/api/pid", post(pid))
        .route("/api/start", post(start))
        .route("/api/stop", post(stop))
        .with_state(shared)
}
