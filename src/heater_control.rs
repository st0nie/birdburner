//! Pure heat-only PID + slow time-proportional AC SSR drive.
//! Boot is standby. No enable is allowed without reviewed safety configuration.

pub const PWM_WINDOW_MS: u64 = 2000;
/// Raw readings are averaged over this window; one window = one control sample.
pub const SAMPLE_WINDOW_MS: u64 = 5000;
/// A control sample older than this cuts the heater (more than two windows).
pub const SENSOR_MAX_AGE_MS: u64 = 12_000;
pub const MIN_AC_PULSE_MS: u64 = 20;
pub const SENSOR_RECOVERY_SAMPLES: u8 = 3;
/// User-adjustable maximum heater duty (whole percent).
pub const MAX_OUTPUT_RANGE_PCT: core::ops::RangeInclusive<u8> = 10..=100;
pub const DEFAULT_MAX_OUTPUT_PCT: u8 = 100;

#[derive(Clone, Copy)]
pub struct PidConfig {
    pub kp: f64, // percent / C
    pub ki: f64, // percent / (C * second)
    pub kd: f64, // percent * second / C; derivative on measurement
    pub output_limit_pct: f64,
}

impl Default for PidConfig {
    fn default() -> Self {
        // Gains are provisional (not tuned on this cage). The output limit is
        // only the first-boot default; the user can change it from the web page.
        Self { kp: 10.0, ki: 0.1, kd: 0.0, output_limit_pct: DEFAULT_MAX_OUTPUT_PCT as f64 }
    }
}

#[derive(Clone, Copy)]
pub struct SafetyLimits {
    pub max_temperature_mc: i32,
    pub heater_watts: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    SensorError,
    SensorStale,
    OverTemperature,
    InvalidSample,
    InvalidConfig,
}

impl Fault {
    pub fn label(self) -> &'static str {
        match self {
            Self::SensorError => "sensor_error",
            Self::SensorStale => "sensor_stale",
            Self::OverTemperature => "over_temperature",
            Self::InvalidSample => "invalid_sample",
            Self::InvalidConfig => "invalid_config",
        }
    }

}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartError {
    SafetyNotConfigured,
    SensorNotReady,
    OverTemperature,
    InvalidConfig,
}

impl StartError {
    pub fn label(self) -> &'static str {
        match self {
            Self::SafetyNotConfigured => "safety_not_configured",
            Self::SensorNotReady => "sensor_not_ready",
            Self::OverTemperature => "over_temperature",
            Self::InvalidConfig => "invalid_config",
        }
    }
}

pub struct Pid {
    config: PidConfig,
    integral: f64,
    previous_sample: Option<(u64, f64)>,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self { config, integral: 0.0, previous_sample: None }
    }

    pub fn valid(&self) -> bool {
        [self.config.kp, self.config.ki, self.config.kd].iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.config.output_limit_pct.is_finite()
            && self.config.output_limit_pct > 0.0 && self.config.output_limit_pct <= 100.0
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_sample = None;
    }

    /// Bridge a short measurement gap: keep the integral (the learned steady-state output) but
    /// forget the previous sample, so the gap is neither integrated nor turned into a derivative
    /// kick, and `update` does not treat it as a gap that wipes the integral.
    pub fn hold(&mut self) {
        self.previous_sample = None;
    }

    pub fn output_limit_pct(&self) -> f64 { self.config.output_limit_pct }

    /// Lowering the limit also clamps the integral so the cap applies at once.
    pub fn set_output_limit(&mut self, pct: f64) {
        self.config.output_limit_pct = pct;
        self.integral = self.integral.min(pct);
    }

    pub fn update(&mut self, target_c: f64, actual_c: f64, now_ms: u64) -> Option<f64> {
        if !self.valid() || !target_c.is_finite() || !actual_c.is_finite() {
            self.reset();
            return None;
        }
        let mut dt_s = 0.0;
        let mut derivative = 0.0;
        if let Some((previous_ms, previous_temp)) = self.previous_sample {
            if now_ms <= previous_ms {
                return None; // duplicate/reversed timestamp, do not integrate
            }
            let elapsed = now_ms - previous_ms;
            if elapsed > SENSOR_MAX_AGE_MS {
                self.reset(); // do not accumulate integral across a measurement gap
            } else {
                dt_s = elapsed as f64 / 1000.0;
                derivative = -(actual_c - previous_temp) / dt_s;
            }
        }
        self.previous_sample = Some((now_ms, actual_c));
        let error = target_c - actual_c;
        // Heat-only safety behavior: never demand heat at/above the target.
        if error <= 0.0 {
            self.integral = 0.0;
            return Some(0.0);
        }
        let proportional = self.config.kp * error;
        let d_term = self.config.kd * derivative;
        let candidate_i = (self.integral + self.config.ki * error * dt_s)
            .clamp(0.0, self.config.output_limit_pct);
        let candidate_output = proportional + candidate_i + d_term;
        // Conditional integration prevents windup at the output limits.
        if (0.0..=self.config.output_limit_pct).contains(&candidate_output) {
            self.integral = candidate_i;
        }
        let output = proportional + self.integral + d_term;
        if !output.is_finite() {
            self.reset();
            return None;
        }
        Some(output.clamp(0.0, self.config.output_limit_pct))
    }
}

pub struct WindowPwm {
    window_start_ms: Option<u64>,
    latched_on_ms: u64,
    window_ms: u64,
}

impl WindowPwm {
    pub fn new(window_ms: u64) -> Self {
        Self { window_start_ms: None, latched_on_ms: 0, window_ms }
    }

    pub fn valid(&self) -> bool {
        self.window_ms >= 1000 && self.window_ms.is_multiple_of(MIN_AC_PULSE_MS)
    }

    fn on_time(&self, duty_pct: f64) -> u64 {
        let raw = (self.window_ms as f64 * duty_pct / 100.0) as u64;
        raw / MIN_AC_PULSE_MS * MIN_AC_PULSE_MS
    }

    /// Latch increases at window boundaries; reductions take effect immediately.
    /// 20ms quantization avoids deliberately issuing sub-cycle pulses at 50Hz.
    pub fn command(&mut self, now_ms: u64, duty_pct: f64, enabled: bool) -> bool {
        if !enabled || !self.valid() || !duty_pct.is_finite() || duty_pct <= 0.0 || duty_pct > 100.0 {
            self.window_start_ms = None;
            self.latched_on_ms = 0;
            return false;
        }
        let requested_on_ms = self.on_time(duty_pct);
        let start = match self.window_start_ms {
            Some(start) if now_ms >= start => start,
            _ => {
                self.window_start_ms = Some(now_ms);
                self.latched_on_ms = requested_on_ms;
                now_ms
            }
        };
        let elapsed = now_ms - start;
        if elapsed >= self.window_ms {
            let next_start = now_ms - elapsed % self.window_ms;
            self.window_start_ms = Some(next_start);
            self.latched_on_ms = requested_on_ms;
        } else {
            self.latched_on_ms = self.latched_on_ms.min(requested_on_ms);
        }
        now_ms - self.window_start_ms.unwrap_or(now_ms) < self.latched_on_ms
    }
}

pub struct HeaterControl {
    pid: Pid,
    pwm: WindowPwm,
    limits: Option<SafetyLimits>,
    target_mc: i32,
    last_sample: Option<(u64, i32)>,
    output_pct: Option<f64>,
    enabled: bool,
    stopped: bool,
    fault: Option<Fault>,
    desired_enabled: bool,
    recovery_samples: u8,
    /// A probe reset is bridging a short gap: the PID integral is kept (see `sensor_resetting`).
    pid_hold: bool,
}

/// Over-temperature clears only once the cage is this far below the cutoff
/// (35 C limit -> resumes below 32 C), so it cannot chatter at the limit.
pub const OVER_TEMP_HYSTERESIS_MC: i32 = 3000;
/// Target must stay at least this far below the safety limit.
pub const TARGET_MARGIN_MC: i32 = 2000;
/// User-adjustable target range (milli-C, 0.5 C steps).
pub const TARGET_RANGE_MC: core::ops::RangeInclusive<i32> = 15_000..=30_000;
/// User-adjustable safety-limit range (milli-C, 0.5 C steps).
pub const LIMIT_RANGE_MC: core::ops::RangeInclusive<i32> = 25_000..=40_000;
/// Values must be on a 0.5 C grid.
pub const STEP_MC: i32 = 500;

impl HeaterControl {
    pub fn new(config: PidConfig, limits: Option<SafetyLimits>) -> Self {
        Self {
            pid: Pid::new(config), pwm: WindowPwm::new(PWM_WINDOW_MS), limits,
            target_mc: 25_000, last_sample: None, output_pct: None,
            enabled: false, stopped: false, fault: None,
            desired_enabled: false, recovery_samples: 0, pid_hold: false,
        }
    }

    pub fn mode(&self) -> &'static str {
        if self.fault.is_some() { "fault" }
        else if self.stopped { "stopped" }
        else if self.enabled { "pid" } else { "standby" }
    }

    pub fn enabled(&self) -> bool { self.enabled }
    pub fn desired_enabled(&self) -> bool { self.desired_enabled }
    pub fn recovery_samples(&self) -> u8 { self.recovery_samples }
    pub fn pid_output_pct(&self) -> Option<f64> { self.output_pct }
    pub fn fault(&self) -> Option<Fault> { self.fault }
    pub fn safety_configured(&self) -> bool {
        self.limits.is_some_and(|limits| limits.heater_watts > 0
            && limits.max_temperature_mc >= self.target_mc + 2000 && limits.max_temperature_mc <= 125_000)
    }
    pub fn max_temperature_mc(&self) -> Option<i32> { self.limits.map(|limits| limits.max_temperature_mc) }
    pub fn heater_watts(&self) -> Option<u32> { self.limits.map(|limits| limits.heater_watts) }
    pub fn target_mc(&self) -> i32 { self.target_mc }
    pub fn max_output_pct(&self) -> u8 { self.pid.output_limit_pct() as u8 }

    /// Change the maximum heater duty (whole percent in `MAX_OUTPUT_RANGE_PCT`).
    /// Takes effect at the next control sample; a lower cap also shortens the
    /// current PWM window immediately.
    pub fn set_max_output(&mut self, pct: u8) -> bool {
        if !MAX_OUTPUT_RANGE_PCT.contains(&pct) { return false; }
        self.pid.set_output_limit(pct as f64);
        if let Some(out) = self.output_pct.as_mut() { *out = out.min(pct as f64); }
        true
    }

    pub fn set_target(&mut self, target_mc: i32) -> bool {
        if !TARGET_RANGE_MC.contains(&target_mc) || target_mc % STEP_MC != 0 { return false; }
        if self.limits.is_some_and(|limits| target_mc + TARGET_MARGIN_MC > limits.max_temperature_mc) { return false; }
        self.target_mc = target_mc;
        self.reset_pid();
        self.output_pct = None; // wait for the NEXT fresh sample; never resume
        true
    }

    /// Every fault cuts the heater at once and recovers automatically; none is
    /// permanent, so the user's on/off choice (`desired_enabled`) is kept.
    /// Change the over-temperature cutoff. Rejected unless it is on the 0.5 C
    /// grid, inside `LIMIT_RANGE_MC` and at least `TARGET_MARGIN_MC` above the
    /// target. Lowering it below the current reading cuts the heater at once.
    pub fn set_max_temperature(&mut self, max_mc: i32) -> bool {
        if self.limits.is_none() || !LIMIT_RANGE_MC.contains(&max_mc) || max_mc % STEP_MC != 0
            || max_mc < self.target_mc + TARGET_MARGIN_MC { return false; }
        if let Some(limits) = self.limits.as_mut() { limits.max_temperature_mc = max_mc; }
        if self.last_sample.is_some_and(|(_, t)| t >= max_mc) { self.trip(Fault::OverTemperature); }
        true
    }

    /// Full PID reset: integral and previous sample gone, no pending hold.
    fn reset_pid(&mut self) {
        self.pid.reset();
        self.pid_hold = false;
    }

    /// Heater off at once; recovery progress starts from scratch. The PID is reset unless
    /// `keep_pid` (a probe reset bridging a short gap).
    fn cut_heater_keeping(&mut self, keep_pid: bool) {
        self.recovery_samples = 0;
        self.enabled = false;
        self.output_pct = None;
        if keep_pid { self.pid.hold(); self.pid_hold = true; } else { self.reset_pid(); }
        self.pwm.command(0, 0.0, false);
    }

    fn cut_heater(&mut self) {
        self.cut_heater_keeping(false);
    }

    fn trip(&mut self, fault: Fault) {
        self.cut_heater();
        self.fault = Some(fault);
    }

    pub fn sensor_error(&mut self) {
        self.last_sample = None;
        self.trip(Fault::SensorError);
    }

    /// The sensor stopped answering and a hardware reset (power cycle) is being tried. Cuts the
    /// heater like `sensor_error` (no reading is trusted) but raises NO fault: the reset usually
    /// brings the probe back within seconds, and `sample` then resumes heating after the usual
    /// 3 good windows. The user's on/off choice and any fault already active are left alone.
    ///
    /// The PID integral (the learned steady-state output) is KEPT while the heater was running,
    /// so a reset does not make the temperature sag while the integral rebuilds. It is only
    /// bridged for the length of the reset ladder (about 32 s plus 3 windows): if the reset
    /// fails the caller escalates to `sensor_error`, which wipes it, as does any fault, stop,
    /// start or target change. No reading arrives meanwhile, so nothing is integrated.
    pub fn sensor_resetting(&mut self) {
        self.last_sample = None;
        let keep = self.fault.is_none() && (self.enabled || self.pid_hold);
        self.cut_heater_keeping(keep);
    }

    /// PID integral term in percent (diagnostics and tests).
    pub fn pid_integral_pct(&self) -> f64 { self.pid.integral }

    /// After a power cut: heater off, no measurement carried over, resume the
    /// saved on/off choice once fresh readings arrive.
    pub fn restore_desired(&mut self, desired: bool) {
        self.stop();
        self.desired_enabled = desired;
        self.stopped = !desired;
    }

    pub fn sample(&mut self, temperature_mc: i32, now_ms: u64) {
        if !(-55_000..=125_000).contains(&temperature_mc) {
            self.last_sample = None;
            self.trip(Fault::InvalidSample);
            return;
        }
        if self.last_sample.is_some_and(|(ms, _)| now_ms <= ms) {
            self.trip(Fault::InvalidSample);
            return;
        }
        self.last_sample = Some((now_ms, temperature_mc));
        if let Some(limits) = self.limits {
            if temperature_mc >= limits.max_temperature_mc {
                self.trip(Fault::OverTemperature);
                return;
            }
            if self.fault == Some(Fault::OverTemperature)
                && temperature_mc >= limits.max_temperature_mc - OVER_TEMP_HYSTERESIS_MC {
                self.recovery_samples = 0; // still too warm to resume
                return;
            }
        }
        let recovering = self.fault.is_some() || (self.desired_enabled && !self.enabled);
        if recovering {
            self.recovery_samples = self.recovery_samples.saturating_add(1);
            if self.recovery_samples < SENSOR_RECOVERY_SAMPLES { return; }
            self.fault = None;
            if self.pid_hold { self.pid.hold(); self.pid_hold = false; } else { self.reset_pid(); }
            self.output_pct = None;
            if self.desired_enabled && self.safety_configured() {
                // Automatic recovery does not require temperature below target:
                // above target the heat-only PID naturally demands zero output.
                self.enabled = true;
                self.stopped = false;
            }
        }
        // Only a manual stop keeps the heater off once readings are good.
        if self.fault.is_some() || self.stopped { return; }
        self.output_pct = self.pid.update(f64::from(self.target_mc) / 1000.0,
            f64::from(temperature_mc) / 1000.0, now_ms);
        if self.output_pct.is_none() { self.trip(Fault::InvalidConfig); }
    }

    pub fn check_start(&self, now_ms: u64) -> Result<(), StartError> {
        if !self.pid.valid() || !self.pwm.valid() { return Err(StartError::InvalidConfig); }
        if !self.safety_configured() { return Err(StartError::SafetyNotConfigured); }
        let Some((sample_ms, temperature_mc)) = self.last_sample else { return Err(StartError::SensorNotReady); };
        if now_ms < sample_ms || now_ms - sample_ms >= SENSOR_MAX_AGE_MS { return Err(StartError::SensorNotReady); }
        let limits = self.limits.unwrap();
        if temperature_mc >= limits.max_temperature_mc - 2000 { return Err(StartError::OverTemperature); }
        Ok(())
    }

    pub fn start(&mut self, now_ms: u64) -> Result<(), StartError> {
        self.check_start(now_ms)?;
        if self.enabled { return Ok(()); }
        self.fault = None;
        self.stopped = false;
        self.desired_enabled = true;
        self.recovery_samples = 0;
        self.enabled = true;
        self.output_pct = None; // start OFF, wait for a new successful conversion
        self.reset_pid();
        self.pwm.command(now_ms, 0.0, false);
        Ok(())
    }

    pub fn stop(&mut self) {
        self.enabled = false;
        self.desired_enabled = false;
        self.recovery_samples = 0;
        self.stopped = true;
        self.reset_pid();
        self.output_pct = None;
        self.pwm.command(0, 0.0, false);
    }

    fn desired_duty_pct(&self, now_ms: u64) -> f64 {
        if !self.enabled || self.fault.is_some() { return 0.0; }
        if !self.last_sample.is_some_and(|(ms, _)| now_ms >= ms && now_ms - ms < SENSOR_MAX_AGE_MS) { return 0.0; }
        self.output_pct.unwrap_or(0.0)
    }

    /// Effective, quantized duty latched into the current SSR window.
    pub fn commanded_duty_pct(&self, now_ms: u64) -> f64 {
        if self.desired_duty_pct(now_ms) <= 0.0 { return 0.0; }
        self.pwm.latched_on_ms as f64 / self.pwm.window_ms as f64 * 100.0
    }

    /// Call every loop. Safety override does not wait for a PWM boundary.
    pub fn tick(&mut self, now_ms: u64) -> bool {
        if self.last_sample.is_some_and(|(ms, _)| now_ms < ms || now_ms - ms >= SENSOR_MAX_AGE_MS) {
            self.last_sample = None;
            self.trip(Fault::SensorStale);
        }
        let duty = self.desired_duty_pct(now_ms);
        self.pwm.command(now_ms, duty, self.enabled && self.fault.is_none())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn configured() -> HeaterControl {
        HeaterControl::new(PidConfig::default(), Some(SafetyLimits { max_temperature_mc: 40_000, heater_watts: 100 }))
    }

    #[test]
    fn pid_proportional_capped_and_heat_only() {
        let mut pid = Pid::new(PidConfig { kp: 10.0, ki: 0.0, kd: 0.0, output_limit_pct: 30.0 });
        assert_eq!(pid.update(25.0, 24.0, 0), Some(10.0));
        assert_eq!(pid.update(25.0, 10.0, 1000), Some(30.0));
        assert_eq!(pid.update(25.0, 25.0, 2000), Some(0.0));
        assert_eq!(pid.update(25.0, 26.0, 3000), Some(0.0));
    }

    #[test]
    fn integral_uses_actual_dt_and_anti_windup() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 1.0, kd: 0.0, output_limit_pct: 30.0 });
        assert_eq!(pid.update(25.0, 24.0, 0), Some(0.0));
        assert_eq!(pid.update(25.0, 24.0, 2000), Some(2.0));
        let mut pid = Pid::new(PidConfig { output_limit_pct: 30.0, ..PidConfig::default() });
        for ms in (0..100_000).step_by(1000) { assert_eq!(pid.update(25.0, 10.0, ms), Some(30.0)); }
        assert_eq!(pid.integral, 0.0);
        assert!(pid.update(25.0, 24.9, 100_000).unwrap() < 2.0);
    }

    #[test]
    fn derivative_is_on_measurement_not_setpoint() {
        let mut pid = Pid::new(PidConfig { kp: 1.0, ki: 0.0, kd: 2.0, output_limit_pct: 100.0 });
        assert_eq!(pid.update(25.0, 20.0, 0), Some(5.0));
        assert_eq!(pid.update(30.0, 20.0, 1000), Some(10.0));
        assert_eq!(pid.update(30.0, 21.0, 2000), Some(7.0));
    }

    #[test]
    fn invalid_numeric_and_duplicate_samples_rejected() {
        let mut pid = Pid::new(PidConfig::default());
        assert_eq!(pid.update(f64::NAN, 20.0, 0), None);
        assert_eq!(pid.update(25.0, f64::INFINITY, 0), None);
        assert!(pid.update(25.0, 20.0, 1000).is_some());
        assert_eq!(pid.update(25.0, 20.0, 1000), None);
        assert_eq!(pid.update(25.0, 20.0, 999), None);
        let mut pid = Pid::new(PidConfig { kp: -1.0, ..PidConfig::default() });
        assert_eq!(pid.update(25.0, 20.0, 0), None);
    }

    #[test]
    fn pwm_slow_window_and_immediate_off() {
        let mut pwm = WindowPwm::new(2000);
        assert!(pwm.command(0, 30.0, true));
        assert!(pwm.command(599, 30.0, true));
        assert!(!pwm.command(600, 30.0, true));
        assert!(!pwm.command(1999, 30.0, true));
        assert!(pwm.command(2000, 30.0, true));
        assert!(!pwm.command(2001, 30.0, false));
        assert!(!pwm.command(2002, 0.0, true));
        assert!(!pwm.command(2003, f64::NAN, true));
    }

    #[test]
    fn pwm_latches_increase_and_reduces_immediately() {
        let mut pwm = WindowPwm::new(2000);
        assert!(pwm.command(0, 30.0, true));
        assert!(!pwm.command(600, 80.0, true));
        assert!(!pwm.command(1999, 80.0, true));
        assert!(pwm.command(2000, 80.0, true));
        assert!(!pwm.command(2600, 10.0, true));
        assert!(!pwm.command(200_600, 10.0, true)); // no short catch-up pulses
    }

    #[test]
    fn no_subcycle_pulses_and_invalid_window() {
        let mut pwm = WindowPwm::new(2000);
        assert!(!pwm.command(0, 0.5, true));
        assert!(!pwm.command(10, 0.5, true));
        let mut pwm = WindowPwm::new(1);
        assert!(!pwm.command(0, 100.0, true));
    }

    #[test]
    fn unconfigured_standby_calculates_but_never_drives() {
        let mut control = HeaterControl::new(PidConfig::default(), None);
        control.sample(20_000, 0);
        assert_eq!(control.pid_output_pct(), Some(50.0)); // kp=10 x 5 C
        assert_eq!(control.mode(), "standby");
        assert_eq!(control.commanded_duty_pct(0), 0.0);
        assert_eq!(control.start(0), Err(StartError::SafetyNotConfigured));
        assert!(!control.tick(100));
    }

    #[test]
    fn start_requires_fresh_sample_and_first_new_conversion() {
        let mut control = configured();
        assert_eq!(control.start(0), Err(StartError::SensorNotReady));
        control.sample(24_000, 0);
        assert_eq!(control.start(0), Ok(()));
        assert!(!control.tick(100));
        control.sample(24_000, 1000);
        assert!(control.tick(1000));
        assert_eq!(control.mode(), "pid");
    }

    #[test]
    fn sensor_error_cuts_output_and_auto_recovers_after_three_good_samples() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 1000);
        assert!(control.tick(1000));
        control.sensor_error();
        assert!(!control.tick(1001));
        assert_eq!(control.fault(), Some(Fault::SensorError));
        assert!(control.desired_enabled());
        control.sample(20_000, 2000);
        control.sample(20_000, 2500);
        assert!(!control.tick(2500));
        assert_eq!(control.mode(), "fault");
        control.sample(20_000, 3000); // third good sample: recover, PID reset
        assert_eq!(control.fault(), None);
        assert_eq!(control.mode(), "pid");
        assert!(control.tick(3000));
    }

    #[test]
    fn sensor_reset_cuts_heat_without_a_fault_and_resumes_after_three_good_samples() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 1000);
        assert!(control.tick(1000));
        control.sensor_resetting();
        assert!(!control.tick(1001), "heater cut at once");
        assert_eq!(control.fault(), None, "a reset in progress is not a fault");
        assert_eq!(control.mode(), "standby");
        assert!(control.desired_enabled(), "the user's on choice is kept");
        assert_eq!(control.commanded_duty_pct(1001), 0.0);
        assert_eq!(control.check_start(1001), Err(StartError::SensorNotReady));
        control.sample(20_000, 2000);
        control.sample(20_000, 3000);
        assert!(!control.tick(3000));
        assert_eq!(control.recovery_samples(), 2);
        assert_eq!(control.fault(), None);
        control.sample(20_000, 4000);
        assert_eq!(control.mode(), "pid");
        assert!(control.tick(4000));
    }

    /// Heating at 20 C towards 25 C: returns the control after `n` 5 s samples (integral built up).
    fn heating_with_integral(n: u64) -> (HeaterControl, u64) {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        let mut ms = 0;
        for _ in 0..n { ms += 5_000; control.sample(20_000, ms); }
        (control, ms)
    }

    #[test]
    fn probe_reset_keeps_the_pid_integral_so_heating_resumes_where_it_left_off() {
        let (mut control, mut ms) = heating_with_integral(8);
        let integral = control.pid_integral_pct();
        assert!(integral > 10.0, "integral built up: {integral}");
        let before = control.pid_output_pct().unwrap();

        control.sensor_resetting();
        assert_eq!(control.fault(), None);
        assert!(!control.tick(ms + 1));
        assert_eq!(control.pid_integral_pct(), integral, "the integral survives the reset");
        // A second reset attempt in the same incident must not lose it either.
        ms += 12_000;
        control.sensor_resetting();
        assert_eq!(control.pid_integral_pct(), integral);

        // 3 good windows later heating resumes with the same integral, not from scratch.
        for _ in 0..3 { ms += 5_000; control.sample(20_000, ms); }
        assert_eq!(control.mode(), "pid");
        let after = control.pid_output_pct().unwrap();
        assert!((after - before).abs() < 1.0, "output continues at {before}, got {after}");
        assert!(after > 10.0 * 5.0 + 10.0, "P term alone is 50: the integral is still in there");
        assert_eq!(control.pid_integral_pct(), integral, "the gap itself is not integrated");
    }

    #[test]
    fn failed_probe_reset_wipes_the_integral() {
        let (mut control, mut ms) = heating_with_integral(8);
        assert!(control.pid_integral_pct() > 10.0);
        control.sensor_resetting();
        control.sensor_error(); // every reset failed: now it is a fault
        assert_eq!(control.pid_integral_pct(), 0.0);
        for _ in 0..3 { ms += 5_000; control.sample(20_000, ms); }
        assert_eq!(control.mode(), "pid");
        assert_eq!(control.pid_output_pct(), Some(50.0), "P only: nothing carried over from before the fault");
    }

    #[test]
    fn integral_is_not_kept_unless_the_heater_was_running() {
        // Standby, never heated: nothing to keep.
        let mut control = configured();
        control.sample(20_000, 0);
        control.sensor_resetting();
        assert_eq!(control.pid_integral_pct(), 0.0);
        // A fault that is already active is not softened by a reset.
        let (mut control, ms) = heating_with_integral(8);
        control.sample(41_000, ms + 5_000);
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert_eq!(control.pid_integral_pct(), 0.0);
        control.sensor_resetting();
        assert_eq!(control.pid_integral_pct(), 0.0);
        // Stop, start and a new target start clean even after a reset.
        let (mut control, _) = heating_with_integral(8);
        control.sensor_resetting();
        control.set_target(26_000);
        assert_eq!(control.pid_integral_pct(), 0.0);
        let (mut control, _) = heating_with_integral(8);
        control.sensor_resetting();
        control.stop();
        assert_eq!(control.pid_integral_pct(), 0.0);
    }

    #[test]
    fn temperature_above_target_after_the_reset_drops_the_integral() {
        let (mut control, mut ms) = heating_with_integral(8);
        control.sensor_resetting();
        for _ in 0..3 { ms += 5_000; control.sample(26_000, ms); } // warmed up meanwhile
        assert_eq!(control.pid_output_pct(), Some(0.0));
        assert_eq!(control.pid_integral_pct(), 0.0, "no heat demanded above target");
    }

    #[test]
    fn sensor_reset_keeps_manual_stop_and_existing_fault() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.stop();
        control.sensor_resetting();
        for ms in [1000, 2000, 3000, 4000] { control.sample(20_000, ms); }
        assert_eq!(control.mode(), "stopped", "recovery never re-arms a stopped heater");
        assert_eq!(control.fault(), None);
        // A fault that is already active is not replaced by the reset.
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(41_000, 1000);
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        control.sensor_resetting();
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert_eq!(control.mode(), "fault");
    }

    #[test]
    fn error_during_recovery_restarts_count() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sensor_error();
        control.sample(20_000, 1000);
        control.sample(20_000, 1500);
        control.sensor_error();
        control.sample(20_000, 2000);
        control.sample(20_000, 2500);
        assert_eq!(control.mode(), "fault");
        control.sample(20_000, 3000);
        assert_eq!(control.mode(), "pid");
    }

    #[test]
    fn stale_reading_cuts_heat_and_auto_recovers() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 1000);
        assert!(control.tick(1000));
        assert!(!control.tick(13_000));
        assert_eq!(control.fault(), Some(Fault::SensorStale));
        for ms in [14_000, 14_800, 15_600] { control.sample(20_000, ms); }
        assert_eq!(control.mode(), "pid");
    }

    #[test]
    fn reboot_restore_waits_for_fresh_samples_and_never_restores_measurement() {
        let mut control = configured();
        control.restore_desired(true);
        assert_eq!(control.mode(), "standby");
        assert!(!control.tick(0));
        control.sample(24_000, 100);
        control.sample(24_000, 900);
        assert!(!control.tick(900));
        control.sample(24_000, 1700);
        assert_eq!(control.mode(), "pid");
        assert!(control.tick(1700));
        let mut off = configured();
        off.restore_desired(false);
        for ms in [0, 800, 1600, 2400] { off.sample(24_000, ms); }
        assert_eq!(off.mode(), "stopped");
        let mut control = HeaterControl::new(PidConfig::default(), None);
        control.restore_desired(true);
        for ms in [0, 800, 1600] { control.sample(24_000, ms); }
        assert!(!control.enabled());
        assert!(!control.tick(1600));
    }

    #[test]
    fn overtemp_cuts_at_once_and_resumes_below_hysteresis() {
        let mut control = configured(); // 40 C limit -> resumes below 37 C
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 800);
        assert!(control.tick(800));
        control.sample(40_000, 1600);
        assert!(!control.tick(1600), "heater must cut on the first over-limit reading");
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert!(control.desired_enabled(), "user's on choice is kept");
        for ms in [2400, 3200, 4000, 4800] { control.sample(38_000, ms); }
        assert_eq!(control.mode(), "fault", "38 C is inside the hysteresis band");
        for ms in [5600, 6400] { control.sample(36_000, ms); }
        assert_eq!(control.mode(), "fault");
        control.sample(36_000, 7200);
        assert_eq!(control.mode(), "pid", "auto-resumes after 3 good readings below 37 C");
    }

    #[test]
    fn repeated_overtemp_never_locks_permanently() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        let mut ms = 0;
        for _ in 0..10 {
            ms += 800; control.sample(41_000, ms);
            assert_eq!(control.fault(), Some(Fault::OverTemperature));
            for _ in 0..3 { ms += 800; control.sample(20_000, ms); }
            assert_eq!(control.mode(), "pid");
        }
    }

    #[test]
    fn adjustable_limit_validated_and_applied() {
        let mut control = configured(); // 40 C limit, 25 C target
        assert!(!control.set_max_temperature(26_500), "must be >= target + 2 C");
        assert!(!control.set_max_temperature(41_000));
        assert!(!control.set_max_temperature(33_200), "0.5 C grid");
        assert!(control.set_max_temperature(27_000));
        assert_eq!(control.max_temperature_mc(), Some(27_000));
        assert!(!control.set_target(25_500), "target now capped at limit - 2 C");
        assert!(control.set_max_temperature(30_000));
        assert!(control.set_target(28_000));
        control.sample(26_000, 0);
        control.start(0).unwrap();
        control.sample(26_000, 800);
        control.sample(27_000, 1600);
        assert!(control.tick(1600));
        assert!(control.set_target(25_000));
        assert!(control.set_max_temperature(27_000), "lowering below the reading is allowed");
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert!(!control.tick(1601), "and cuts the heater immediately");
        let mut none = HeaterControl::new(PidConfig::default(), None);
        assert!(!none.set_max_temperature(35_000), "cannot enable heating via the limit");
    }

    #[test]
    fn adjustable_max_output() {
        let mut control = configured();
        assert_eq!(control.max_output_pct(), 100);
        assert!(!control.set_max_output(9));
        assert!(!control.set_max_output(101));
        control.sample(10_000, 0);
        control.start(0).unwrap();
        control.sample(10_000, 5000);
        assert_eq!(control.pid_output_pct(), Some(100.0), "15 C below target, kp=10 -> capped at 100");
        assert!(control.set_max_output(60));
        assert_eq!(control.pid_output_pct(), Some(60.0), "lower cap applies to the current output at once");
        control.sample(10_000, 10_000);
        assert_eq!(control.pid_output_pct(), Some(60.0));
        assert!(control.set_max_output(100));
        control.sample(10_000, 15_000);
        assert_eq!(control.pid_output_pct(), Some(100.0));
    }

    #[test]
    fn target_validated_against_safety_margin() {
        let mut control = HeaterControl::new(PidConfig::default(),
            Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }));
        assert!(control.set_target(30_000));
        assert!(control.safety_configured());
        let mut tight = HeaterControl::new(PidConfig::default(),
            Some(SafetyLimits { max_temperature_mc: 27_000, heater_watts: 100 }));
        assert!(!tight.set_target(26_000));
        assert!(tight.set_target(25_000));
    }


    #[test]
    fn target_updates_and_stop_never_rearm() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 1000);
        assert!(control.tick(1000));
        control.stop();
        assert!(control.set_target(30_000));
        control.sample(20_000, 2000);
        assert!(!control.tick(2000));
        assert_eq!(control.mode(), "stopped");
        assert!(!control.set_target(31_000));
        assert!(!control.set_target(25_100));
        let control = configured();
        assert!(!control.enabled());
        assert_eq!(control.mode(), "standby");
    }
}
