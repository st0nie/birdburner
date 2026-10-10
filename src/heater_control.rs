//! Pure heat-only PID + slow time-proportional AC SSR drive.
//! Boot is standby. No enable is allowed without reviewed safety configuration.

pub const PWM_WINDOW_MS: u64 = 2000;
/// Raw readings are averaged over this window; one window = one control sample.
pub const SAMPLE_WINDOW_MS: u64 = 5000;
/// A control sample older than this cuts the heater (more than two windows).
pub const SENSOR_MAX_AGE_MS: u64 = 12_000;
/// Absolute age limit even while probe recovery holds output; never renewed by reset attempts.
/// Normally the 32 s reset ladder gives up earlier. This guard also covers a stalled supervisor.
pub const SENSOR_RESET_MAX_AGE_MS: u64 = 45_000;
pub const MIN_AC_PULSE_MS: u64 = 20;
pub const SENSOR_RECOVERY_SAMPLES: u8 = 3;
/// User-adjustable maximum heater duty (whole percent).
pub const MAX_OUTPUT_RANGE_PCT: core::ops::RangeInclusive<u8> = 10..=100;
pub const DEFAULT_MAX_OUTPUT_PCT: u8 = 100;
/// PID gains are stored and exchanged in thousandths. Kp: percent / C, Ki: percent / (C * s),
/// Kd: percent * s / C. Starting values informed by observed closed-loop temperature/output:
/// Kp 20 / Ki 0.02 (Ti ~1000 s) / Kd 120 (Td 6 s), with a 30 s derivative filter.
/// Lower integral action avoids the previous rapid output ramp. Ambient temperature and a
/// controlled step response were not measured; these values still need on-hardware tuning.
pub const DEFAULT_KP_MILLI: u32 = 20_000;
pub const DEFAULT_KI_MILLI: u32 = 20;
pub const DEFAULT_KD_MILLI: u32 = 120_000;
/// First-order low-pass time constant on the derivative term, thousandths of a second.
/// 0 disables filtering. 30 s spans six samples: one 0.0625 C LSB step then moves the D term by
/// at most Kd * 0.0625 / (5 + 30) instead of Kd * 0.0625 / 5.
pub const DEFAULT_D_FILTER_MILLI: u32 = 30_000;
/// At or above target + this, the PID output is forced to 0 regardless of the integral.
pub const HEAT_CUTOFF_ABOVE_TARGET_C: f64 = 1.0;
pub const D_FILTER_RANGE_MILLI: core::ops::RangeInclusive<u32> = 0..=300_000;
pub const KP_RANGE_MILLI: core::ops::RangeInclusive<u32> = 0..=100_000;
pub const KI_RANGE_MILLI: core::ops::RangeInclusive<u32> = 0..=2_000;
pub const KD_RANGE_MILLI: core::ops::RangeInclusive<u32> = 0..=200_000;

#[derive(Clone, Copy)]
pub struct PidConfig {
    pub kp: f64, // percent / C
    pub ki: f64, // percent / (C * second)
    pub kd: f64, // percent * second / C; derivative on measurement
    pub d_filter_s: f64, // first-order low-pass time constant on the derivative; 0 = off
    pub output_limit_pct: f64,
}

impl Default for PidConfig {
    fn default() -> Self {
        // Gains are provisional (not tuned on this cage). The output limit is
        // only the first-boot default; the user can change it from the web page.
        Self { kp: DEFAULT_KP_MILLI as f64 / 1000.0, ki: DEFAULT_KI_MILLI as f64 / 1000.0, kd: DEFAULT_KD_MILLI as f64 / 1000.0, d_filter_s: DEFAULT_D_FILTER_MILLI as f64 / 1000.0, output_limit_pct: DEFAULT_MAX_OUTPUT_PCT as f64 }
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
    /// Low-pass filtered derivative of the measurement (C/s, negated).
    filtered_derivative: f64,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self { config, integral: 0.0, previous_sample: None, filtered_derivative: 0.0 }
    }

    pub fn valid(&self) -> bool {
        [self.config.kp, self.config.ki, self.config.kd, self.config.d_filter_s].iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.config.output_limit_pct.is_finite()
            && self.config.output_limit_pct > 0.0 && self.config.output_limit_pct <= 100.0
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_sample = None;
        self.filtered_derivative = 0.0;
    }

    /// Bridge a short measurement gap: keep the integral (the learned steady-state output) but
    /// forget the previous sample, so the gap is neither integrated nor turned into a derivative
    /// kick, and `update` does not treat it as a gap that wipes the integral.
    pub fn hold(&mut self) {
        self.previous_sample = None;
        self.filtered_derivative = 0.0;
    }

    pub fn output_limit_pct(&self) -> f64 { self.config.output_limit_pct }

    pub fn gains_milli(&self) -> (u32, u32, u32) {
        // Validated non-negative gains: adding half then truncating rounds without std/libm.
        let m = |v: f64| (v * 1000.0 + 0.5) as u32;
        (m(self.config.kp), m(self.config.ki), m(self.config.kd))
    }

    /// Takes effect at the next sample. Retains the integral except when integral action is
    /// disabled (Ki=0), so turning off I cannot leave a hidden residual output.
    pub fn set_gains_milli(&mut self, kp: u32, ki: u32, kd: u32) {
        self.config.kp = kp as f64 / 1000.0;
        self.config.ki = ki as f64 / 1000.0;
        self.config.kd = kd as f64 / 1000.0;
        if ki == 0 { self.integral = 0.0; }
    }

    pub fn d_filter_milli(&self) -> u32 { (self.config.d_filter_s * 1000.0 + 0.5) as u32 }

    /// Takes effect at the next sample; the current filter state is kept.
    pub fn set_d_filter_milli(&mut self, tf: u32) {
        self.config.d_filter_s = tf as f64 / 1000.0;
    }

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
                let raw = -(actual_c - previous_temp) / dt_s;
                // Discrete first-order low-pass: alpha = dt / (Tf + dt); Tf = 0 passes raw.
                let alpha = dt_s / (self.config.d_filter_s + dt_s);
                self.filtered_derivative += alpha * (raw - self.filtered_derivative);
                derivative = self.filtered_derivative;
            }
        }
        self.previous_sample = Some((now_ms, actual_c));
        let error = target_c - actual_c;
        let proportional = self.config.kp * error;
        let d_term = self.config.kd * derivative;
        let integral_step = self.config.ki * error * dt_s;
        let candidate_i = (self.integral + integral_step).clamp(0.0, self.config.output_limit_pct);
        let candidate_output = proportional + candidate_i + d_term;
        // Direction-aware conditional integration: block only steps further into saturation.
        // A corrective step is allowed even if P or D keeps the request outside the range.
        let pushes_upper_limit = candidate_output > self.config.output_limit_pct && integral_step > 0.0;
        let pushes_lower_limit = candidate_output < 0.0 && integral_step < 0.0;
        if !pushes_upper_limit && !pushes_lower_limit {
            self.integral = candidate_i;
        }
        let output = proportional + self.integral + d_term;
        if !output.is_finite() {
            self.reset();
            return None;
        }
        // Heat-only guard: well above the target never heat. The integral is kept (it is the
        // learned holding power); conditional integration can reduce it through negative error,
        // but stops at lower saturation. Clearing it at the target caused the sawtooth seen on 2026-10-09.
        if -error >= HEAT_CUTOFF_ABOVE_TARGET_C { return Some(0.0); }
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
    /// The probe is being power-cycled and the heater keeps running on its last output
    /// (see `sensor_resetting`).
    probe_resetting: bool,
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
            desired_enabled: false, recovery_samples: 0, probe_resetting: false,
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
        self.pid.reset();
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

    /// Heater off at once; PID and recovery progress start from scratch.
    fn cut_heater(&mut self) {
        self.recovery_samples = 0;
        self.enabled = false;
        self.output_pct = None;
        self.probe_resetting = false;
        self.pid.reset();
        self.pwm.command(0, 0.0, false);
    }

    fn trip(&mut self, fault: Fault) {
        self.cut_heater();
        self.fault = Some(fault);
    }

    pub fn sensor_error(&mut self) {
        self.last_sample = None;
        // Probe recovery must not replace e.g. an over-temperature fault and lose its hysteresis.
        if self.fault.is_none() { self.trip(Fault::SensorError); } else { self.cut_heater(); }
    }

    /// The sensor stopped answering and a hardware reset (power cycle) is being tried. This is
    /// NOT a fault and, while the heater is running, not even an interruption: the heater keeps
    /// going on its last PID output (same duty, same PWM windows) and the integral is kept, so
    /// output is unchanged. No reading arrives meanwhile, so a new over-temperature cannot be
    /// detected. The reset ladder normally gives up after 32 s; an independent 45 s measurement
    /// age guard also bounds this hold, even if the supervisor stalls. Either cuts the heater
    /// and wipes the PID. The first valid control sample ends the
    /// hold. If the heater was not running (standby, stopped, already faulted) the reset just
    /// holds it off as before; the user's on/off choice and any active fault are left alone.
    pub fn sensor_resetting(&mut self) {
        if self.enabled && self.fault.is_none() && self.last_sample.is_some() {
            self.probe_resetting = true;
            return;
        }
        self.last_sample = None;
        self.cut_heater();
    }

    /// The heater is running on its last output while the probe is being reset.
    pub fn holding_through_reset(&self) -> bool { self.probe_resetting }

    /// PID integral term in percent (test inspection only; not persisted).
    #[cfg(test)]
    pub fn pid_integral_pct(&self) -> f64 { self.pid.integral }

    /// A fresh raw reading during recovery can cut an over-temperature immediately without
    /// treating the display cache or a single raw read as a PID control sample.
    pub fn recovery_raw_reading(&mut self, temperature_mc: i32) {
        if self.probe_resetting && self.limits.is_some_and(|l| temperature_mc >= l.max_temperature_mc) {
            self.trip(Fault::OverTemperature);
        }
    }

    pub fn pid_gains_milli(&self) -> (u32, u32, u32) { self.pid.gains_milli() }

    /// Change the PID gains (thousandths); rejected when out of range.
    pub fn set_pid_gains(&mut self, kp: u32, ki: u32, kd: u32) -> bool {
        if !KP_RANGE_MILLI.contains(&kp) || !KI_RANGE_MILLI.contains(&ki) || !KD_RANGE_MILLI.contains(&kd) { return false; }
        self.pid.set_gains_milli(kp, ki, kd);
        true
    }

    pub fn d_filter_milli(&self) -> u32 { self.pid.d_filter_milli() }

    /// Change the derivative filter time constant (thousandths of a second); rejected when out of range.
    pub fn set_d_filter(&mut self, tf: u32) -> bool {
        if !D_FILTER_RANGE_MILLI.contains(&tf) { return false; }
        self.pid.set_d_filter_milli(tf);
        true
    }

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
        if self.probe_resetting && self.last_sample.is_some_and(|(ms, _)| now_ms.saturating_sub(ms) >= SENSOR_RESET_MAX_AGE_MS) {
            self.trip(Fault::SensorStale); // a late sample must not bypass the hold deadline
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
        if self.probe_resetting {
            // The probe answered again: the gap since the previous sample is not integrated.
            self.probe_resetting = false;
            self.pid.hold();
        }
        let recovering = self.fault.is_some() || (self.desired_enabled && !self.enabled);
        if recovering {
            self.recovery_samples = self.recovery_samples.saturating_add(1);
            if self.recovery_samples < SENSOR_RECOVERY_SAMPLES { return; }
            self.fault = None;
            self.pid.reset();
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
        self.pid.reset();
        self.pwm.command(now_ms, 0.0, false);
        Ok(())
    }

    pub fn stop(&mut self) {
        if self.probe_resetting { self.last_sample = None; } // nothing left to hold or to go stale
        self.probe_resetting = false;
        self.enabled = false;
        self.desired_enabled = false;
        self.recovery_samples = 0;
        self.stopped = true;
        self.pid.reset();
        self.output_pct = None;
        self.pwm.command(0, 0.0, false);
    }

    fn desired_duty_pct(&self, now_ms: u64) -> f64 {
        if !self.enabled || self.fault.is_some() { return 0.0; }
        let max_age = if self.probe_resetting { SENSOR_RESET_MAX_AGE_MS } else { SENSOR_MAX_AGE_MS };
        let fresh = |(ms, _): (u64, i32)| now_ms >= ms && now_ms - ms < max_age;
        if !self.last_sample.is_some_and(fresh) { return 0.0; }
        self.output_pct.unwrap_or(0.0)
    }

    /// Effective, quantized duty latched into the current SSR window.
    pub fn commanded_duty_pct(&self, now_ms: u64) -> f64 {
        if self.desired_duty_pct(now_ms) <= 0.0 { return 0.0; }
        self.pwm.latched_on_ms as f64 / self.pwm.window_ms as f64 * 100.0
    }

    /// Call every loop. Safety override does not wait for a PWM boundary.
    pub fn tick(&mut self, now_ms: u64) -> bool {
        if let Some((ms, _)) = self.last_sample {
            let max_age = if self.probe_resetting { SENSOR_RESET_MAX_AGE_MS } else { SENSOR_MAX_AGE_MS };
            if now_ms < ms || now_ms - ms >= max_age {
                self.last_sample = None;
                if self.fault.is_none() { self.trip(Fault::SensorStale); }
            }
        }
        let duty = self.desired_duty_pct(now_ms);
        self.pwm.command(now_ms, duty, self.enabled && self.fault.is_none())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fixed gains the behaviour tests were written against, independent of the factory defaults.
    fn legacy() -> PidConfig { PidConfig { kp: 10.0, ki: 0.1, kd: 0.0, d_filter_s: 0.0, output_limit_pct: 100.0 } }

    fn configured() -> HeaterControl {
        HeaterControl::new(legacy(), Some(SafetyLimits { max_temperature_mc: 40_000, heater_watts: 100 }))
    }

    #[test]
    fn pid_proportional_capped_and_heat_only() {
        let mut pid = Pid::new(PidConfig { kp: 10.0, ki: 0.0, kd: 0.0, d_filter_s: 0.0, output_limit_pct: 30.0 });
        assert_eq!(pid.update(25.0, 24.0, 0), Some(10.0));
        assert_eq!(pid.update(25.0, 10.0, 1000), Some(30.0));
        assert_eq!(pid.update(25.0, 25.0, 2000), Some(0.0));
        assert_eq!(pid.update(25.0, 26.0, 3000), Some(0.0));
    }

    #[test]
    fn integral_survives_crossing_the_target_and_bleeds_down() {
        let mut pid = Pid::new(PidConfig { kp: 1.0, ki: 0.1, kd: 0.0, d_filter_s: 0.0, output_limit_pct: 100.0 });
        pid.integral = 10.0; // learned holding power
        pid.update(25.0, 25.0, 0);
        // Slightly above target: still heats with I minus P, and I decreases.
        let out = pid.update(25.0, 25.2, 5000).unwrap();
        assert!((out - (10.0 - 0.1 - 0.2)).abs() < 1e-9, "{out}");
        assert!((pid.integral - 9.9).abs() < 1e-9);
        // At target + 1 C the output is forced off but the integral is not cleared.
        assert_eq!(pid.update(25.0, 26.0, 10_000), Some(0.0));
        assert!(pid.integral > 9.0 && pid.integral < 9.9);
        // Back below target: resumes from the kept integral, no restart from zero.
        assert!(pid.update(25.0, 24.9, 15_000).unwrap() > 9.0);
    }

    #[test]
    fn integral_uses_actual_dt_and_anti_windup() {
        let mut pid = Pid::new(PidConfig { kp: 0.0, ki: 1.0, kd: 0.0, d_filter_s: 0.0, output_limit_pct: 30.0 });
        assert_eq!(pid.update(25.0, 24.0, 0), Some(0.0));
        assert_eq!(pid.update(25.0, 24.0, 2000), Some(2.0));
        let mut pid = Pid::new(PidConfig { output_limit_pct: 30.0, ..legacy() });
        for ms in (0..100_000).step_by(1000) { assert_eq!(pid.update(25.0, 10.0, ms), Some(30.0)); }
        assert_eq!(pid.integral, 0.0);
        assert!(pid.update(25.0, 24.9, 100_000).unwrap() < 2.0);
    }

    #[test]
    fn conditional_integration_freezes_updates_further_into_upper_saturation() {
        let mut pid = Pid::new(PidConfig { output_limit_pct: 30.0, ..legacy() });
        pid.integral = 5.0;
        // P=40, I=5: positive error would push the request further above the 30% cap.
        for ms in (0..=60_000).step_by(5_000) {
            assert_eq!(pid.update(25.0, 21.0, ms), Some(30.0));
            assert_eq!(pid.integral, 5.0, "no windup at {ms}");
        }
    }

    #[test]
    fn conditional_integration_freezes_updates_further_into_lower_saturation() {
        let mut pid = Pid::new(PidConfig { kp: 20.0, ki: 0.02, ..legacy() });
        pid.integral = 4.0;
        // P=-6, I=4: a negative integral step would only make the request more negative.
        // Keeping I here is intentional clamping, not a reason to clear holding power.
        for ms in (0..=3_600_000).step_by(5_000) {
            assert_eq!(pid.update(25.0, 25.3, ms), Some(0.0));
            assert_eq!(pid.integral, 4.0, "no forced integral decay at {ms}");
        }
    }

    #[test]
    fn conditional_integration_decreases_i_while_request_still_exceeds_upper_limit() {
        let mut pid = Pid::new(PidConfig { kd: 100.0, output_limit_pct: 30.0, ..legacy() });
        pid.integral = 10.0;
        pid.update(25.0, 25.9, 0);
        // Cooling 0.4 C/s gives D=40. Despite negative error and a decreasing I, the
        // requested output stays above 30%. Do not block this direction of integration.
        assert_eq!(pid.update(25.0, 25.5, 1_000), Some(30.0));
        assert!((pid.integral - 9.95).abs() < 1e-9, "{}", pid.integral);
        assert_eq!(pid.update(25.0, 25.1, 2_000), Some(30.0));
        assert!((pid.integral - 9.94).abs() < 1e-9, "{}", pid.integral);
        // Once D falls away, normal control resumes from the corrected integral.
        let out = pid.update(25.0, 25.1, 3_000).unwrap();
        assert!((out - 8.93).abs() < 1e-9, "{out}");
    }

    #[test]
    fn conditional_integration_increases_i_while_request_still_below_lower_limit() {
        let mut pid = Pid::new(PidConfig { kd: 100.0, ..legacy() });
        pid.integral = 4.0;
        pid.update(25.0, 24.0, 0);
        // Warming 0.5 C in 5 s gives D=-10. P=5 and candidate I=4.25 still request
        // -0.75%, but the positive integral step helps return towards the output range.
        assert_eq!(pid.update(25.0, 24.5, 5_000), Some(0.0));
        assert!((pid.integral - 4.25).abs() < 1e-9, "{}", pid.integral);
        let out = pid.update(25.0, 24.5, 10_000).unwrap();
        assert!((out - 9.5).abs() < 1e-9, "{out}");
    }

    #[test]
    fn conditional_integration_accepts_exact_output_bounds_and_keeps_i_bounded() {
        let cfg = PidConfig { kp: 0.0, ki: 1.0, output_limit_pct: 30.0, ..legacy() };
        let mut pid = Pid::new(cfg);
        pid.integral = 29.0;
        assert_eq!(pid.update(25.0, 24.0, 0), Some(29.0));
        assert_eq!(pid.update(25.0, 24.0, 1_000), Some(30.0));
        assert_eq!(pid.integral, 30.0);
        assert_eq!(pid.update(25.0, 24.0, 2_000), Some(30.0));
        assert_eq!(pid.integral, 30.0, "the integral's own cap is unchanged");
        assert_eq!(pid.update(25.0, 25.5, 3_000), Some(29.5));
        let mut pid = Pid::new(cfg);
        pid.integral = 0.5;
        assert_eq!(pid.update(25.0, 25.5, 0), Some(0.5));
        assert_eq!(pid.update(25.0, 25.5, 1_000), Some(0.0));
        assert_eq!(pid.integral, 0.0);
        assert_eq!(pid.update(25.0, 25.5, 2_000), Some(0.0));
        assert_eq!(pid.integral, 0.0, "the integral cannot wind down below zero");
    }

    #[test]
    fn derivative_is_on_measurement_not_setpoint() {
        let mut pid = Pid::new(PidConfig { kp: 1.0, ki: 0.0, kd: 2.0, d_filter_s: 0.0, output_limit_pct: 100.0 });
        assert_eq!(pid.update(25.0, 20.0, 0), Some(5.0));
        assert_eq!(pid.update(30.0, 20.0, 1000), Some(10.0));
        assert_eq!(pid.update(30.0, 21.0, 2000), Some(7.0));
    }

    #[test]
    fn derivative_low_pass_filters_lsb_steps() {
        // Kd 100, Tf 15 s, 5 s samples: alpha = 5 / 20 = 0.25.
        let cfg = PidConfig { kp: 0.0, ki: 0.0, kd: 100.0, d_filter_s: 15.0, output_limit_pct: 100.0 };
        let mut pid = Pid::new(cfg);
        assert_eq!(pid.update(30.0, 20.0, 0), Some(0.0));
        // Falling 0.5 C in 5 s: raw D = 100 * 0.1 = 10 %, filtered 2.5 %.
        assert_eq!(pid.update(30.0, 19.5, 5000), Some(2.5));
        // Steady: the filtered term decays instead of dropping to zero.
        assert!((pid.update(30.0, 19.5, 10_000).unwrap() - 1.875).abs() < 1e-9);
        // Unfiltered: full raw step, then zero.
        let mut raw = Pid::new(PidConfig { d_filter_s: 0.0, ..cfg });
        raw.update(30.0, 20.0, 0);
        assert_eq!(raw.update(30.0, 19.5, 5000), Some(10.0));
        assert_eq!(raw.update(30.0, 19.5, 10_000), Some(0.0));
        // A held gap restarts the filter: no stale derivative.
        pid.hold();
        assert_eq!(pid.update(30.0, 19.5, 15_000), Some(0.0));
        assert_eq!(pid.d_filter_milli(), 15_000);
        assert!(!Pid::new(PidConfig { d_filter_s: f64::NAN, ..cfg }).valid());
    }

    #[test]
    fn d_filter_setter_range_checked() {
        let mut control = HeaterControl::new(legacy(), None);
        assert_eq!(HeaterControl::new(PidConfig::default(), None).d_filter_milli(), DEFAULT_D_FILTER_MILLI);
        assert!(control.set_d_filter(0));
        assert!(control.set_d_filter(*D_FILTER_RANGE_MILLI.end()));
        assert!(!control.set_d_filter(*D_FILTER_RANGE_MILLI.end() + 1));
        assert_eq!(control.d_filter_milli(), *D_FILTER_RANGE_MILLI.end());
    }

    #[test]
    fn invalid_numeric_and_duplicate_samples_rejected() {
        let mut pid = Pid::new(legacy());
        assert_eq!(pid.update(f64::NAN, 20.0, 0), None);
        assert_eq!(pid.update(25.0, f64::INFINITY, 0), None);
        assert!(pid.update(25.0, 20.0, 1000).is_some());
        assert_eq!(pid.update(25.0, 20.0, 1000), None);
        assert_eq!(pid.update(25.0, 20.0, 999), None);
        let mut pid = Pid::new(PidConfig { kp: -1.0, ..legacy() });
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
        let mut control = HeaterControl::new(legacy(), None);
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

    /// Heating at 20 C towards 25 C: returns the control after `n` 5 s samples (integral built up).
    fn heating_with_integral(n: u64) -> (HeaterControl, u64) {
        let mut control = configured();
        control.sample(20_000, 0);
        control.start(0).unwrap();
        let mut ms = 0;
        for _ in 0..n { ms += 5_000; control.sample(20_000, ms); }
        (control, ms)
    }

    /// Runs `tick` every 100 ms from `from` to `to`; returns how many ticks had the SSR closed.
    fn run_ticks(control: &mut HeaterControl, from: u64, to: u64) -> (u64, u64) {
        let (mut on, mut total) = (0, 0);
        let mut ms = from;
        while ms < to { ms += 100; total += 1; if control.tick(ms) { on += 1; } }
        (on, total)
    }

    #[test]
    fn probe_reset_keeps_the_heater_running_without_a_fault() {
        let (mut control, ms) = heating_with_integral(8);
        let duty = control.commanded_duty_pct(ms);
        let before = control.pid_output_pct().unwrap();
        run_ticks(&mut control, ms, ms + 2_000);
        assert!(before > 50.0 && control.commanded_duty_pct(ms + 2_000) > 0.0 && duty == 0.0);

        control.sensor_resetting();
        assert!(control.holding_through_reset());
        assert_eq!(control.fault(), None, "a reset in progress is not a fault");
        assert_eq!(control.mode(), "pid", "nothing changes for the user");
        assert_eq!(control.pid_output_pct(), Some(before), "same output");
        // 40 s without a sample (the whole ladder, far beyond the usual 12 s) and the SSR keeps
        // switching at the same duty: duty = commanded on-time / window.
        let (on, total) = run_ticks(&mut control, ms + 2_000, ms + 42_000);
        assert_eq!(control.fault(), None);
        assert_eq!(control.mode(), "pid");
        let ratio = on as f64 / total as f64 * 100.0;
        assert!((ratio - control.commanded_duty_pct(ms + 42_000)).abs() < 5.0, "ratio {ratio}");
        assert!(ratio > 20.0, "the heater really keeps heating: {ratio}");
    }

    #[test]
    fn reset_preserves_every_pwm_edge_and_cannot_extend_the_hold_deadline() {
        let (mut control, ms) = heating_with_integral(8);
        control.tick(ms);
        let output = control.pid_output_pct().unwrap();
        let mut expected = WindowPwm {
            window_start_ms: control.pwm.window_start_ms,
            latched_on_ms: control.pwm.latched_on_ms,
            window_ms: PWM_WINDOW_MS,
        };
        control.sensor_resetting();
        for age in (5..SENSOR_RESET_MAX_AGE_MS).step_by(5) {
            if age == 7_000 || age == 16_000 { control.sensor_resetting(); }
            assert_eq!(control.tick(ms + age), expected.command(ms + age, output, true), "PWM edge at {age}");
            assert_eq!(control.fault(), None);
        }
        assert_eq!(control.commanded_duty_pct(ms + SENSOR_RESET_MAX_AGE_MS), 0.0, "read-only duty also enforces age limit");
        assert!(!control.tick(ms + SENSOR_RESET_MAX_AGE_MS));
        assert_eq!(control.fault(), Some(Fault::SensorStale));
        assert_eq!(control.pid_integral_pct(), 0.0);
        control.sensor_resetting();
        assert!(!control.tick(ms + SENSOR_RESET_MAX_AGE_MS + 1_000), "another reset cannot re-arm heating");
    }

    #[test]
    fn a_late_sample_and_reversed_clock_cannot_bypass_recovery_deadline() {
        let (mut control, ms) = heating_with_integral(8);
        control.sensor_resetting();
        control.sample(20_000, ms + SENSOR_RESET_MAX_AGE_MS);
        assert_eq!(control.fault(), Some(Fault::SensorStale));
        assert_eq!(control.recovery_samples(), 1);
        assert!(!control.tick(ms + SENSOR_RESET_MAX_AGE_MS));
        let (mut control, ms) = heating_with_integral(8);
        control.sensor_resetting();
        assert!(!control.tick(ms - 1));
        assert_eq!(control.fault(), Some(Fault::SensorStale));
    }

    #[test]
    fn raw_overtemperature_during_recovery_and_failed_reset_keep_hysteresis() {
        let (mut control, ms) = heating_with_integral(8);
        control.sensor_resetting();
        control.recovery_raw_reading(41_000);
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert!(!control.tick(ms + 1));
        control.sensor_error();
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        for age in [5_000, 10_000, 15_000] { control.sample(38_000, ms + age); }
        assert_eq!(control.fault(), Some(Fault::OverTemperature), "38 C is above the 37 C recovery threshold");
        control.tick(ms + 30_000);
        assert_eq!(control.fault(), Some(Fault::OverTemperature), "staleness must not erase hysteresis either");
    }

    #[test]
    fn disabling_integral_action_clears_residual_heat() {
        let (mut control, ms) = heating_with_integral(8);
        assert!(control.pid_integral_pct() > 10.0);
        assert!(control.set_pid_gains(0, 0, 0));
        assert_eq!(control.pid_integral_pct(), 0.0);
        control.sample(20_000, ms + 5_000);
        assert_eq!(control.pid_output_pct(), Some(0.0));
        assert!(!control.tick(ms + 5_000));
    }

    #[test]
    fn first_sample_after_the_reset_resumes_control_with_the_integral_kept() {
        let (mut control, mut ms) = heating_with_integral(8);
        let integral = control.pid_integral_pct();
        assert!(integral > 10.0, "integral built up: {integral}");
        let before = control.pid_output_pct().unwrap();
        control.sensor_resetting();
        run_ticks(&mut control, ms, ms + 30_000); // far beyond the usual 12 s
        ms += 30_000;
        control.sample(20_000, ms);
        assert!(!control.holding_through_reset());
        assert_eq!(control.mode(), "pid");
        assert_eq!(control.pid_integral_pct(), integral, "the gap itself is not integrated");
        let after = control.pid_output_pct().unwrap();
        assert!((after - before).abs() < 1.0, "output continues at {before}, got {after}");
        // And normal 5 s control goes on from there.
        ms += 5_000;
        control.sample(20_000, ms);
        assert!(control.pid_integral_pct() > integral);
    }

    #[test]
    fn failed_probe_reset_cuts_the_heater_and_wipes_the_pid() {
        let (mut control, ms) = heating_with_integral(8);
        assert!(control.pid_integral_pct() > 10.0);
        control.sensor_resetting();
        run_ticks(&mut control, ms, ms + 30_000);
        assert!(control.tick(ms + 30_000) || control.commanded_duty_pct(ms + 30_000) > 0.0, "still heating until the ladder gives up");
        control.sensor_error(); // every reset failed: now it is a fault
        assert_eq!(control.fault(), Some(Fault::SensorError));
        assert!(!control.tick(ms + 30_100));
        assert_eq!(control.commanded_duty_pct(ms + 30_100), 0.0);
        assert_eq!(control.pid_integral_pct(), 0.0);
        assert!(!control.holding_through_reset());
        // Recovery as before: 3 good windows, PID from scratch (P term only).
        let mut t = ms + 30_100;
        for _ in 0..3 { t += 5_000; control.sample(20_000, t); }
        assert_eq!(control.mode(), "pid");
        assert_eq!(control.pid_output_pct(), Some(50.0));
    }

    #[test]
    fn hold_does_not_outlive_a_user_action() {
        for action in 0..2 {
            let (mut control, ms) = heating_with_integral(8);
            control.sensor_resetting();
            if action == 0 { control.stop(); } else { control.sensor_error(); }
            assert!(!control.holding_through_reset(), "action {action}");
            assert_eq!(control.pid_integral_pct(), 0.0, "action {action}");
            assert!(!control.tick(ms + 30_000), "no heating on a stale sample after action {action}");
        }
        // A new target during the reset starts the PID clean and keeps the heater off until the
        // next sample, without turning the reset into a fault.
        let (mut control, ms) = heating_with_integral(8);
        control.sensor_resetting();
        assert!(control.set_target(26_000));
        assert_eq!(control.pid_integral_pct(), 0.0);
        assert!(!control.tick(ms + 30_000));
        assert_eq!(control.fault(), None);
    }

    #[test]
    fn reset_while_not_heating_holds_the_heater_off_without_a_fault() {
        let mut control = configured();
        control.sample(20_000, 0);
        control.sensor_resetting(); // never started: nothing to hold
        assert!(!control.holding_through_reset());
        assert_eq!(control.fault(), None);
        assert_eq!(control.mode(), "standby");
        assert_eq!(control.pid_integral_pct(), 0.0);
        // Armed but waiting for 3 good windows (e.g. just after a boot): stays off.
        let mut control = configured();
        control.restore_desired(true);
        control.sample(20_000, 0);
        control.sensor_resetting();
        assert_eq!(control.mode(), "standby");
        assert!(!control.tick(1000));
        for ms in [5_000, 10_000, 15_000] { control.sample(20_000, ms); }
        assert_eq!(control.mode(), "pid");
    }

    #[test]
    fn temperature_well_above_target_after_the_reset_cuts_heat() {
        let (mut control, mut ms) = heating_with_integral(8);
        control.sensor_resetting();
        ms += 20_000;
        control.sample(26_000, ms); // warmed up meanwhile
        assert_eq!(control.pid_output_pct(), Some(0.0), "no heat at target + 1 C");
    }

    #[test]
    fn over_temperature_after_the_reset_still_trips() {
        let (mut control, mut ms) = heating_with_integral(8);
        control.sensor_resetting();
        ms += 20_000;
        control.sample(41_000, ms);
        assert_eq!(control.fault(), Some(Fault::OverTemperature));
        assert!(!control.tick(ms + 1));
        assert!(!control.holding_through_reset());
    }

    #[test]
    fn pid_gains_are_validated_and_applied() {
        let mut control = configured();
        assert_eq!(control.pid_gains_milli(), (10_000, 100, 0));
        for bad in [(100_001, 100, 0), (10_000, 2_001, 0), (10_000, 100, 200_001)] {
            assert!(!control.set_pid_gains(bad.0, bad.1, bad.2), "{bad:?}");
        }
        assert_eq!(control.pid_gains_milli(), (10_000, 100, 0), "rejected values change nothing");
        assert!(control.set_pid_gains(20_000, 0, 0));
        assert_eq!(control.pid_gains_milli(), (20_000, 0, 0));
        control.sample(20_000, 0);
        control.start(0).unwrap();
        control.sample(20_000, 5_000);
        control.sample(20_000, 10_000);
        assert_eq!(control.pid_output_pct(), Some(100.0), "Kp 20 x 5 C = 100%, no integral at Ki 0");
        assert!(control.set_pid_gains(1_500, 0, 0));
        control.sample(20_000, 15_000);
        assert_eq!(control.pid_output_pct(), Some(7.5), "the new gain applies at the next sample");
        assert!(control.set_pid_gains(0, 0, 0));
        control.sample(20_000, 20_000);
        assert_eq!(control.pid_output_pct(), Some(0.0));
        assert!(control.set_pid_gains(100_000, 2_000, 200_000), "range limits are inclusive");
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
        let mut control = HeaterControl::new(legacy(), None);
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
        let mut none = HeaterControl::new(legacy(), None);
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
        let mut control = HeaterControl::new(legacy(),
            Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }));
        assert!(control.set_target(30_000));
        assert!(control.safety_configured());
        let mut tight = HeaterControl::new(legacy(),
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

#[cfg(test)]
mod default_tests {
    use super::*;

    #[test]
    fn factory_defaults_are_valid_and_in_range() {
        let pid = Pid::new(PidConfig::default());
        assert!(pid.valid());
        assert_eq!(pid.gains_milli(), (DEFAULT_KP_MILLI, DEFAULT_KI_MILLI, DEFAULT_KD_MILLI));
        assert!(KP_RANGE_MILLI.contains(&DEFAULT_KP_MILLI) && KI_RANGE_MILLI.contains(&DEFAULT_KI_MILLI)
            && KD_RANGE_MILLI.contains(&DEFAULT_KD_MILLI) && D_FILTER_RANGE_MILLI.contains(&DEFAULT_D_FILTER_MILLI));
        assert_eq!(pid.d_filter_milli(), DEFAULT_D_FILTER_MILLI);
    }

    #[test]
    fn default_d_term_on_one_lsb_step_is_small() {
        // One 0.0625 C drop between two 5 s samples at the setpoint offset.
        let mut pid = Pid::new(PidConfig::default());
        pid.update(25.0, 24.9375, 0);
        let base = pid.update(25.0, 24.9375, 5000).unwrap();
        let stepped = pid.update(25.0, 24.875, 10_000).unwrap();
        let d_kick = stepped - base - 20.0 * 0.0625; // remove the P change
        assert!(d_kick > 0.0 && d_kick < 0.3, "filtered D kick {d_kick}");
    }
}
