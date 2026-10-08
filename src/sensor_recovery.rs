//! Hardware recovery for a DS18B20 that stops answering.
//!
//! Failure seen on this build: the probe's 1-Wire interface keeps answering (presence and CRC are
//! fine) but its conversion core is latched up. Nothing sent on the bus clears that; only removing
//! VDD does. The probe's VDD is therefore fed from a GPIO and this state machine decides when to
//! switch it off.
//!
//! Policy: a whole averaging window without a valid reading starts a reset ladder (VDD off for
//! 1 s, then 3 s, then 10 s). That is *not* a fault: the heater is held off meanwhile
//! (`HeaterControl::sensor_resetting`) but nothing is reported as failed. The first valid reading
//! ends the ladder. Only when every attempt has failed is the sensor declared faulty; the power
//! cycle then repeats once a minute, so a probe that comes back heals itself.
//!
//! Pure logic (no HAL, no clock): the caller feeds in events with timestamps and executes the power
//! cycles it asks for.

/// Power cycles tried before the sensor is reported faulty.
pub const MAX_ATTEMPTS: u8 = 3;
/// VDD-off time of attempt 1, 2 and 3 (ms). Later attempts stay off longer in case something
/// holds charge.
pub const OFF_MS: [u64; MAX_ATTEMPTS as usize] = [1_000, 3_000, 10_000];
/// After the off time the probe has this long to deliver a valid reading before the attempt
/// counts as failed. Re-initialisation plus one 12-bit conversion take about 1 s.
pub const RESPONSE_MS: u64 = 6_000;
/// Once every attempt has failed (fault reported) the power cycle repeats this often.
pub const RETRY_MS: u64 = 60_000;
/// VDD-off time of those periodic retries (ms).
pub const RETRY_OFF_MS: u64 = 3_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Probe answering (or not judged yet).
    Healthy,
    /// Reset attempt `n` (1..=`MAX_ATTEMPTS`) is under way. Not a fault.
    Resetting(u8),
    /// Every attempt failed: report a fault. Power cycles continue every `RETRY_MS`.
    Failed,
}

/// What just changed; the caller applies it to the heater control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A reset attempt started: hold the heater off, but do not raise a fault.
    AttemptStarted(u8),
    /// Every attempt failed: raise the sensor fault.
    GaveUp,
    /// The probe delivered a valid reading again.
    Recovered,
}

/// One power cycle for the sensor task to perform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PowerCycle {
    /// How long VDD stays off (ms).
    pub off_ms: u64,
    /// 1..=`MAX_ATTEMPTS` on the reset ladder; 0 for the periodic retry after giving up.
    pub attempt: u8,
}

pub struct SensorRecovery {
    phase: Phase,
    /// Resetting: when the current attempt is judged failed. Failed: when the next retry is due.
    deadline_ms: u64,
    pending: Option<PowerCycle>,
    power_cycles_total: u32,
    recoveries_total: u32,
}

impl SensorRecovery {
    pub const fn new() -> Self {
        Self { phase: Phase::Healthy, deadline_ms: 0, pending: None, power_cycles_total: 0, recoveries_total: 0 }
    }

    pub fn phase(&self) -> Phase { self.phase }
    /// A reset attempt is under way (not a fault).
    pub fn resetting(&self) -> bool { matches!(self.phase, Phase::Resetting(_)) }
    /// 1..=`MAX_ATTEMPTS` while resetting, otherwise 0.
    pub fn attempt(&self) -> u8 {
        match self.phase {
            Phase::Resetting(n) => n,
            _ => 0,
        }
    }
    /// Power cycles actually performed since boot.
    pub fn power_cycles_total(&self) -> u32 { self.power_cycles_total }
    /// Times the probe answered again after a reset or a fault.
    pub fn recoveries_total(&self) -> u32 { self.recoveries_total }
    pub fn power_cycle_pending(&self) -> bool { self.pending.is_some() }

    /// Hands the sensor task its next power cycle; each request is handed out once.
    pub fn take_power_cycle(&mut self) -> Option<PowerCycle> {
        let cycle = self.pending.take()?;
        self.power_cycles_total = self.power_cycles_total.saturating_add(1);
        Some(cycle)
    }

    /// A whole averaging window passed without a valid reading. Starts the ladder; ignored while
    /// a ladder is running or the fault is already reported (time decides those).
    pub fn window_failed(&mut self, now_ms: u64) -> Option<Event> {
        if self.phase != Phase::Healthy { return None; }
        Some(self.start_attempt(1, now_ms))
    }

    /// A valid reading arrived: the probe works, so no further power cycle is needed.
    pub fn good_read(&mut self) -> Option<Event> {
        if self.phase == Phase::Healthy { return None; }
        self.phase = Phase::Healthy;
        self.pending = None;
        self.recoveries_total = self.recoveries_total.saturating_add(1);
        Some(Event::Recovered)
    }

    /// Time-driven progress: ends an attempt that did not help, schedules the periodic retry
    /// once the fault is reported. Cheap; call it often.
    pub fn poll(&mut self, now_ms: u64) -> Option<Event> {
        if now_ms < self.deadline_ms { return None; }
        match self.phase {
            Phase::Healthy => None,
            Phase::Resetting(n) if n < MAX_ATTEMPTS => Some(self.start_attempt(n + 1, now_ms)),
            Phase::Resetting(_) => {
                self.phase = Phase::Failed;
                self.deadline_ms = now_ms.saturating_add(RETRY_MS);
                Some(Event::GaveUp)
            }
            Phase::Failed => {
                self.pending = Some(PowerCycle { off_ms: RETRY_OFF_MS, attempt: 0 });
                self.deadline_ms = now_ms.saturating_add(RETRY_MS);
                None
            }
        }
    }

    fn start_attempt(&mut self, attempt: u8, now_ms: u64) -> Event {
        let off_ms = OFF_MS[usize::from(attempt - 1)];
        self.phase = Phase::Resetting(attempt);
        self.deadline_ms = now_ms.saturating_add(off_ms + RESPONSE_MS);
        self.pending = Some(PowerCycle { off_ms, attempt });
        Event::AttemptStarted(attempt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drives a recovery to `Failed`; returns the time at which it gave up.
    fn give_up(r: &mut SensorRecovery, t0: u64) -> u64 {
        r.window_failed(t0);
        r.take_power_cycle();
        let t2 = t0 + 7_000;
        assert_eq!(r.poll(t2), Some(Event::AttemptStarted(2)));
        r.take_power_cycle();
        let t3 = t2 + 9_000;
        assert_eq!(r.poll(t3), Some(Event::AttemptStarted(3)));
        r.take_power_cycle();
        let tf = t3 + 16_000;
        assert_eq!(r.poll(tf), Some(Event::GaveUp));
        tf
    }

    #[test]
    fn healthy_probe_asks_for_nothing() {
        let mut r = SensorRecovery::new();
        assert_eq!(r.phase(), Phase::Healthy);
        assert_eq!(r.poll(1_000_000), None);
        assert_eq!(r.good_read(), None);
        assert_eq!(r.take_power_cycle(), None);
        assert_eq!((r.power_cycles_total(), r.recoveries_total(), r.attempt()), (0, 0, 0));
    }

    #[test]
    fn failed_window_starts_the_first_attempt_once() {
        let mut r = SensorRecovery::new();
        assert_eq!(r.window_failed(5_000), Some(Event::AttemptStarted(1)));
        assert_eq!(r.phase(), Phase::Resetting(1));
        assert!(r.resetting());
        assert_eq!(r.attempt(), 1);
        assert!(r.power_cycle_pending());
        assert_eq!(r.power_cycles_total(), 0, "counted when performed, not when requested");
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 1_000, attempt: 1 }));
        assert_eq!(r.take_power_cycle(), None, "each request is handed out once");
        assert_eq!(r.power_cycles_total(), 1);
        // The windows that fail while the reset is under way change nothing.
        assert_eq!(r.window_failed(10_000), None);
        assert_eq!(r.phase(), Phase::Resetting(1));
        assert!(!r.power_cycle_pending());
    }

    #[test]
    fn first_valid_reading_ends_the_reset() {
        let mut r = SensorRecovery::new();
        r.window_failed(5_000);
        r.take_power_cycle();
        assert_eq!(r.good_read(), Some(Event::Recovered));
        assert_eq!(r.phase(), Phase::Healthy);
        assert_eq!(r.recoveries_total(), 1);
        assert_eq!(r.poll(1_000_000), None, "a recovered probe is not escalated later");
        assert_eq!(r.good_read(), None);
        // A later failure is a new incident and starts from attempt 1 again.
        assert_eq!(r.window_failed(60_000), Some(Event::AttemptStarted(1)));
    }

    #[test]
    fn valid_reading_before_the_cycle_ran_cancels_it() {
        let mut r = SensorRecovery::new();
        r.window_failed(5_000);
        assert!(r.power_cycle_pending());
        assert_eq!(r.good_read(), Some(Event::Recovered));
        assert!(!r.power_cycle_pending());
        assert_eq!(r.take_power_cycle(), None);
        assert_eq!(r.power_cycles_total(), 0, "a working probe is never power-cycled");
    }

    #[test]
    fn attempts_escalate_then_give_up() {
        let mut r = SensorRecovery::new();
        let t0 = 100_000;
        assert_eq!(r.window_failed(t0), Some(Event::AttemptStarted(1)));
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 1_000, attempt: 1 }));
        // attempt 1: 1 s off + 6 s to answer
        assert_eq!(r.poll(t0 + 6_999), None);
        assert_eq!(r.poll(t0 + 7_000), Some(Event::AttemptStarted(2)));
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 3_000, attempt: 2 }));
        // attempt 2: 3 s off + 6 s
        assert_eq!(r.poll(t0 + 7_000 + 8_999), None);
        assert_eq!(r.poll(t0 + 7_000 + 9_000), Some(Event::AttemptStarted(3)));
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 10_000, attempt: 3 }));
        // attempt 3: 10 s off + 6 s, then the fault is reported
        assert_eq!(r.poll(t0 + 16_000 + 15_999), None);
        assert_eq!(r.phase(), Phase::Resetting(3), "still not a fault until the last attempt fails");
        assert_eq!(r.poll(t0 + 16_000 + 16_000), Some(Event::GaveUp));
        assert_eq!(r.phase(), Phase::Failed);
        assert!(!r.resetting());
        assert_eq!(r.attempt(), 0);
        assert_eq!(r.power_cycles_total(), 3);
        assert!(!r.power_cycle_pending());
    }

    #[test]
    fn after_giving_up_it_keeps_retrying_every_minute() {
        let mut r = SensorRecovery::new();
        let tf = give_up(&mut r, 100_000);
        assert_eq!(r.window_failed(tf + 1), None, "windows keep failing; the fault is already reported");
        assert_eq!(r.poll(tf + 59_999), None);
        assert!(!r.power_cycle_pending());
        assert_eq!(r.poll(tf + 60_000), None, "a retry is a request, not an event");
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 3_000, attempt: 0 }));
        assert_eq!(r.phase(), Phase::Failed, "retrying does not clear the fault");
        assert_eq!(r.poll(tf + 119_999), None);
        assert!(!r.power_cycle_pending());
        r.poll(tf + 120_000);
        assert_eq!(r.take_power_cycle(), Some(PowerCycle { off_ms: 3_000, attempt: 0 }));
        assert_eq!(r.power_cycles_total(), 5);
    }

    #[test]
    fn probe_that_returns_after_giving_up_recovers_and_stops_retrying() {
        let mut r = SensorRecovery::new();
        let tf = give_up(&mut r, 100_000);
        assert_eq!(r.good_read(), Some(Event::Recovered));
        assert_eq!(r.phase(), Phase::Healthy);
        assert_eq!(r.recoveries_total(), 1);
        assert_eq!(r.poll(tf + 10 * RETRY_MS), None);
        assert!(!r.power_cycle_pending());
    }

    #[test]
    fn huge_timestamps_do_not_overflow() {
        let mut r = SensorRecovery::new();
        assert_eq!(r.window_failed(u64::MAX - 1), Some(Event::AttemptStarted(1)));
        assert_eq!(r.poll(u64::MAX), Some(Event::AttemptStarted(2)));
    }
}
