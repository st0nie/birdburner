//! Display-only last-good cache. NEVER use this cache as a PID measurement.

pub const ERROR_DISPLAY_THRESHOLD: u32 = 5;

/// Rounded two-decimal display, not a claim of 0.01 C sensor accuracy.
pub struct DisplayTemperature(pub i32);
impl core::fmt::Display for DisplayTemperature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let centi = (self.0.unsigned_abs() + 5) / 10;
        write!(f, "{}{}.{:02}", if self.0 < 0 && centi != 0 { "-" } else { "" }, centi / 100, centi % 100)
    }
}

#[derive(Default)]
pub struct SensorDisplay {
    last_good_mc: Option<i32>,
    consecutive_errors: u32,
}

impl SensorDisplay {
    pub fn success(&mut self, temperature_mc: i32) {
        self.last_good_mc = Some(temperature_mc);
        self.consecutive_errors = 0;
    }

    pub fn error(&mut self) {
        self.consecutive_errors = self.consecutive_errors.saturating_add(1);
    }

    pub fn last_good_mc(&self) -> Option<i32> { self.last_good_mc }
    pub fn consecutive_errors(&self) -> u32 { self.consecutive_errors }
    pub fn stale(&self) -> bool { self.consecutive_errors > 0 }
    pub fn show_error(&self) -> bool { self.consecutive_errors >= ERROR_DISPLAY_THRESHOLD }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_two_decimals_rounded_not_truncated() {
        assert_eq!(DisplayTemperature(23_562).to_string(), "23.56");
        assert_eq!(DisplayTemperature(23_687).to_string(), "23.69");
        assert_eq!(DisplayTemperature(23_999).to_string(), "24.00");
        assert_eq!(DisplayTemperature(-62).to_string(), "-0.06");
        assert_eq!(DisplayTemperature(0).to_string(), "0.00");
    }

    #[test]
    fn initial_state_has_no_fake_temperature() {
        let cache = SensorDisplay::default();
        assert_eq!(cache.last_good_mc(), None);
        assert!(!cache.stale());
        assert!(!cache.show_error());
    }

    #[test]
    fn four_errors_keep_cache_then_fifth_shows_error() {
        let mut cache = SensorDisplay::default();
        cache.success(23_562);
        for count in 1..=4 {
            cache.error();
            assert_eq!(cache.consecutive_errors(), count);
            assert_eq!(cache.last_good_mc(), Some(23_562));
            assert!(cache.stale());
            assert!(!cache.show_error());
        }
        cache.error();
        assert!(cache.show_error());
        assert_eq!(cache.last_good_mc(), Some(23_562));
    }

    #[test]
    fn success_resets_consecutive_errors_and_updates_cache() {
        let mut cache = SensorDisplay::default();
        for _ in 0..8 { cache.error(); }
        assert!(cache.show_error());
        cache.success(24_000);
        assert_eq!(cache.consecutive_errors(), 0);
        assert!(!cache.stale());
        assert!(!cache.show_error());
        assert_eq!(cache.last_good_mc(), Some(24_000));
        cache.error();
        assert_eq!(cache.consecutive_errors(), 1);
        assert!(!cache.show_error());
    }
}
