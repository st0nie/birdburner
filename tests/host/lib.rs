//! Host-side tests for the firmware's pure logic (no ESP hardware needed).
extern crate alloc;
#[path = "../../src/ds18b20_data.rs"] pub mod ds18b20_data;
#[path = "../../src/heater_control.rs"] pub mod heater_control;
#[path = "../../src/persist.rs"] pub mod persist;
#[path = "../../src/sensor_display.rs"] pub mod sensor_display;
#[path = "../../src/http_api.rs"] pub mod http_api;

#[cfg(test)]
mod tests {
    use super::{heater_control::SafetyLimits, http_api::AppState, persist::Settings};

    const LIMITS: Option<SafetyLimits> = Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 });

    #[test]
    fn display_cache_star_then_sensor_error_after_five() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.sample(23_562, 100);
        assert_eq!(app.temp_text(), "23.56 C");
        for n in 1..=4 {
            app.window_failed(100 + n);
            assert_eq!(app.temp_text(), "23.56 C*");
            assert_eq!(app.temperature_mc, None);
        }
        app.window_failed(105);
        assert_eq!(app.temp_text(), "Sensor Error");
        app.sample(24_000, 900);
        assert_eq!(app.temp_text(), "24.00 C");
    }

    #[test]
    fn window_average_ignores_failed_reads_and_rounds() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.raw_reading(25_000, 0);
        app.raw_error(500, "crc");
        app.raw_reading(25_125, 1000);
        app.raw_error(1500, "crc");
        app.raw_reading(25_062, 2000);
        assert_eq!(app.temperature_mc, None, "no sample before the window closes");
        app.tick(4999);
        assert_eq!(app.temperature_mc, None);
        app.tick(5000);
        assert_eq!(app.temperature_mc, Some(25_062), "(25000+25125+25062)/3 rounded");
        assert_eq!(app.last_window, (3, 2));
        assert_eq!(app.control.fault(), None, "failed reads inside a good window are not faults");
        assert_eq!(app.temp_text(), "25.06 C");
    }

    #[test]
    fn mostly_failing_sensor_still_controls_if_one_read_per_window_succeeds() {
        let mut app = AppState::new(LIMITS, Settings { desired_enabled: true, ..Settings::default() });
        let mut ms = 0;
        for _ in 0..6 {
            for _ in 0..20 { app.raw_error(ms, "crc"); ms += 200; }
            app.raw_reading(22_000, ms);
            ms += 1000;
            app.tick(ms);
        }
        assert_eq!(app.control.fault(), None);
        assert_eq!(app.control.mode(), "pid");
        assert!(!app.temp_text().contains('*'));
    }

    #[test]
    fn whole_window_without_reads_is_a_failure_even_if_sensor_task_hangs() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.raw_reading(22_000, 0);
        app.tick(5000);
        assert_eq!(app.temperature_mc, Some(22_000));
        app.tick(10_000); // nothing arrived in this window
        assert_eq!(app.temperature_mc, None);
        assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"));
        assert_eq!(app.temp_text(), "22.00 C*");
    }

    #[test]
    fn negative_average_rounds_away_from_zero_symmetrically() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.raw_reading(-62, 0);
        app.raw_reading(-63, 1);
        app.tick(5000);
        assert_eq!(app.temperature_mc, Some(-63));
    }

    #[test]
    fn settings_dirty_only_on_real_changes() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.sample(20_000, 0);
        assert!(!app.settings_dirty, "a measurement must not trigger a flash write");
        app.control.start(0).unwrap(); // direct call bypasses mutate: not dirty
        assert!(!app.settings_dirty);
        app.window_failed(10); // faults are never persisted
        assert!(!app.settings_dirty);
    }

    #[test]
    fn power_cycle_resumes_after_fresh_samples_only() {
        let saved = Settings { target_mc: 26_000, desired_enabled: true, ..Settings::default() };
        let mut app = AppState::new(LIMITS, saved);
        assert_eq!(app.control.target_mc(), 26_000);
        assert!(!app.tick(0), "heater must be off at boot");
        app.sample(22_000, 100);
        app.sample(22_000, 900);
        assert!(!app.tick(900));
        app.sample(22_000, 1700);
        assert_eq!(app.control.mode(), "pid");
        assert!(app.tick(1700));
    }

    #[test]
    fn saved_limit_restored_and_changes_mark_dirty() {
        let saved = Settings { target_mc: 22_000, desired_enabled: false, max_temperature_mc: 26_000, max_output_pct: 60 };
        let mut app = AppState::new(LIMITS, saved);
        assert_eq!(app.control.max_temperature_mc(), Some(26_000));
        assert_eq!(app.control.max_output_pct(), 60);
        assert_eq!(app.settings(), saved);
        assert!(!app.settings_dirty);
        app.sample(25_000, 0); // below a 26 C limit: fine
        assert_eq!(app.control.fault(), None);
        app.sample(26_000, 800);
        assert_eq!(app.control.fault().map(|f| f.label()), Some("over_temperature"));
    }

    #[test]
    fn overtemp_is_not_persisted_and_keeps_user_choice() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.sample(22_000, 0);
        app.control.start(0).unwrap();
        app.sample(35_000, 800);
        assert_eq!(app.control.mode(), "fault");
        assert!(!app.tick(800));
        assert!(app.settings().desired_enabled, "a fault must not flip the saved on/off choice");
        for ms in [1600, 2400, 3200] { app.sample(25_000, ms); }
        assert_eq!(app.control.mode(), "pid");
    }
}

#[cfg(test)]
mod metrics_tests {
    use super::{heater_control::SafetyLimits, http_api::AppState, persist::Settings};

    #[test]
    fn metrics_format_and_absent_temperature() {
        let mut app = AppState::new(Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }), Settings::default());
        let m = app.metrics();
        assert!(m.contains("# TYPE birdburner_temperature_celsius gauge"));
        assert!(!m.lines().any(|l| l.starts_with("birdburner_temperature_celsius ")), "no reading -> no sample line");
        assert!(m.contains("birdburner_mode{mode=\"stopped\"} 1"));
        assert!(m.contains("birdburner_fault{fault=\"none\"} 1"));
        app.sample(25_437, 1000);
        let m = app.metrics();
        assert!(m.contains("birdburner_temperature_celsius 25.437\n"));
        assert!(m.contains("birdburner_max_temperature_celsius 35\n"));
        assert!(m.contains("birdburner_heater_rated_watts 100\n"));
        app.raw_error(1100, "crc");
        app.window_failed(1100);
        let m = app.metrics();
        assert!(m.contains("birdburner_sensor_errors_total 1\n"));
        assert!(m.contains("birdburner_sensor_errors_by_kind_total{kind=\"crc\"} 1\n"));
        assert!(m.contains("birdburner_fault{fault=\"sensor_error\"} 1"));
        for line in m.lines().filter(|l| !l.starts_with('#')) {
            let (name, value) = line.rsplit_once(' ').unwrap();
            assert!(name.starts_with("birdburner_"), "{line}");
            assert!(value.parse::<f64>().is_ok(), "{line}");
        }
    }

    #[test]
    fn heater_on_time_counts_only_closed_periods() {
        let mut app = AppState::new(Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }), Settings::default());
        app.sample(20_000, 0);
        app.control.start(0).unwrap();
        app.sample(20_000, 500);
        assert!(app.tick(500));
        app.tick(600);
        assert_eq!(app.relay_on_ms_total, 100);
        assert_eq!(app.relay_switches_total, 1);
    }
}
