//! Host-side tests for the firmware's pure logic (no ESP hardware needed).
extern crate alloc;
#[path = "../../src/ds18b20_data.rs"] pub mod ds18b20_data;
#[path = "../../src/heater_control.rs"] pub mod heater_control;
#[path = "../../src/persist.rs"] pub mod persist;
#[path = "../../src/sensor_display.rs"] pub mod sensor_display;
#[path = "../../src/sensor_recovery.rs"] pub mod sensor_recovery;
#[path = "../../src/http_api.rs"] pub mod http_api;

#[cfg(test)]
mod http_tests;

#[cfg(test)]
mod tests {
    use super::{heater_control::SafetyLimits, http_api::AppState, persist::Settings, sensor_recovery::Phase};

    const LIMITS: Option<SafetyLimits> = Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 });

    /// Silent probe: keep ticking (as the relay task does) until every reset has failed.
    fn run_until_failed(app: &mut AppState, ms: &mut u64) {
        while app.recovery.phase() != Phase::Failed {
            *ms += 100;
            app.tick(*ms);
            assert!(*ms < 200_000, "the resets must give up");
        }
    }

    #[test]
    fn display_keeps_the_star_while_resetting_and_shows_sensor_error_only_after_resets_fail() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.sample(23_562, 100);
        assert_eq!(app.temp_text(), "23.56 C");
        for n in 1..=5 {
            app.window_failed(100 + n);
            assert_eq!(app.temp_text(), "23.56 C*", "a probe being reset keeps its last value, never 'Sensor Error'");
            assert_eq!(app.temperature_mc, None);
        }
        assert!(app.recovery.resetting());
        assert_eq!(app.control.fault(), None);
        let mut ms = 105;
        run_until_failed(&mut app, &mut ms);
        assert_eq!(app.temp_text(), "Sensor Error");
        assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"));
        app.raw_reading(24_000, ms);
        app.tick(ms + 5_000);
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
    fn whole_window_without_reads_starts_a_probe_reset_even_if_sensor_task_hangs() {
        let mut app = AppState::new(LIMITS, Settings::default());
        app.raw_reading(22_000, 0);
        app.tick(5000);
        assert_eq!(app.temperature_mc, Some(22_000));
        app.tick(10_000); // nothing arrived in this window
        assert_eq!(app.temperature_mc, None);
        assert_eq!(app.control.fault(), None, "a probe reset is tried before any fault is reported");
        assert!(app.recovery.resetting());
        assert_eq!(app.temp_text(), "22.00 C*");
        // A hung sensor task never answers: once every reset has failed it is a fault.
        let mut ms = 10_000;
        run_until_failed(&mut app, &mut ms);
        assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"));
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
        let saved = Settings { target_mc: 22_000, desired_enabled: false, max_temperature_mc: 26_000, max_output_pct: 60, ..Settings::default() };
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

    /// One healthy 5 s window: a good read at its start, then the window closes.
    fn healthy_window(app: &mut AppState, ms: &mut u64, temp_mc: i32) {
        app.raw_reading(temp_mc, *ms);
        *ms += 5_000;
        app.tick(*ms);
    }

    /// Heating (PID) at 22 C after four healthy windows; returns the app and the current time.
    fn running_app() -> (AppState, u64) {
        let mut app = AppState::new(LIMITS, Settings { desired_enabled: true, ..Settings::default() });
        let mut ms = 0;
        for _ in 0..4 { healthy_window(&mut app, &mut ms, 22_000); }
        assert_eq!(app.control.mode(), "pid");
        (app, ms)
    }

    /// Ticks every 100 ms like the relay task; from `read_at` on the probe delivers a good raw
    /// read every 700 ms. The sensor task performs every power cycle it is handed. Returns the
    /// number of ticks with the SSR closed and the power cycles `(attempt, VDD-off ms)`.
    /// Asserts `check` on every tick.
    fn run(app: &mut AppState, ms: &mut u64, until: u64, read_at: Option<u64>, check: &dyn Fn(&AppState, u64)) -> (u64, Vec<(u8, u64)>) {
        let mut closed = 0;
        let mut cycles = Vec::new();
        let mut next_read = read_at;
        while *ms < until {
            *ms += 100;
            if next_read.is_some_and(|t| *ms >= t) { app.raw_reading(22_100, *ms); next_read = Some(*ms + 700); }
            if app.tick(*ms) { closed += 1; }
            if let Some(c) = app.take_power_cycle() { cycles.push((c.attempt, c.off_ms)); }
            check(app, *ms);
        }
        (closed, cycles)
    }

    #[test]
    fn probe_reset_that_works_is_invisible_and_the_heater_never_stops() {
        let (mut app, mut ms) = running_app();
        let t0 = ms;
        run(&mut app, &mut ms, t0 + 2_000, None, &|_, _| {});
        assert!(app.control.commanded_duty_pct(ms) > 0.0);
        assert_eq!(app.status().sensor, "ok");

        // The probe latches up: the first window without any reading closes 5 s later and the
        // reset starts; the probe answers about 2.5 s after that.
        let t = t0 + 5_000;
        let (closed, cycles) = run(&mut app, &mut ms, t + 12_000, Some(t + 2_500), &|app, at| {
            assert_eq!(app.control.fault(), None, "never a fault (at {at})");
            assert_eq!(app.control.mode(), "pid", "heater stays in PID mode (at {at})");
            assert!(app.control.commanded_duty_pct(at) > 0.0, "same duty all along (at {at})");
            assert_eq!(app.status().start_blocked_by, None);
        });
        assert!(closed > 30, "the SSR keeps switching through the reset: {closed} closed ticks in 12 s");
        // The reset really happened, and it is not shown as heater state: the probe is back and
        // the next window carried it.
        assert_eq!(cycles, [(1, 1_000)]);
        assert_eq!((app.recovery.power_cycles_total(), app.recovery.recoveries_total()), (1, 1));
        assert_eq!(app.status().sensor, "ok");
        assert_eq!(app.temp_text(), "22.10 C");
        assert!(!app.control.holding_through_reset());
    }

    #[test]
    fn reset_is_diagnostics_only_while_the_heater_rides_through() {
        let (mut app, mut ms) = running_app();
        let until = ms + 5_100;
        run(&mut app, &mut ms, until, None, &|_, _| {}); // first silent window closes
        let status = app.status();
        assert_eq!((status.sensor, status.sensor_reset_attempt, status.fault, status.mode), ("resetting", 1, None, "pid"));
        assert_eq!(app.temp_text(), "22.00 C*");
        assert!(app.control.holding_through_reset());
        let m = app.metrics();
        assert!(m.contains("birdburner_sensor_resetting 1\n") && m.contains("birdburner_mode{mode=\"pid\"} 1"));
        assert!(m.contains("birdburner_fault{fault=\"none\"} 1"));
    }

    #[test]
    fn a_later_reset_can_still_save_the_day_without_a_fault_or_a_stop() {
        let (mut app, mut ms) = running_app();
        let t = ms + 5_000;
        // Reset 1 (VDD off 1 s) does not help; reset 2 (3 s) starts 7 s later and works.
        let (closed, cycles) = run(&mut app, &mut ms, t + 20_000, Some(t + 11_000), &|app, at| {
            assert_eq!(app.control.fault(), None, "never a fault (at {at})");
            assert_eq!(app.control.mode(), "pid", "(at {at})");
            assert!(app.control.commanded_duty_pct(at) > 0.0, "(at {at})");
        });
        assert!(closed > 50, "{closed}");
        assert_eq!(app.status().sensor, "ok");
        assert_eq!(app.recovery.recoveries_total(), 1);
        assert_eq!(cycles, [(1, 1_000), (2, 3_000)]);
    }

    #[test]
    fn dead_probe_heats_until_the_resets_fail_then_cuts_and_reports_the_fault() {
        let (mut app, mut ms) = running_app();
        let t = ms + 5_000; // the first window without a reading closes here
        let mut cycles = Vec::new();
        let mut closed_during_ladder = 0;
        while ms < t + 160_000 {
            ms += 100;
            let heater_on = app.tick(ms);
            if let Some(c) = app.take_power_cycle() { cycles.push((ms, c.attempt, c.off_ms)); }
            if ms < t + 32_000 {
                assert_eq!(app.control.fault(), None, "still resetting, not a fault (at {ms})");
                assert_eq!(app.control.mode(), "pid", "the heater keeps going until the last reset fails (at {ms})");
                if heater_on { closed_during_ladder += 1; }
            } else if ms > t + 32_100 {
                assert!(!heater_on, "heater cut once the resets failed (at {ms})");
                assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"), "at {ms}");
            }
        }
        assert!(closed_during_ladder > 100, "{closed_during_ladder}");
        // 3 resets (1 s, 3 s, 10 s off); the fault is reported 32 s after the first failed window;
        // from then on the power cycle repeats once a minute without clearing the fault.
        assert_eq!(cycles, [(t, 1, 1_000), (t + 7_000, 2, 3_000), (t + 16_000, 3, 10_000),
                            (t + 92_000, 0, 3_000), (t + 152_000, 0, 3_000)]);
        assert_eq!(app.temp_text(), "Sensor Error");
        assert_eq!((app.status().sensor, app.status().mode), ("error", "fault"));
        assert!(app.control.desired_enabled(), "the user's choice survives");

        // A retry frees the probe: the fault clears after 3 good windows, then heating resumes.
        assert!(app.raw_reading(22_300, ms));
        let healed_at = ms;
        for window in 1..=3u64 {
            ms = healed_at + 5_000 * window;
            app.tick(ms);
            if window < 3 {
                assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"));
                app.raw_reading(22_300, ms);
            }
        }
        assert_eq!(app.control.fault(), None);
        assert_eq!(app.control.mode(), "pid");
        assert_eq!(app.recovery.recoveries_total(), 1);
    }

    #[test]
    fn a_probe_reset_while_the_heater_is_off_shows_the_reset_and_keeps_it_off() {
        let mut app = AppState::new(LIMITS, Settings::default()); // never started
        app.sample(23_000, 0);
        app.window_failed(5_000);
        assert!(app.recovery.resetting());
        assert_eq!(app.control.fault(), None);
        assert!(!app.tick(5_100));
        assert_eq!(app.status().sensor, "resetting");
    }

    #[test]
    fn pid_gains_load_apply_and_persist() {
        use super::http_api::gain_milli;
        use super::heater_control::{KD_RANGE_MILLI, KI_RANGE_MILLI, KP_RANGE_MILLI};
        let saved = Settings { kp_milli: 25_000, ki_milli: 40, kd_milli: 5_000, ..Settings::default() };
        let mut app = AppState::new(LIMITS, saved);
        assert_eq!(app.control.pid_gains_milli(), (25_000, 40, 5_000));
        assert_eq!(app.settings(), saved);
        assert!(!app.settings_dirty);
        let st = app.status();
        assert_eq!((st.pid_kp, st.pid_ki, st.pid_kd), (25.0, 0.04, 5.0));
        let m = app.metrics();
        assert!(m.contains("birdburner_pid_kp 25\n") && m.contains("birdburner_pid_ki 0.04\n") && m.contains("birdburner_pid_kd 5\n"));
        // A change marks the settings dirty so the storage task writes them to flash.
        assert!(app.mutate_gains(12_345, 2_000, 0));
        assert!(app.settings_dirty);
        assert_eq!((app.settings().kp_milli, app.settings().ki_milli, app.settings().kd_milli), (12_345, 2_000, 0));
        // Rejected input changes nothing and writes nothing.
        app.settings_dirty = false;
        assert!(!app.mutate_gains(100_001, 0, 0));
        assert!(!app.settings_dirty);
        assert_eq!(app.control.pid_gains_milli(), (12_345, 2_000, 0));
        // JSON numbers -> thousandths.
        assert_eq!(gain_milli(10.0, KP_RANGE_MILLI), Some(10_000));
        assert_eq!(gain_milli(0.1, KI_RANGE_MILLI), Some(100));
        assert_eq!(gain_milli(0.0005, KI_RANGE_MILLI), Some(1), "rounds to the nearest 0.001");
        assert_eq!(gain_milli(100.0, KP_RANGE_MILLI), Some(100_000));
        assert_eq!(gain_milli(200.0, KD_RANGE_MILLI), Some(200_000));
        for bad in [-0.001, 100.0001, 100.001, f64::NAN, f64::INFINITY, 1e300] {
            assert_eq!(gain_milli(bad, KP_RANGE_MILLI), None, "{bad}");
        }
        assert_eq!(gain_milli(2.001, KI_RANGE_MILLI), None);
    }

    #[test]
    fn flash_pending_covers_debounce_write_retry_and_changes_during_a_write() {
        let mut app = AppState::new(LIMITS, Settings::default());
        assert!(!app.status().settings_pending);
        app.mutate_gains(11_000, 100, 0);
        assert!(app.status().settings_pending, "dirty/debounce must not say saved");
        let first = app.take_settings_to_save().unwrap();
        assert!(!app.settings_dirty);
        assert!(app.status().settings_pending, "write/read-back verification still pending");
        assert_eq!(app.take_settings_to_save(), None, "cannot start a concurrent write");
        app.finish_settings_save(false);
        assert!(app.status().settings_pending);
        assert!(!app.storage_ok);
        assert_eq!(app.storage_failures_total, 1);
        assert_eq!(app.take_settings_to_save(), Some(first));
        app.mutate_gains(12_000, 100, 0); // another setting was applied before the write completed
        app.finish_settings_save(true);
        assert!(app.status().settings_pending, "the newer settings are not confirmed by the older save");
        let second = app.take_settings_to_save().unwrap();
        assert_eq!(second.kp_milli, 12_000);
        app.finish_settings_save(true);
        assert!(!app.status().settings_pending);
        assert!(app.storage_ok);
        assert_eq!(app.storage_writes_total, 3);
        assert_eq!(app.take_settings_to_save(), None);
    }

    #[test]
    fn reset_raw_read_to_average_transition_is_recovering_not_error() {
        let (mut app, mut ms) = running_app();
        ms += 5_000;
        app.tick(ms);
        app.take_power_cycle();
        assert_eq!(app.status().sensor, "resetting");
        assert!(app.raw_reading(22_100, ms + 2_000));
        assert_eq!(app.status().sensor, "recovering", "valid raw sample arrived; average still pending");
        assert_eq!(app.status().fault, None);
        assert_eq!(app.status().mode, "pid");
        assert_eq!(app.temperature_mc, None, "never fabricate a current temperature from the cache");
        app.tick(ms + 5_000);
        assert_eq!(app.status().sensor, "ok");
        assert!(!app.status().heater_output_held);
    }

    #[test]
    fn an_empty_window_interrupts_consecutive_good_windows_even_after_resets_failed() {
        let (mut app, mut ms) = running_app();
        let end = ms + 50_000;
        while ms < end { ms += 100; app.tick(ms); app.take_power_cycle(); }
        assert_eq!(app.control.fault().map(|f| f.label()), Some("sensor_error"));
        app.raw_reading(22_000, ms);
        app.tick(ms + 5_000);
        assert_eq!(app.control.recovery_samples(), 1);
        app.raw_reading(22_000, ms + 5_000);
        app.tick(ms + 10_000);
        assert_eq!(app.control.recovery_samples(), 2);
        app.tick(ms + 15_000); // no good raw read in this window
        assert_eq!(app.control.recovery_samples(), 0, "must require consecutive, not accumulated good windows");
        app.raw_reading(22_000, ms + 15_000);
        app.tick(ms + 20_000);
        assert_eq!(app.control.recovery_samples(), 1);
        assert_eq!(app.control.mode(), "fault");
    }

    #[test]
    fn a_valid_raw_overtemperature_during_reset_cuts_heat_before_averaging() {
        let (mut app, mut ms) = running_app();
        ms += 5_000;
        app.tick(ms);
        app.take_power_cycle();
        app.raw_reading(36_000, ms + 2_000);
        assert_eq!(app.control.fault().map(|f| f.label()), Some("over_temperature"));
        assert!(!app.tick(ms + 2_001));
        assert_eq!(app.control.commanded_duty_pct(ms + 2_001), 0.0);
    }
}

#[cfg(test)]
mod metrics_tests {
    use super::{heater_control::SafetyLimits, http_api::AppState, persist::Settings, sensor_recovery::Phase};

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
        // The probe is being power-cycled: not a fault yet.
        assert!(m.contains("birdburner_sensor_resetting 1\n"));
        assert!(m.contains("birdburner_fault{fault=\"none\"} 1"));
        assert!(m.contains("birdburner_sensor_power_cycles_total 0\n"), "counted when performed");
        app.take_power_cycle();
        assert!(app.raw_reading(25_000, 2000));
        let m = app.metrics();
        assert!(m.contains("birdburner_sensor_resetting 0\n"));
        assert!(m.contains("birdburner_sensor_power_cycles_total 1\n"));
        assert!(m.contains("birdburner_sensor_recoveries_total 1\n"));
        // Silent through every reset: now it is a fault.
        let mut ms = 2000;
        app.window_failed(ms);
        while app.recovery.phase() != Phase::Failed { ms += 100; app.tick(ms); }
        let m = app.metrics();
        assert!(m.contains("birdburner_fault{fault=\"sensor_error\"} 1"));
        assert!(m.contains("birdburner_sensor_resetting 0\n"));
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
