# Birdburner HTTP API

Base URL: `http://<IP shown on the OLED>`, port 80. HTTP is handled by picoserve with 4 concurrent workers.
**No authentication or TLS — trusted LAN only; never expose to the Internet.**

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Industrial-style control page: measurements, start/stop, operating limits, PID form and diagnostics |
| GET | `/api/status` | Status JSON |
| GET | `/metrics` | Prometheus metrics (text format 0.0.4) |
| POST | `/api/target` | Set target `{"target_c":25.5}` — 15–30 °C, 0.5 °C steps, saved to flash |
| POST | `/api/pid` | Set PID gains `{"kp":20,"ki":0.02,"kd":120,"d_filter_s":30}` — kp/ki/kd required, `d_filter_s` optional (0–300 s, 0 = off; omitted keeps current); Kp 0–100, Ki 0–2, Kd 0–200 (rounded to 0.001), applied at the next sample, saved to flash |
| POST | `/api/max_output` | Set max heater power `{"max_output_pct":80}` — integer 10–100 %, saved to flash |
| POST | `/api/limit` | Set safety cutoff `{"max_temperature_c":35}` — 25–40 °C, 0.5 °C steps, must exceed target by ≥2 °C, saved to flash |
| POST | `/api/start` | Enable heating control (saved as "desired on") |
| POST | `/api/stop` | Stop heating, immediately commands the SSR open (saved as "desired off") |

All POSTs are idempotent. On success they return the current status JSON; on failure `{"error":"..."}`. Settings apply to RAM first; a successful response does not guarantee a completed flash write. Wait for `settings_pending:false` and `storage_ok:true` before RST or removing power.

## /api/status fields

| Field | Meaning |
| --- | --- |
| `temperature_c` | Average of all successful reads in the latest 5 s window; null when a whole window had none |
| `display` | Exact OLED text, e.g. `25.44 C`, `25.44 C*` (cached, also shown while the probe is being reset), `Sensor Error` (only once every reset failed) |
| `target_c` | Target temperature |
| `sensor` / `consecutive_sensor_errors` | `ok` / `error` / `reading` / `resetting` / `recovering`; number of consecutive 5 s windows without a valid reading. `resetting` = automatic power-cycle attempts; `recovering` = a valid raw read has returned, awaiting the next 5 s average. Neither transitional state is itself a fault |
| `sensor_reset_attempt` / `sensor_power_cycles` | Reset attempt in progress (1–3, 0 when none) / probe power cycles performed since boot |
| `window_ok_reads` / `window_failed_reads` | Successful / failed raw reads in the last 5 s window |
| `mode` | `pid` controlling (also during bounded probe recovery if heating was already active) · `standby` awaiting fresh data · `stopped` user stopped · `fault` fault |
| `desired_enabled` | User intent (persisted; used to resume after a power cut) |
| `recovery_samples` | Good samples gathered toward startup or recovery from an actual fault (resumes at 3). Successful short probe recovery while running does not require this wait |
| `pid_output_pct` | PID-computed value, 0–`max_output_pct`; null when not computed |
| `max_output_pct` | Max heater duty cap (%), default 100 |
| `pid_kp` / `pid_ki` / `pid_kd` | Current PID gains (Kp %/°C, Ki %/(°C·s), Kd %·s/°C); persisted; factory 20 / 0.02 / 120 |
| `pid_d_filter_s` | Derivative first-order low-pass time constant, s (0 = off); persisted; factory 30 |
| `commanded_duty_pct` | Duty actually commanded (2 s window, 20 ms quantised) |
| `ssr_command` | `Closed` (energised) / `Open` — a software command, not measured load feedback |
| `fault` | Active fault code or null. All faults auto-recover |
| `max_temperature_c` | Software safety cutoff |
| `start_blocked_by` | Why start is refused, or null |
| `storage_ok` | Last flash save succeeded (true initially; alone does not establish the newest settings were saved) |
| `settings_pending` | A settings change is dirty, debouncing, or being written to flash; wait for false with `storage_ok:true` |
| `heater_output_held` | True while output is held at the last PID duty during bounded probe recovery |
| `sample_hz` / `sample_seq` | Measured sampling rate / total successful reads |

## Faults and recovery

**Nothing latches permanently.** Any fault cuts the heater immediately and recovers automatically once conditions are good; the user's on/off intent is never changed by a fault and faults are never written to flash.

| Fault | Cause | Auto-recovery |
| --- | --- | --- |
| `sensor_error` | The probe is still silent after **all 3 power-cycle resets** (a whole 5 s window without a single good read — CRC errors, no response, conversion timeout, 85 °C power-on value — first starts the resets, it is not a fault by itself) | The first valid reading, then 3 consecutive good windows |
| `sensor_stale` | No new control sample for 12 s outside recovery, or 45 s even during probe recovery; also a reversed clock | 3 consecutive good windows |
| `invalid_sample` | Out-of-range reading / timestamp anomaly | 3 consecutive good windows |
| `over_temperature` | Temperature ≥ limit | Temperature drops **3 °C below the limit** (hysteresis), then 3 good windows |
| `invalid_config` | PID produced an invalid value | 3 consecutive good windows |

After recovery, heating resumes if it was on before; a manual stop stays stopped.

**Sampling:** raw reads are grouped into 5 s windows. If any read in a window succeeds, the average of the successful reads is that window's temperature; failed reads are discarded and do not cut the heater. A window with zero successes starts the probe power reset; an already running heater retains its last duty until the attempts fail or the independent age guard expires. OLED rule: while a failed window is being handled the last value stays with `*`; `Sensor Error` appears only once every reset has failed. Over-temperature detection can therefore lag by up to ~5 s.

**No prolonged blind heating.** A short, bounded probe recovery holds the last duty; a failed recovery cuts it. If the probe fails for a long time, the heater stays off — keep the probe wiring reliable. Watch the `sensor_errors_by_kind_total` and `sensor_power_cycles_total` metrics and the `BirdburnerSensorMissing` alert.

Possible `start_blocked_by` values: `safety_not_configured`, `sensor_not_ready` (waiting for a fresh reading), `over_temperature` (within 2 °C of the limit), `invalid_config`.

## Probe power reset

The observed probe failure can recover after power removal even when ESP32 RST does not help. This motivates automatic power cycling; the counters and restart behaviour do not prove physical latch-up or a particular electrical cause. The probe's VDD is wired to **GPIO22** (wiring: see README), and the firmware power-cycles it by itself before reporting a fault.

Trigger: a 5 s window without a single valid reading. **A running heater is not interrupted**: it keeps its last PID output (same duty and PWM windows) and no fault is raised. Then:

| Step | VDD off | Wait for a valid reading | `sensor` | `fault` | `mode` (heater was running) |
| --- | --- | --- | --- | --- | --- |
| reset 1 | 1 s | up to 6 s | `resetting`, `sensor_reset_attempt` 1 | `null` | `pid` |
| reset 2 | 3 s | up to 6 s | `resetting`, 2 | `null` | `pid` |
| reset 3 | 10 s | up to 6 s | `resetting`, 3 | `null` | `pid` |
| all failed (≈ 32 s after the first silent window) | – | – | `error` | `sensor_error` | `fault`, heater cut |

- The **first valid raw reading** ends the reset sequence. The API then reports `sensor:"recovering"` until the next valid 5 s average; output remains held and normal PID updates resume without a 3-window wait. The integral survives, while the gap contributes neither integration nor derivative kick.
- If the heater was **not** running (stopped, standby, a fault already active) the reset just keeps it off; `mode` stays `standby`/`stopped`/`fault`, and `start_blocked_by` is `sensor_not_ready` until a reading arrives.
- For this sensor-recovery path, the heater is cut after all three attempts fail. An independent 45 s measurement-age guard also cuts it if the sequence stalls or valid averages never resume; repeated attempts do not renew the deadline. Existing faults, Stop and a new over-temperature still take priority. While output is held there is no new temperature, so the software cannot detect a new over-temperature. The independent hardware cutout must stay in place.
- After giving up, the power cycle **repeats every 60 s** (VDD off 3 s) while the fault stays reported; a probe that comes back at any time clears it (first valid reading, then 3 good windows, PID from scratch).
- Each power cycle holds DATA low as well, so the pull-ups cannot keep the chip alive. If the probe is wired to 3V3 instead of GPIO22 the resets have no effect and the fault appears after the same ~32 s.
- Manual **stop** immediately cancels heating and clears the PID, even during recovery; it does not have to cancel a pending probe power cycle. A target change resets the PID and keeps output off until a fresh sample. The user's on/off choice is never changed by probe recovery.

Observability: `sensor_resetting` (gauge), `sensor_power_cycles_total` and `sensor_recoveries_total` (counters) in `/metrics`; the serial log prints `[SENSOR] ...` phase changes and `[DS18B20] ... power-cycling the probe`. The OLED and the web page do not show a reset that the running heater rides through.

## Power-cut recovery

Persisted to flash: target, safety limit, max output, PID gains, derivative filter time constant, desired on/off. **Readings and faults are never persisted.**
On boot the SSR stays open; after 3 valid fresh samples, if the saved intent is "on", the PID resumes automatically.
Writes alternate between two sectors with a CRC; a power cut mid-write falls back to the previous valid record. BDS5 records (40 bytes) store Kp/Ki/Kd and the D filter time constant as unsigned thousandths and validate their ranges and CRC. Older BDS1/BDS2/BDS3 records still load with their existing target/on-off/limits and default PID gains (20 / 0.02 / 120) and D filter 30 s; BDS4 records keep their gains and get the default D filter. A settings change is debounced before writing, so wait for flash confirmation. Both ESP32 RST and full power removal retain a completed flash save; the live PID integral is not persisted. Older firmware does not understand BDS5 records; do not assume a downgrade will preserve settings.

Crossing the target does not clear the integral. Near the target, output is clamped P+I+D. Conditional integration blocks only changes further into saturation: increasing I when the requested sum is above the output cap, or decreasing I when it is below zero. Changes towards the output range are permitted even if the sum remains outside it. The integral's own 0–cap bounds remain; a nonzero I at zero output is intentional, not necessarily an error. At or above target + 1 °C, PID output is forced to zero. The absolute over-temperature cutoff, sensor protections and manual Stop are unchanged.

## /metrics (Prometheus)

All metrics carry the `birdburner_` prefix.

| Metric | Type | Meaning |
| --- | --- | --- |
| `temperature_celsius` | gauge | 5 s window average; **the line is absent when there is no valid reading** (never reports a fake 0) |
| `target_temperature_celsius` / `max_temperature_celsius` | gauge | Target / safety limit |
| `sensor_ok` / `sensor_consecutive_errors` | gauge | Last window had a valid reading / consecutive failed windows |
| `sensor_resetting` | gauge | 1 while the probe is being power-cycled (reset 1–3 running, not a fault yet) |
| `window_ok_reads` / `window_failed_reads` | gauge | Good / failed raw reads in the last 5 s window |
| `sample_rate_hertz` | gauge | Successful raw sampling rate |
| `pid_output_percent` | gauge | PID computed output; absent when not computed |
| `max_output_percent` | gauge | Max heater duty cap |
| `pid_d_filter_seconds` | gauge | Derivative low-pass time constant (s, 0 = off) |
| `pid_kp` / `pid_ki` / `pid_kd` | gauge | PID gains configured in the controller; update after an accepted request and restore after reboot |
| `heater_output_held` | gauge | 1 while the heater uses its last output during bounded probe recovery |
| `heater_duty_percent` / `heater_on` | gauge | Commanded duty / SSR energised |
| `control_enabled` / `control_desired_enabled` | gauge | PID driving / user wants heating on |
| `wifi_connected` / `storage_ok` | gauge | WiFi up / last flash save succeeded |
| `heater_rated_watts` | gauge | Configured heater rating |
| `mode{mode=...}` | gauge | `pid`/`standby`/`stopped`/`fault`, the active one is 1 |
| `fault{fault=...}` | gauge | `none` and each fault code, the active one is 1 |
| `uptime_seconds` | counter | Seconds since boot; **a drop means reboot/power cut** |
| `sensor_readings_total` / `sensor_errors_total` | counter | Good / failed raw reads |
| `sensor_power_cycles_total` | counter | Probe power cycles performed (VDD switched off and on), including the 60 s retries after a fault |
| `sensor_recoveries_total` | counter | Times the probe answered again after a reset or a fault |
| `sensor_errors_by_kind_total{kind=...}` | counter | Failures by cause: `no_presence`, `crc`, `bad_data`, `sensor_reset`, `timing`, `power` |
| `heater_on_seconds_total` | counter | Total SSR-energised seconds |
| `heater_switches_total` | counter | SSR command transitions |
| `storage_writes_total` / `storage_failures_total` | counter | Flash write attempts / failed verifications |

Counters reset on boot; Prometheus `rate()` / `increase()` handle that automatically.

For a Grafana PID-parameter panel, use separate queries `birdburner_pid_kp`, `birdburner_pid_ki` and `birdburner_pid_kd` (not `rate()`; these are gauges). They can be overlaid with `birdburner_pid_output_percent` or shown as stat panels. Brief probe recovery deliberately leaves `temperature_celsius` absent rather than pretending a cached value is a current measurement.

Estimated average heater power (W):

```promql
rate(birdburner_heater_on_seconds_total[10m]) * birdburner_heater_rated_watts
```

Scrape config:

```yaml
scrape_configs:
  - job_name: birdburner
    scrape_interval: 15s
    static_configs:
      - targets: ["192.168.50.135:80"]   # reserve a fixed IP for the device on your router
```

Suggested alerts:

```yaml
groups:
  - name: birdburner
    rules:
      - alert: BirdburnerDown
        expr: up{job="birdburner"} == 0
        for: 2m
      - alert: BirdburnerFault
        expr: birdburner_fault{fault!="none"} == 1
        for: 1m
      - alert: BirdburnerSensorMissing
        expr: absent(birdburner_temperature_celsius) or birdburner_sensor_ok == 0
        for: 1m   # a reset that works keeps sensor_ok at 0 for only ~5-10 s
      - alert: BirdburnerProbeUnstable
        expr: increase(birdburner_sensor_power_cycles_total[1h]) >= 3
        # the probe keeps locking up or going silent: check wiring, the probe-end capacitor/resistor, cable routing
      - alert: BirdburnerTooHot
        expr: birdburner_temperature_celsius > 32
        for: 2m
      - alert: BirdburnerTooCold
        expr: birdburner_control_enabled == 1 and birdburner_temperature_celsius < birdburner_target_temperature_celsius - 3
        for: 30m
      - alert: BirdburnerRebooted
        expr: resets(birdburner_uptime_seconds[10m]) > 0
```

The 32 °C `TooHot` threshold is an example — adjust to your situation. Remote alerting depends on network and Prometheus and cannot replace the hardware cutout.

## Examples

```sh
IP=192.168.50.135
curl http://$IP/api/status
curl -X POST http://$IP/api/target     -H 'Content-Type: application/json' -d '{"target_c":25}'
curl -X POST http://$IP/api/pid        -H 'Content-Type: application/json' -d '{"kp":20,"ki":0.02,"kd":120,"d_filter_s":30}'
curl -X POST http://$IP/api/max_output -H 'Content-Type: application/json' -d '{"max_output_pct":80}'
curl -X POST http://$IP/api/limit      -H 'Content-Type: application/json' -d '{"max_temperature_c":35}'
curl -X POST http://$IP/api/start
curl -X POST http://$IP/api/stop
```
