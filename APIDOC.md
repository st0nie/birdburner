# Birdburner HTTP API

Base URL: `http://<IP shown on the OLED>`, port 80. HTTP is handled by picoserve with 4 concurrent workers.
**No authentication or TLS — trusted LAN only; never expose to the Internet.**

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Phone control page (target slider, start/stop, fault display) |
| GET | `/api/status` | Status JSON |
| GET | `/metrics` | Prometheus metrics (text format 0.0.4) |
| POST | `/api/target` | Set target `{"target_c":25.5}` — 15–30 °C, 0.5 °C steps, saved to flash |
| POST | `/api/max_output` | Set max heater power `{"max_output_pct":80}` — integer 10–100 %, saved to flash |
| POST | `/api/limit` | Set safety cutoff `{"max_temperature_c":35}` — 25–40 °C, 0.5 °C steps, must exceed target by ≥2 °C, saved to flash |
| POST | `/api/start` | Enable heating control (saved as "desired on") |
| POST | `/api/stop` | Stop heating, immediately commands the SSR open (saved as "desired off") |

All POSTs are idempotent. On success they return the current status JSON; on failure `{"error":"..."}`.

## /api/status fields

| Field | Meaning |
| --- | --- |
| `temperature_c` | Average of all successful reads in the latest 5 s window; null when a whole window had none |
| `display` | Exact OLED text, e.g. `25.44 C`, `25.44 C*` (cached, also shown while the probe is being reset), `Sensor Error` (only once every reset failed) |
| `target_c` | Target temperature |
| `sensor` / `consecutive_sensor_errors` | `ok` / `error` / `reading` / `resetting`; number of consecutive 5 s windows without a valid reading. `resetting` = the probe stopped answering and is being power-cycled (see [Probe power reset](#probe-power-reset)); it is **not** a fault |
| `sensor_reset_attempt` / `sensor_power_cycles` | Reset attempt in progress (1–3, 0 when none) / probe power cycles performed since boot |
| `window_ok_reads` / `window_failed_reads` | Successful / failed raw reads in the last 5 s window |
| `mode` | `pid` controlling · `standby` off awaiting (also while the probe is being reset, with `desired_enabled` true) · `stopped` user stopped · `fault` fault |
| `desired_enabled` | User intent (persisted; used to resume after a power cut) |
| `recovery_samples` | Good samples gathered toward auto-recovery or resuming after a probe reset (resumes at 3) |
| `pid_output_pct` | PID-computed value, 0–`max_output_pct`; null when not computed |
| `max_output_pct` | Max heater duty cap (%), default 100 |
| `commanded_duty_pct` | Duty actually commanded (2 s window, 20 ms quantised) |
| `ssr_command` | `Closed` (energised) / `Open` — a software command, not measured load feedback |
| `fault` | Active fault code or null. All faults auto-recover |
| `max_temperature_c` | Software safety cutoff |
| `start_blocked_by` | Why start is refused, or null |
| `storage_ok` | Last flash save succeeded |
| `sample_hz` / `sample_seq` | Measured sampling rate / total successful reads |

## Faults and recovery

**Nothing latches permanently.** Any fault cuts the heater immediately and recovers automatically once conditions are good; the user's on/off intent is never changed by a fault and faults are never written to flash.

| Fault | Cause | Auto-recovery |
| --- | --- | --- |
| `sensor_error` | The probe is still silent after **all 3 power-cycle resets** (a whole 5 s window without a single good read — CRC errors, no response, conversion timeout, 85 °C power-on value — first starts the resets, it is not a fault by itself) | The first valid reading, then 3 consecutive good windows |
| `sensor_stale` | No new window result for 12 s | 3 consecutive good windows |
| `invalid_sample` | Out-of-range reading / timestamp anomaly | 3 consecutive good windows |
| `over_temperature` | Temperature ≥ limit | Temperature drops **3 °C below the limit** (hysteresis), then 3 good windows |
| `invalid_config` | PID produced an invalid value | 3 consecutive good windows |

After recovery, heating resumes if it was on before; a manual stop stays stopped.

**Sampling:** raw reads are grouped into 5 s windows. If any read in a window succeeds, the average of the successful reads is that window's temperature; failed reads are discarded and do not cut the heater. A window with zero successes cuts the heater and starts the probe power reset; it becomes a fault only if the resets fail. OLED rule: while a failed window is being handled the last value stays with `*`; `Sensor Error` appears only once every reset has failed. Over-temperature detection can therefore lag by up to ~5 s.

**No reading means no heating.** If the probe fails for a long time, the heater stays off — keep the probe wiring reliable. Watch the `sensor_errors_by_kind_total` and `sensor_power_cycles_total` metrics and the `BirdburnerSensorMissing` alert.

Possible `start_blocked_by` values: `safety_not_configured`, `sensor_not_ready` (waiting for a fresh reading), `over_temperature` (within 2 °C of the limit), `invalid_config`.

## Probe power reset

The DS18B20 can latch up: its 1-Wire interface still answers (presence and CRC fine) but it never converts again, and only removing its VDD clears that. The probe's VDD is wired to **GPIO22** (wiring: see README), and the firmware power-cycles it by itself before reporting a fault.

Trigger: a 5 s window without a single valid reading. The heater is cut at that moment (no change from before) but no fault is raised. Then:

| Step | VDD off | Wait for a valid reading | `sensor` | `fault` |
| --- | --- | --- | --- | --- |
| reset 1 | 1 s | up to 6 s | `resetting`, `sensor_reset_attempt` 1 | `null` |
| reset 2 | 3 s | up to 6 s | `resetting`, 2 | `null` |
| reset 3 | 10 s | up to 6 s | `resetting`, 3 | `null` |
| all failed (≈ 32 s after the first silent window) | – | – | `error` | `sensor_error` |

- The **first valid reading** ends the sequence at any step. The heater stays off until 3 good windows have been collected (`recovery_samples` 1, 2, then `mode` returns to `pid`), so it resumes about 15 s after the silent window when the first reset works.
- After giving up, the power cycle **repeats every 60 s** (VDD off 3 s) while the fault stays reported; a probe that comes back at any time clears it (first valid reading, then 3 good windows).
- Each power cycle holds DATA low as well, so the pull-ups cannot keep the chip alive. If the probe is wired to 3V3 instead of GPIO22 the resets have no effect and the fault appears after the same ~32 s.
- The PID integral is kept across a reset that works (only if the heater was running), so `pid_output_pct` resumes near its previous value once heating restarts. It is wiped when the resets fail (`sensor_error`), on any other fault, on stop/start and on a target change.
- Manual **stop** and `desired_enabled` are never changed by a reset; `POST /api/start` during a reset is refused with `sensor_not_ready`.
- Unlike the old behaviour, a silent probe now reaches `fault` ~32 s after the first silent window instead of immediately. The heater is off from the first silent window either way.

Observability: `sensor_resetting` (gauge), `sensor_power_cycles_total` and `sensor_recoveries_total` (counters) in `/metrics`; the serial log prints `[SENSOR] ...` phase changes and `[DS18B20] ... power-cycling the probe`.

## Power-cut recovery

Persisted to flash: target, safety limit, max output, desired on/off. **Readings and faults are never persisted.**
On boot the SSR stays open; after 3 valid fresh samples, if the saved intent is "on", the PID resumes automatically.
Writes alternate between two sectors with a CRC; a power cut mid-write falls back to the previous valid record.

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
curl -X POST http://$IP/api/max_output -H 'Content-Type: application/json' -d '{"max_output_pct":80}'
curl -X POST http://$IP/api/limit      -H 'Content-Type: application/json' -d '{"max_temperature_c":35}'
curl -X POST http://$IP/api/start
curl -X POST http://$IP/api/stop
```
