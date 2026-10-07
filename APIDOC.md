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
| `display` | Exact OLED text, e.g. `25.44 C`, `25.44 C*` (cached), `Sensor Error` |
| `target_c` | Target temperature |
| `sensor` / `consecutive_sensor_errors` | `ok` / `error` / `reading`; number of consecutive 5 s windows without a valid reading |
| `window_ok_reads` / `window_failed_reads` | Successful / failed raw reads in the last 5 s window |
| `mode` | `pid` controlling · `standby` off awaiting · `stopped` user stopped · `fault` fault |
| `desired_enabled` | User intent (persisted; used to resume after a power cut) |
| `recovery_samples` | Good samples gathered toward auto-recovery (recovers at 3) |
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
| `sensor_error` | A whole 5 s window without a single good read (CRC errors, no response, conversion timeout, 85 °C power-on value) | 3 consecutive good windows |
| `sensor_stale` | No new window result for 12 s | 3 consecutive good windows |
| `invalid_sample` | Out-of-range reading / timestamp anomaly | 3 consecutive good windows |
| `over_temperature` | Temperature ≥ limit | Temperature drops **3 °C below the limit** (hysteresis), then 3 good windows |
| `invalid_config` | PID produced an invalid value | 3 consecutive good windows |

After recovery, heating resumes if it was on before; a manual stop stays stopped.

**Sampling:** raw reads are grouped into 5 s windows. If any read in a window succeeds, the average of the successful reads is that window's temperature; failed reads are discarded and do not cut the heater. Only a window with zero successes counts as a fault and cuts the heater. OLED rule: 1–4 consecutive failed windows show the last value plus `*`; from the 5th, `Sensor Error`. Over-temperature detection can therefore lag by up to ~5 s.

**No reading means no heating.** If the probe fails for a long time, the heater stays off — keep the probe wiring reliable. Watch the `sensor_errors_by_kind_total` metric and the `BirdburnerSensorMissing` alert.

Possible `start_blocked_by` values: `safety_not_configured`, `sensor_not_ready` (waiting for a fresh reading), `over_temperature` (within 2 °C of the limit), `invalid_config`.

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
        for: 1m
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
