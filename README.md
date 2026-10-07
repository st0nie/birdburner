# birdburner — ESP32-C6 bird cage temperature controller

## Hardware

| Component | Wiring |
| --- | --- |
| DS18B20 probe (cage centre) | DATA → GPIO23, VDD → 3.3 V, GND → GND, 4.7 kΩ pull-up DATA→3.3 V |
| SSD1306 128×64 OLED (I2C) | SDA → GPIO6, SCL → GPIO7, VCC → 3.3 V |
| SSR-25DA heater control | GPIO15 → terminal 3 (+), GND → terminal 4 (−) |

- Heater: 100 W ceramic heat emitter (no light), 220 V AC via the SSR (load terminals 1–2). Switch the live wire; a KSD9700 thermal cutout is wired in series as an independent hardware safety.
- Flash: 16 MB; settings persist in two flash sectors at 0xFF0000 / 0xFF1000.
- Serial log: 115200 8N1.

> **Safety:** this heats an animal enclosure. Software limits are a backup, not a substitute for the hardware cutout (KSD9700) or a fuse. Test empty, watch the first full-power run, and always cut mains power before rewiring. The centre probe cannot see hot spots near the heater.

## Firmware

- Rust `no_std` + Embassy; HTTP served by **picoserve** (4 workers), phone control page embedded (`src/index.html`).
- DS18B20 at **12-bit (0.0625 °C)**, displayed with two decimals. Readings are averaged per 5 s window: any successful reads in a window are averaged into one control sample, failed reads are dropped, and a window with zero successful reads is one fault.
- PID (heat-only) drives the SSR as **2 s time-proportional PWM** (not kHz PWM). Target 15–30 °C in 0.5 °C steps; default 25 °C.
- Software safety limit default **35 °C** (adjustable 25–40 °C on the web page, must exceed the target by ≥2 °C). At the limit the heater cuts off instantly; it resumes automatically 3 °C below the limit (hysteresis).
- **Persistent settings** (target, safety limit, max output, on/off intent) survive power cuts (dual-sector, CRC-checked). Temperature readings and faults are never persisted.
- Every fault cuts the heater and recovers automatically — there is no permanent lockout: sensor errors recover after 3 good windows; over-temperature recovers below limit−3 °C. After a power cut the heater stays off until 3 good samples, then resumes the saved on/off intent.
- SSR command Open/Closed is a software command only; there is no load-current feedback.

## WiFi credentials (never committed)

Credentials live in **`src/secrets.rs`**, which is gitignored:

```sh
cp src/secrets.example.rs src/secrets.rs   # then fill in SSID / PASSWORD
```

## Phone control

1. Connect the phone to the same WiFi, open the IP shown on the OLED bottom line (e.g. `http://192.168.50.135`).
2. Drag the slider to set the target temperature; releasing saves it to flash.
3. Tap **Start heating**. Status shows `Heating`, the output shows the actual duty.
4. **Stop heating** opens the SSR immediately and saves the off intent.
5. Faults recover automatically; the page shows the cause and recovery progress.
6. The **Max output** slider (10–100 %) caps heater power; the **Safety limit** slider (25–40 °C) sets the over-temperature cutoff. Both persist.

OLED line 3 `PID 7% SSR:off` means the PID is running at 7 % duty and the SSR happens to be in the off part of its 2 s window. `resuming 1/3` means it is waiting for fresh valid readings before heating again.

## PID tuning

Edit **`src/heater_control.rs` → `impl Default for PidConfig`**:

```rust
Self { kp: 10.0, ki: 0.1, kd: 0.0, output_limit_pct: DEFAULT_MAX_OUTPUT_PCT as f64 }
```

These are **provisional, not tuned** on this cage:

| Field | Meaning / unit |
| --- | --- |
| `kp` | Proportional gain, output %/°C |
| `ki` | Integral gain, output %/(°C·s) |
| `kd` | Derivative gain (on measurement), output %·s/°C |
| `output_limit_pct` | Duty cap used on first boot only; afterwards the persisted "Max output" slider value (10–100 %) applies |

Same file, at the top:

- `PWM_WINDOW_MS = 2000`: SSR time window (30 % ≈ 600 ms on / 1400 ms off).
- `MIN_AC_PULSE_MS = 20`: pulse quantisation for 50 Hz mains.
- `SAMPLE_WINDOW_MS = 5000`: one control sample per 5 s averaging window.
- `SENSOR_MAX_AGE_MS = 12_000`: sample older than this cuts the heater.

Tuning order: set real safety limits and verify wiring first; start with `ki=0`, `kd=0`, tune `kp`; add `ki` only to remove steady-state offset; change one parameter at a time and watch the response.

## Build & flash

```sh
cd /root/Workspaces/birdburner
cargo build --release --bin birdburner
# binary: target/riscv32imac-unknown-none-elf/release/birdburner
PORT=/dev/ttyACM0
/root/.cargo/bin/espflash flash --port "$PORT" \
  target/riscv32imac-unknown-none-elf/release/birdburner
```

Target toolchain: `riscv32imac-unknown-none-elf` (see `.cargo/config.toml` / `rust-toolchain.toml`). espflash may not be on PATH in this environment — use the absolute path above. Close any serial monitor before flashing.

### Windows → WSL USB forwarding

The board is Windows COM7, forwarded to WSL as `/dev/ttyACM0`. Unplugging/replugging breaks the forwarding; re-attach from Windows PowerShell:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>   # bind first if unshared: usbipd bind --busid <BUSID>
```

Check from WSL: `ls /dev/ttyACM* /dev/ttyUSB*`.

## Serial log (115200 baud, 8N1)

```sh
gcc -O2 -o /tmp/espmon tools/espmon.c
/tmp/espmon /dev/ttyACM0 30        # resets via RTS, prints 30 s incl. boot log
```

Or without resetting:

```sh
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb raw -echo -ixon -ixoff
cat /dev/ttyACM0                    # Ctrl+C to stop
```

**Stopping the serial reader does not stop the firmware or the heater.** To stop heating call `/api/stop`; to guarantee a dead load, cut mains power.

## HTTP API

Full reference (fields, fault recovery, Prometheus scrape config and alert rules): [APIDOC.md](APIDOC.md). Quick start:

```sh
IP=192.168.50.135   # whatever the OLED shows
curl http://$IP/api/status
curl http://$IP/metrics
curl -X POST http://$IP/api/target     -H 'Content-Type: application/json' -d '{"target_c":25.5}'
curl -X POST http://$IP/api/max_output -H 'Content-Type: application/json' -d '{"max_output_pct":100}'
curl -X POST http://$IP/api/limit      -H 'Content-Type: application/json' -d '{"max_temperature_c":35}'
curl -X POST http://$IP/api/start
curl -X POST http://$IP/api/stop
```

No authentication/TLS — trusted LAN only, do not expose to the Internet.

## Troubleshooting & code map

| Topic | File |
| --- | --- |
| WiFi, tasks, SSR output, safety config | `src/main.rs` |
| PID gains, anti-windup, PWM window, protections | `src/heater_control.rs` |
| picoserve routes, JSON, app state, metrics | `src/http_api.rs` |
| Phone control page | `src/index.html` |
| Flash persistence (dual sector + CRC) | `src/persist.rs` |
| DS18B20 12-bit config, conversion, CRC | `src/ds18b20.rs`, `src/ds18b20_data.rs` |
| Display cache, `*` marker, 5-failure rule | `src/sensor_display.rs` |
| Serial reset/read tool | `tools/espmon.c` |

`[OLED] write error` = I2C failure (the firmware retries init once a second); check GPIO6/7 wiring and power. `[DS18B20]` errors: check GPIO23, 3.3 V, common ground and the ~4.7 kΩ pull-up; keep the cable straight, uncoiled and away from mains wiring. Occasional CRC errors are tolerated by the 5 s averaging window; a window with zero successful reads cuts the heater until readings recover.

Standalone diagnostic firmware (flashing them replaces the main firmware):

```sh
cargo build --release --bin ds18b20_serial   # serial temperature diagnostics, SSR held off
cargo build --release --bin ssr_toggle       # GPIO15 toggles every 5 s, NO thermal protection — wiring test only
```

Host-side logic tests (no device, no GPIO):

```sh
cargo test --manifest-path tests/host/Cargo.toml \
  --target x86_64-unknown-linux-gnu --target-dir target/host-tests
```
