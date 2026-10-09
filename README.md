# birdburner — ESP32-C6 bird cage temperature controller

## Hardware

| Component | Wiring |
| --- | --- |
| DS18B20 probe (cage centre), via adapter board + 2 m cable | DATA → GPIO23, **VDD → GPIO22** (switchable supply, not the 3.3 V pin), GND → GND, 4.7 kΩ pull-up DATA→VDD (on the adapter board). Details below |
| SSD1306 128×64 OLED (I2C) | SDA → GPIO6, SCL → GPIO7, VCC → 3.3 V |
| SSR-25DA heater control | GPIO15 → terminal 3 (+), GND → terminal 4 (−) |

- Heater: 100 W ceramic heat emitter (no light), 220 V AC via the SSR (load terminals 1–2). Switch the live wire; a KSD9700 thermal cutout is wired in series as an independent hardware safety.
- Flash: 16 MB; settings persist in two flash sectors at 0xFF0000 / 0xFF1000.
- Serial log: 115200 8N1.

> **Safety:** this heats an animal enclosure. Software limits are a backup, not a substitute for the hardware cutout (KSD9700) or a fuse. Test empty, watch the first full-power run, and always cut mains power before rewiring. The centre probe cannot see hot spots near the heater.

## DS18B20 wiring and probe power reset

### Why the probe's VDD is on a GPIO

On this build the DS18B20 occasionally **latches up**: after hours of normal operation its 1-Wire interface keeps answering (presence and CRC stay perfect) but it never converts again. Nothing sent on the bus clears that, and the ESP32's RST button does not either, because the probe stays powered; only removing its VDD does (unplugging the board's power was the only cure). So the probe is powered from **GPIO22**, and the firmware switches it off and on by itself.

### Wiring (adapter board, 2 m cable)

| Adapter board / cable | Connect to | Note |
| --- | --- | --- |
| **VCC** (red) | **GPIO22** | The probe's switchable supply. **Not 3V3.** Draws ≈ 1.5 mA while converting (plus the pull-up current and an LED if the board has one, a few mA at most), far below what a GPIO can source, but power nothing else from it. |
| **DATA** / DQ (yellow) | **GPIO23** | The adapter board's 4.7 kΩ pull-up goes to its VCC, i.e. to GPIO22, so it is switched off together with the probe. |
| **GND** (black) | GND | |
| Shield (shielded cable only) | GND **at the adapter/ESP32 end only** | Leave the probe end of the shield cut short and insulated, away from the probe pins and its metal housing. The shield never replaces the black GND wire; use three cores + shield. Grounding both ends creates a ground loop. |

Wire colours vary between probes; confirm VDD / GND / DATA with a multimeter (continuity from the adapter terminal to each probe wire) before powering.

```
ESP32-C6      adapter board        2 m cable                probe end (solder here)

GPIO22 ─────► VCC  ──────────────── red ─────────┬──────────────► VDD
                                                 │ 0.1 µF (+ 10 µF in parallel)
GND    ─────► GND  ──────────────── black ───────┴──────────────► GND

GPIO23 ◄───► DATA ──────────────── yellow ──[100 Ω]─────────────► DATA

(the adapter board carries the 4.7 kΩ pull-up between its VCC and DATA terminals;
 shielded cable: the shield joins GND at the adapter/ESP32 end only)
```

Moving an existing build over: pull the VCC wire off the **3V3** pin and put it on **GPIO22** (one wire). GPIO22 is an ordinary GPIO on the ESP32-C6, not a strapping pin (on common dev boards it is the header pin next to GPIO23; check your board's pinout). The OLED keeps using 3V3; the probe no longer needs the split. If VCC stays on 3V3 the firmware still works, but the reset cannot do anything: a latched probe is reported as `sensor_error` after the three failed resets (about 32 s) and stays that way until power is removed, as before.

### Recommended hardening at the probe end (parts cost a few cents)

These stop the spikes that trigger a latch-up in the first place; the reset is the safety net, not a substitute.

1. **0.1 µF ceramic capacitor (marked `104`) across VDD–GND, soldered right at the probe's pins**, ideally with a 10 µF electrolytic in parallel (stripe = GND). It absorbs spikes and keeps the probe's own supply steady during a conversion, which a 2 m thin cable and Dupont contacts cannot.
2. **100 Ω resistor in series with DATA, also at the probe end.** It limits the current a spike can inject into the pin and does not disturb 1-Wire timing.
3. **Route the cable away from mains and the SSR**: at least 10–15 cm apart, crossing at 90° where unavoidable, never bundled with the 220 V wiring.
4. Solder (or use proper terminals) at the probe end and heat-shrink every joint; Dupont contacts corrode in a damp cage.
5. A genuine Maxim DS18B20 (authorised distributor, part number `DS18B20+`) is much harder to latch than the unbranded clones sold with waterproof probes.

### What the firmware does

A **5 s window without a single valid reading** is the trigger (isolated bad reads inside a window are tolerated as before). The heater is cut at once, as it always was, but this is **not reported as a fault**:

| Step | VDD off | Waits for a valid reading | OLED line 3 |
| --- | --- | --- | --- |
| reset 1 | 1 s | up to 6 s | `Sensor reset 1/3` |
| reset 2 | 3 s | up to 6 s | `Sensor reset 2/3` |
| reset 3 | 10 s | up to 6 s | `Sensor reset 3/3` |
| still silent | - | - | `FAULT sensor_error`, temperature line `Sensor Error` |

- The **first valid reading ends the sequence**; the heater then resumes after 3 good 5 s windows (`resuming 1/3`, `2/3`, then PID). When the first reset works, heating is back roughly 15 s after the silent window was detected, and the heater stays off the whole time.
- While resetting, the temperature line keeps the last good value with a `*`.
- **The PID integral is kept** across a reset that works (the heater was running), so heating resumes at the output it had learned instead of ramping up from zero and letting the cage sag. Nothing is integrated during the gap. If the resets fail (`sensor_error`), or on any other fault, stop, start or target change, the PID starts from scratch as before.
- Only if all three resets fail is `sensor_error` raised (about 32 s after the first silent window). The power cycle then **repeats once a minute** (VDD off 3 s) without clearing the fault, so a probe that comes back (re-plugged, power glitch) recovers by itself.
- Each power cycle holds DATA low together with VDD. Otherwise the pull-ups would feed the chip through its input protection diode and a latched probe would never lose power.
- The reset needs no network and runs even while the page is unreachable. A manual **Stop** is kept through a reset; the on/off choice is never changed.

Where to see it:

| Where | What |
| --- | --- |
| OLED line 3 | `Sensor reset n/3` while resetting, `FAULT sensor_error` after giving up |
| `/api/status` | `"sensor":"resetting"`, `sensor_reset_attempt` (1-3), `sensor_power_cycles` (since boot); `fault` stays `null` until the resets fail |
| `/metrics` | `birdburner_sensor_resetting`, `birdburner_sensor_power_cycles_total`, `birdburner_sensor_recoveries_total` |
| Serial log | `[SENSOR] ...` phase changes and `[DS18B20] no valid reading: power-cycling the probe (reset 1/3, VDD off 1000 ms)` |

Timings are constants at the top of `src/sensor_recovery.rs` (`OFF_MS`, `RESPONSE_MS`, `RETRY_MS`, `RETRY_OFF_MS`, `MAX_ATTEMPTS`).

### Checking the wiring after the change

1. **VCC really comes from GPIO22**: with the firmware running, pull the VCC wire. Readings must stop and the OLED must show `Sensor reset 1/3` within about 10 s (a probe still reading with VCC pulled is not powered from GPIO22). Put the wire back: it recovers on its own, no `FAULT` is shown.
2. **Reset path**: unplug DATA for about 10 s and plug it back. Expect `Sensor reset 1/3` (maybe `2/3`), no `FAULT`, and `sensor_power_cycles` increasing. Keep it unplugged for 40 s or more and `FAULT sensor_error` appears; plug it back and it clears after the probe answers and 3 good windows.
3. The heater is off during both tests by design; watch `/api/status` (`curl http://<IP>/api/status`) or the serial log.

This proves the wiring and the supervision logic. A real latch-up cannot be provoked on demand: if one happens, `sensor_power_cycles_total` and `sensor_recoveries_total` in `/metrics` will have counted it.

## Firmware

- Rust `no_std` + Embassy; HTTP served by **picoserve** (4 workers), phone control page embedded (`src/index.html`).
- DS18B20 at **12-bit (0.0625 °C)**, displayed with two decimals. Readings are averaged per 5 s window: any successful reads in a window are averaged into one control sample and failed reads are dropped. A window with zero successful reads cuts the heater and first triggers an automatic **probe power reset** (see above); only if three resets fail is it a fault.
- PID (heat-only) drives the SSR as **2 s time-proportional PWM** (not kHz PWM). Target 15–30 °C in 0.5 °C steps; default 25 °C.
- Software safety limit default **35 °C** (adjustable 25–40 °C on the web page, must exceed the target by ≥2 °C). At the limit the heater cuts off instantly; it resumes automatically 3 °C below the limit (hysteresis).
- **Persistent settings** (target, safety limit, max output, on/off intent) survive power cuts (dual-sector, CRC-checked). Temperature readings and faults are never persisted.
- Every fault cuts the heater and recovers automatically — there is no permanent lockout: sensor errors (after the resets failed) recover once the probe answers and 3 good windows follow; over-temperature recovers below limit−3 °C. After a power cut the heater stays off until 3 good samples, then resumes the saved on/off intent.
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

OLED line 3 `PID 7% SSR:off` means the PID is running at 7 % duty and the SSR happens to be in the off part of its 2 s window. `resuming 1/3` means it is waiting for fresh valid readings before heating again. `Sensor reset 1/3` means the probe stopped answering and is being power-cycled (heater off, not a fault yet).

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
| DS18B20 12-bit config, conversion, CRC, probe power switch | `src/ds18b20.rs`, `src/ds18b20_data.rs` |
| Probe power-reset ladder (when to cycle VDD, when it is a fault) | `src/sensor_recovery.rs` |
| Display cache, `*` marker, 5-failure rule | `src/sensor_display.rs` |
| Serial reset/read tool | `tools/espmon.c` |

`[OLED] write error` = I2C failure (the firmware retries init once a second); check GPIO6/7 wiring and power. `[DS18B20]` errors: check DATA on GPIO23, VCC on GPIO22, common ground and the ~4.7 kΩ pull-up; keep the cable straight, uncoiled and away from mains wiring. Occasional CRC errors are tolerated by the 5 s averaging window; a window with zero successful reads cuts the heater and starts the probe power reset, and the heater stays off until readings are good again. Repeated resets (`birdburner_sensor_power_cycles_total` climbing) mean the probe or its cable/supply is unhealthy: apply the hardening list above. `sensor_errors_by_kind_total` tells the story: a latched probe shows `timing` and `bad_data` growing with `no_presence`, `crc` and `power` at 0; a loose wire shows `no_presence`.

Standalone diagnostic firmware (flashing them replaces the main firmware):

```sh
cargo build --release --bin ds18b20_serial   # serial temperature diagnostics (powers the probe from GPIO22), SSR held off
cargo build --release --bin ssr_toggle       # GPIO15 toggles every 5 s, NO thermal protection — wiring test only
```

Host-side logic tests (no device, no GPIO):

```sh
cargo test --manifest-path tests/host/Cargo.toml \
  --target x86_64-unknown-linux-gnu --target-dir target/host-tests
```
