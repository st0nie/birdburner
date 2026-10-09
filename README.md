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

On this build the DS18B20 occasionally stops providing valid temperatures despite answering on the bus. The user observed that an ESP32 RST did not help, while a full power cycle did. That supports trying a probe power cycle; it does not by itself establish physical CMOS latch-up, a clone chip or a particular electrical cause. The probe is powered from **GPIO22** so the firmware can restart it without rebooting the controller.

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

These can improve supply and signal integrity; the cause of the observed failures has not been measured electrically. Automatic recovery is a safety net, not a substitute for reliable wiring.

1. **0.1 µF ceramic capacitor (marked `104`) across VDD–GND, close to the probe's pins**, optionally with a 10 µF capacitor in parallel (electrolytic stripe = GND). This improves local decoupling. If the probe and cable are sealed, do not cut open the waterproof seal just to add parts; use a suitable replacement probe or accessible termination.
2. A **100 Ω resistor in series with DATA** can limit transient current, but it also affects the bus waveform. Verify successful readings after fitting it; it is not a universal fix.
3. **Route the cable away from mains and the SSR**: at least 10–15 cm apart, crossing at 90° where unavoidable, never bundled with the 220 V wiring.
4. Solder (or use proper terminals) at the probe end and heat-shrink every joint; Dupont contacts corrode in a damp cage.
5. If failures continue, try a probe from a traceable supplier. A `DS18B20+` marking alone does not establish authenticity.

### What the firmware does

A **5 s window without a single valid reading** is the trigger (isolated bad reads inside a window are tolerated as before). From then on the probe is power-cycled, and **a running heater does not notice**: it keeps going on its last PID output (same duty, same 2 s PWM windows), the page keeps saying *Heating*, the OLED keeps its `PID n% SSR:..` line and nothing is reported as a fault.

| Step | VDD off | Waits for a valid reading |
| --- | --- | --- |
| reset 1 | 1 s | up to 6 s |
| reset 2 | 3 s | up to 6 s |
| reset 3 | 10 s | up to 6 s |
| still silent | - | the heater is cut and `FAULT sensor_error` is shown (temperature line `Sensor Error`) |

- The **first valid raw reading ends the reset sequence** at any step. Normal PID updates resume at the next valid 5 s average: no 3-window wait, no PID restart, the integral is kept and the gap is not integrated. Until that average arrives the heater keeps its held output; the API reports `sensor:"recovering"`, not an error.
- A sensor failure cuts the heater **after all three resets fail** (normally about 32 s after the first silent window). An independent **45 s maximum measurement-age guard** also cuts it if recovery stalls; reset attempts do not extend that deadline. This deliberately allows bounded open-loop heating at the last duty while no new temperature is available, so the software cannot detect a new over-temperature during that interval. Existing faults, manual Stop and a newly observed over-temperature still take priority. Keep the independent hardware thermal cutout in place.
- If the heater was not running (stopped, standby, already faulted) the reset simply keeps it off; the OLED then shows `Sensor reset n/3` and the page the reset progress.
- The temperature line keeps the last good value with a `*` (it is a few seconds old).
- If all three resets fail, `sensor_error` is raised and the PID is wiped like for any fault. The power cycle then **repeats once a minute** (VDD off 3 s) without clearing the fault, so a probe that comes back recovers after 3 good windows. A measurement-age timeout reports `sensor_stale` instead; both keep the heater off.
- Each power cycle holds DATA low together with VDD. Otherwise the pull-ups would feed the chip through its input protection diode and a latched probe would never lose power.
- The reset needs no network and runs even while the page is unreachable. A manual **Stop**, a target change or a new limit during a reset behave as usual; the on/off choice is never changed by a reset.

Where to see it:

| Where | What |
| --- | --- |
| OLED line 3 | Nothing while a running heater rides through the reset (`PID n% SSR:..`); `Sensor reset n/3` if the heater was off; `FAULT sensor_error` after giving up |
| `/api/status` | `"sensor":"resetting"`, `sensor_reset_attempt` (1-3), `sensor_power_cycles` (since boot); `mode` stays `pid` and `fault` stays `null` while the heater rides through, `fault` appears only when the resets fail (diagnostic fields, not shown on the page) |
| `/metrics` | `birdburner_sensor_resetting`, `birdburner_sensor_power_cycles_total`, `birdburner_sensor_recoveries_total` |
| Serial log | `[SENSOR] ...` phase changes and `[DS18B20] no valid reading: power-cycling the probe (reset 1/3, VDD off 1000 ms)` |

Reset timings are constants at the top of `src/sensor_recovery.rs` (`OFF_MS`, `RESPONSE_MS`, `RETRY_MS`, `RETRY_OFF_MS`, `MAX_ATTEMPTS`); the independent age guard is `SENSOR_RESET_MAX_AGE_MS` in `src/heater_control.rs`.

### Checking the wiring after the change

1. **With mains power disconnected**, check that the adapter VCC wire is connected to GPIO22 rather than 3V3. Only the low-voltage board should be powered for subsequent tests. Removing VCC alone is not a definitive test because DATA can parasitically power some probes.
2. **Reset path**: with heating stopped, disconnect DATA for about 10 s and reconnect it. Expect `Sensor reset 1/3` (possibly `2/3`), no fault, and `sensor_power_cycles` increasing. Disconnect it for 45 s and a sensor fault must appear; reconnect it and the fault clears after 3 good windows.
3. To verify PWM continuity, use an empty enclosure and a monitored test load. A short recovery must keep the commanded duty and the `pid` state; prolonged failure must force duty to zero. Stop must work during recovery. Do not run these tests with birds inside.

This checks recovery supervision; it does not prove a specific physical failure mechanism. `sensor_power_cycles_total` and `sensor_recoveries_total` count recovery activity.

## Firmware

- Rust `no_std` + Embassy; HTTP served by **picoserve** (4 workers), phone control page embedded (`src/index.html`).
- DS18B20 at **12-bit (0.0625 °C)**, displayed with two decimals. Readings are averaged per 5 s window: any successful reads in a window are averaged into one control sample and failed reads are dropped. A window with zero successful reads first triggers an automatic **probe power reset** (see above) with the heater carrying on; only if three resets fail is the heater cut and a fault reported.
- PID (heat-only) drives the SSR as **2 s time-proportional PWM** (not kHz PWM). Target 15–30 °C in 0.5 °C steps; default 25 °C.
- Software safety limit default **35 °C** (adjustable 25–40 °C on the web page, must exceed the target by ≥2 °C). At the limit the heater cuts off instantly; it resumes automatically 3 °C below the limit (hysteresis).
- **Persistent settings** (target, safety limit, max output, PID gains, on/off intent) survive power cuts (dual-sector, CRC-checked). Temperature readings and faults are never persisted.
- A probe power reset (see above) is not a fault and does not interrupt a running heater; only when it fails is the heater cut.
- Every fault cuts the heater and recovers automatically — there is no permanent lockout: sensor errors (after the resets failed) recover once the probe answers and 3 good windows follow; over-temperature recovers below limit−3 °C. After a power cut the heater stays off until 3 good samples, then resumes the saved on/off intent.
- SSR command Open/Closed is a software command only; there is no load-current feedback.

## WiFi credentials (never committed)

Credentials live in **`src/secrets.rs`**, which is gitignored:

```sh
cp src/secrets.example.rs src/secrets.rs   # then fill in SSID / PASSWORD
```

## Phone control

1. Connect the phone to the same WiFi, open the IP shown on the OLED bottom line (e.g. `http://192.168.50.135`).
2. In **Operating limits**, enter a temperature setpoint and press its **Set** button.
3. Tap **Start**. Status shows `AUTO / HEATING`; the output shows the commanded duty.
4. **Stop heating** opens the SSR immediately and saves the off intent.
5. Faults recover automatically; the alarm explains the cause. A short probe reset preserves the running status; cached temperatures have a `*`. A lost network connection is marked **OFFLINE**, not confused with a probe failure.
6. **Maximum output** (10–100 %) caps duty; **Temperature cutoff** (25–40 °C) sets the over-temperature threshold. Set each independently.
7. **PID parameters** accepts Kp/Ki/Kd together via **Apply PID**. Drafts are not overwritten by status polling. **Load defaults** only edits the form; it does not send values until Apply.
8. Save feedback appears in the panel that was changed. It says saving while a flash write is pending, then confirms success for 4 s. Wait for the success confirmation before RST or removing power. An HTTP success means applied in RAM, not necessarily already stored in flash.

OLED line 3 `PID 7% SSR:off` means the PID is running at 7 % duty and the SSR happens to be in the off part of its 2 s window. `resuming 1/3` means it is waiting for fresh valid readings before heating again. `Sensor reset 1/3` (only shown while the heater is off) means the probe stopped answering and is being power-cycled, not a fault yet; a running heater keeps its `PID` line during a reset.

## PID tuning

Tune from the phone page: the **PID parameters** panel has Kp, Ki, Kd and Tf boxes and an **Apply PID** button. The values are applied from the next 5 s sample and **saved to flash** (they survive power cuts, like the other settings). The same over HTTP: `POST /api/pid {"kp":20,"ki":0.02,"kd":120,"d_filter_s":30}` (see [APIDOC.md](APIDOC.md)); the current values are in `/api/status` (`pid_kp`, `pid_ki`, `pid_kd`, `pid_d_filter_s`) and `/metrics` (`birdburner_pid_kp/ki/kd`, `birdburner_pid_d_filter_seconds`), handy to overlay on the Grafana curves.

Factory starting values: Kp 20, Ki 0.02, Kd 120, Tf 30 s. The observed closed-loop data at 25 °C motivated reducing integral action and filtering the derivative. Ambient temperature was not measured and there was no controlled step test, so the thermal gain and delay are not identified; illustrative simulations are not hardware validation. Verify and tune on the cage, especially with the cover on vs off. Saved v4 gains are kept on upgrade and get the default Tf; v1–v3 records get all defaults. Use **Load defaults → Apply PID** to replace saved gains explicitly.

| Field | Meaning / unit | Allowed |
| --- | --- | --- |
| Kp | Proportional gain, output %/°C | 0–100 |
| Ki | Integral gain, output %/(°C·s) | 0–2 |
| Kd | Derivative gain (on measurement), output %·s/°C | 0–200 |
| Tf | Derivative first-order low-pass time constant, s (`d_filter_s`); α = dt/(Tf+dt), 0 = off | 0–300 |

Values are kept to 0.001. **Maximum output** (10–100 %) is the duty cap; the integral is clamped to it. Crossing the target does not clear the integral: slightly above target the output is P+I+D (I bleeds down through the negative error), and at target + 1 °C (`HEAT_CUTOFF_ABOVE_TARGET_C`) the output is forced to 0 while the integral is kept. The over-temperature cutoff is unchanged. Changing gains retains the integral unless Ki is set to zero, which clears it; gains can still change the output at the next sample. The factory defaults live in `src/heater_control.rs` (`DEFAULT_KP_MILLI`, `DEFAULT_KI_MILLI`, `DEFAULT_KD_MILLI`, `DEFAULT_D_FILTER_MILLI`, in thousandths).

Same file, at the top:

- `PWM_WINDOW_MS = 2000`: SSR time window (30 % ≈ 600 ms on / 1400 ms off).
- `MIN_AC_PULSE_MS = 20`: pulse quantisation for 50 Hz mains.
- `SAMPLE_WINDOW_MS = 5000`: one control sample per 5 s averaging window.
- `SENSOR_MAX_AGE_MS = 12_000`: sample older than this cuts the heater (except while a probe power reset is bridging it, see above).

Tuning order: set real safety limits and verify wiring first; start with `ki=0`, `kd=0`, tune `kp`; with Kd on, raise Tf if the output twitches on single 0.0625 °C steps; add `ki` only to remove steady-state offset; change one parameter at a time and watch the response.

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
curl -X POST http://$IP/api/pid        -H 'Content-Type: application/json' -d '{"kp":20,"ki":0.02,"kd":120,"d_filter_s":30}'
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

`[OLED] write error` = I2C failure (the firmware retries init once a second); check GPIO6/7 wiring and power. `[DS18B20]` errors: check DATA on GPIO23, VCC on GPIO22, common ground and the ~4.7 kΩ pull-up; keep the cable straight, uncoiled and away from mains wiring. Occasional CRC errors are tolerated by the 5 s averaging window; a window with zero successful reads starts probe recovery. A running heater retains its last duty for the bounded recovery interval; unsuccessful recovery or the independent age timeout cuts it. Repeated resets (`birdburner_sensor_power_cycles_total` climbing) mean the probe or its cable/supply is unhealthy: apply the hardening list above. `sensor_errors_by_kind_total` helps locate the failure class: `timing` includes early completion and timeout, `bad_data` is a rejected scratchpad, and `no_presence` means no reset response. These counters alone do not identify the electrical cause.

Standalone diagnostic firmware (flashing them replaces the main firmware):

```sh
cargo build --release --bin ds18b20_serial   # serial temperature diagnostics (powers the probe from GPIO22), SSR held off
cargo build --release --bin ssr_toggle       # GPIO15 toggles every 5 s, NO thermal protection — wiring test only
```

Host-side logic tests (no device, no GPIO):

```sh
cargo test --manifest-path tests/host/Cargo.toml \
  --target x86_64-unknown-linux-gnu --target-dir target/host-tests
node --test tests/frontend.test.mjs
```

Host tests cover control/recovery, flash-record migration, and the real picoserve API over loopback TCP. The dependency-free frontend tests cover polling, feedback, request retries, drafts and Stop preemption. Physical flash/GPIO/thermal behaviour must still be checked on the board.
