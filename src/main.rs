#![no_std]
#![no_main]
extern crate alloc;

mod ds18b20;
mod ds18b20_data;
mod heater_control;
mod http_api;
mod persist;
mod sensor_display;
mod sensor_recovery;

use alloc::{format, rc::Rc};
use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::Timer;
use embedded_graphics::{mono_font::{ascii::FONT_6X10, MonoTextStyle}, pixelcolor::BinaryColor, prelude::*, text::Text};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_hal::{
    Blocking,
    gpio::{Flex, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    time::{Instant, Rate},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{self, AuthMethod, ClientConfig, ModeConfig, WifiController, WifiDevice};
use esp_storage::FlashStorage;
use heater_control::SafetyLimits;
use http_api::{AppState, Shared};
use persist::{Settings, RECORD_LEN, SLOT_ADDR};
use sensor_recovery::{MAX_ATTEMPTS, Phase, PowerCycle};
use ssd1306::{Ssd1306, prelude::*};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi credentials are local-only (gitignored src/secrets.rs), never committed.
mod secrets;
use secrets::{PASSWORD, SSID};
/// 100 W heater. The cutoff here is only the first-boot default (35 C at the
/// cage-centre probe); the user can change it from the web page and it is saved
/// to flash. Software only: an independent thermal cutoff is still required.
const HEATER_SAFETY: Option<SafetyLimits> =
    Some(SafetyLimits { max_temperature_mc: persist::DEFAULT_MAX_MC, heater_watts: 100 });

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    critical_section::with(|_| {
        // Force the SSR input low (heater off) before anything else.
        let _pin = Output::new(unsafe { esp_hal::peripherals::GPIO15::steal() }, Level::Low, OutputConfig::default());
        println!("panic: {}; SSR forced LOW", info);
        loop { core::hint::spin_loop(); }
    })
}

fn now_ms() -> u64 { Instant::now().duration_since_epoch().as_millis() }

type Oled = Ssd1306<I2CInterface<I2c<'static, Blocking>>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>>;

// ---------- persistence ----------

fn read_slot(flash: &mut FlashStorage, i: usize) -> Option<(u32, Settings)> {
    let mut buf = [0u8; RECORD_LEN];
    flash.read(SLOT_ADDR[i], &mut buf).ok()?;
    persist::decode(&buf)
}

/// Returns saved settings, the last sequence number and the slot to write next.
fn load_settings(flash: &mut FlashStorage) -> (Settings, u32, usize) {
    match persist::choose([read_slot(flash, 0), read_slot(flash, 1)]) {
        Some((s, seq, slot)) => {
            println!("[STORE] restored target={} limit={} max_out={}% desired_on={} (seq {})", s.target_mc, s.max_temperature_mc, s.max_output_pct, s.desired_enabled, seq);
            (s, seq, 1 - slot)
        }
        None => {
            println!("[STORE] no saved settings; defaults (target 25 C, limit 35 C, max output 100%, off)");
            (Settings::default(), 0, 0)
        }
    }
}

fn write_slot(flash: &mut FlashStorage, slot: usize, seq: u32, s: Settings) -> bool {
    let addr = SLOT_ADDR[slot];
    let record = persist::encode(seq, s);
    flash.erase(addr, addr + FlashStorage::SECTOR_SIZE).is_ok()
        && flash.write(addr, &record).is_ok()
        && read_slot(flash, slot) == Some((seq, s))
}

#[embassy_executor::task]
async fn storage_task(mut flash: FlashStorage, shared: Shared, mut seq: u32, mut slot: usize) {
    loop {
        Timer::after_millis(500).await;
        {
            let mut app = shared.borrow_mut();
            if !app.settings_dirty { continue; }
            app.settings_dirty = false;
        }
        // Debounce slider drags: wait, then save whatever is current.
        Timer::after_millis(1500).await;
        let s = shared.borrow().settings();
        seq = seq.wrapping_add(1);
        let ok = write_slot(&mut flash, slot, seq, s);
        if ok { slot = 1 - slot; }
        println!("[STORE] save target={} limit={} max_out={}% desired_on={} -> {}", s.target_mc, s.max_temperature_mc, s.max_output_pct, s.desired_enabled, if ok { "ok" } else { "FAILED" });
        let mut app = shared.borrow_mut();
        app.storage_ok = ok;
        app.storage_writes_total += 1;
        if !ok {
            app.storage_failures_total += 1;
            app.settings_dirty = true; // retry
        }
    }
}

// ---------- network ----------

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) { runner.run().await; }

#[embassy_executor::task]
async fn wifi_task(mut wifi: WifiController<'static>, stack: Stack<'static>, shared: Shared) {
    loop {
        if !wifi.is_connected().unwrap_or(false) {
            {
                let mut app = shared.borrow_mut();
                app.wifi_connected = false;
                app.ip = None;
            }
            println!("[WiFi] connecting to {}", SSID);
            if let Err(e) = wifi.connect() { println!("[WiFi] connect error: {:?}", e); }
            for _ in 0..150 {
                if wifi.is_connected().unwrap_or(false) { break; }
                Timer::after_millis(100).await;
            }
        }
        let connected = wifi.is_connected().unwrap_or(false);
        let ip = if connected { stack.config_v4().map(|c| format!("{}", c.address.address())) } else { None };
        {
            let mut app = shared.borrow_mut();
            if ip != app.ip { println!("[WiFi] IP={:?}", ip); }
            app.wifi_connected = connected;
            app.ip = ip;
        }
        Timer::after_millis(if connected { 1000 } else { 2000 }).await;
    }
}

const HTTP_TASKS: usize = 4;

#[embassy_executor::task(pool_size = HTTP_TASKS)]
async fn http_task(id: usize, stack: Stack<'static>, shared: Shared) {
    let app = http_api::router(shared);
    let config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: picoserve::time::Duration::from_secs(5),
        persistent_start_read_request: picoserve::time::Duration::from_secs(5),
        read_request: picoserve::time::Duration::from_secs(5),
        write: picoserve::time::Duration::from_secs(5),
    }).close_connection_after_response();
    if id == 0 { println!("[HTTP] listening on :80 ({} workers)", HTTP_TASKS); }
    picoserve::Server::new(&app, &config, &mut [0u8; 2048])
        .listen_and_serve(id, stack, 80, &mut [0u8; 1024], &mut [0u8; 4096]).await;
}

// ---------- sensor / relay / display ----------

/// After VDD returns the DS18B20 needs a moment before it answers a reset pulse.
const PROBE_POWER_ON_SETTLE_MS: u64 = 100;

/// Power-cycle the probe: DATA low + VDD off for `cycle.off_ms`, then VDD back, DATA released and
/// a settle time. Fully async so the heater, WiFi and HTTP keep running meanwhile.
async fn power_cycle_probe(sensor: &mut ds18b20::Ds18b20<'static>, cycle: PowerCycle) {
    if cycle.attempt > 0 {
        println!("[DS18B20] no valid reading: power-cycling the probe (reset {}/{}, VDD off {} ms)", cycle.attempt, MAX_ATTEMPTS, cycle.off_ms);
    } else {
        println!("[DS18B20] probe still silent: periodic power cycle (VDD off {} ms)", cycle.off_ms);
    }
    sensor.power_off();
    Timer::after_millis(cycle.off_ms).await;
    sensor.power_on();
    Timer::after_millis(20).await;
    sensor.release_data();
    Timer::after_millis(PROBE_POWER_ON_SETTLE_MS).await;
}

#[embassy_executor::task]
async fn sensor_task(mut sensor: ds18b20::Ds18b20<'static>, shared: Shared) {
    // VDD was only just switched on in main(): let the probe start up.
    Timer::after_millis(PROBE_POWER_ON_SETTLE_MS).await;
    let mut stats_at = now_ms();
    let (mut count, mut errors) = (0u32, 0u32);
    loop {
        let due = shared.borrow_mut().take_power_cycle();
        if let Some(cycle) = due { power_cycle_probe(&mut sensor, cycle).await; }
        match sensor.start_conversion() {
            Ok(_) => loop {
                Timer::after_millis(10).await;
                // A power cycle was requested: abandon this conversion, the loop top handles it.
                if shared.borrow().power_cycle_pending() { break; }
                if let Some(result) = sensor.poll_temperature() {
                    match result {
                        Ok(mc) => {
                            let back = shared.borrow_mut().raw_reading(mc, now_ms());
                            count += 1;
                            if back { println!("[DS18B20] probe answers again"); }
                        }
                        Err(e) => { shared.borrow_mut().raw_error(now_ms(), e.kind()); errors += 1; println!("[DS18B20] {}", e); }
                    }
                    break;
                }
            },
            Err(e) => {
                shared.borrow_mut().raw_error(now_ms(), e.kind());
                errors += 1;
                println!("[DS18B20] {}", e);
                Timer::after_millis(200).await;
            }
        }
        let ms = now_ms();
        if ms.saturating_sub(stats_at) >= 5000 {
            let mut app = shared.borrow_mut();
            app.sample_hz = count as f64 * 1000.0 / (ms - stats_at) as f64;
            println!("[TEMP] {} | ok={} err={} {:.2}Hz | mode={} pid={:?} duty={:.0}%",
                app.temp_text(), count, errors, app.sample_hz, app.control.mode(),
                app.control.pid_output_pct(), app.control.commanded_duty_pct(ms));
            stats_at = ms;
            count = 0;
            errors = 0;
        }
    }
}

#[embassy_executor::task]
async fn relay_task(mut pin: Output<'static>, shared: Shared) {
    let mut previous = false;
    let mut phase = Phase::Healthy;
    loop {
        let (close, now_phase) = {
            let mut app = shared.borrow_mut();
            (app.tick(now_ms()), app.recovery.phase())
        };
        pin.set_level(if close { Level::High } else { Level::Low });
        if close != previous {
            println!("[SSR] {}", if close { "ON" } else { "OFF" });
            previous = close;
        }
        if now_phase != phase {
            match now_phase {
                Phase::Healthy => println!("[SENSOR] answering again; heating resumes after 3 good windows"),
                Phase::Resetting(n) => println!("[SENSOR] no valid reading for a whole window: heater held off, reset {}/{} (not a fault yet)", n, MAX_ATTEMPTS),
                Phase::Failed => println!("[SENSOR] still silent after {} resets: FAULT sensor_error, retrying once a minute", MAX_ATTEMPTS),
            }
            phase = now_phase;
        }
        Timer::after_millis(5).await;
    }
}

#[embassy_executor::task]
async fn oled_task(mut display: Oled, shared: Shared) {
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let mut next_reinit = 0;
    loop {
        let started = now_ms();
        let lines = {
            let app = shared.borrow();
            let c = &app.control;
            let state = match c.fault() {
                Some(f) => format!("FAULT {}", f.label()),
                None if app.recovery.resetting() => format!("Sensor reset {}/{}", app.recovery.attempt(), MAX_ATTEMPTS),
                None if c.enabled() => format!("PID {:.0}% SSR:{}", c.commanded_duty_pct(app.now_ms),
                    if app.relay_closed { "on" } else { "off" }),
                None if c.desired_enabled() => format!("resuming {}/3", c.recovery_samples()),
                None => format!("{}", c.mode()),
            };
            [
                format!("Temp: {}", app.temp_text()),
                format!("Set:  {:.1} C", c.target_mc() as f64 / 1000.0),
                state,
                app.ip.as_ref().map(|ip| format!("{}", ip)).unwrap_or_else(|| format!("WiFi...")),
            ]
        };
        display.clear(BinaryColor::Off).ok();
        for (i, text) in lines.iter().enumerate() {
            Text::new(text, Point::new(0, 10 + i as i32 * 12), style).draw(&mut display).ok();
        }
        if let Err(e) = display.flush() {
            if started >= next_reinit {
                println!("[OLED] write error: {:?}; reinitializing", e);
                display.init().ok();
                next_reinit = started + 1000;
            }
        }
        Timer::after_millis(500u64.saturating_sub(now_ms() - started).max(1)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    // Heater off before anything else runs.
    let pin = Output::new(p.GPIO15, Level::Low, OutputConfig::default());
    esp_alloc::heap_allocator!(size: 128 * 1024);
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw.software_interrupt0);

    let mut flash = FlashStorage::new();
    let (saved, seq, next_slot) = load_settings(&mut flash);
    let shared = Rc::new(RefCell::new(AppState::new(HEATER_SAFETY, saved)));

    // Probe VDD comes from GPIO22 (not the 3.3 V rail) so a latched-up DS18B20 can be power-cycled.
    let probe_vdd = Output::new(p.GPIO22, Level::High, OutputConfig::default());
    let sensor = ds18b20::Ds18b20::new(Flex::new(p.GPIO23), probe_vdd);
    let i2c = I2c::new(p.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(400))).unwrap()
        .with_sda(p.GPIO6).with_scl(p.GPIO7);
    let mut display = Ssd1306::new(ssd1306::I2CDisplayInterface::new(i2c), DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    if let Err(e) = display.init() { println!("[OLED] init error: {:?}", e); }

    // Local control loop runs regardless of network state.
    spawner.spawn(relay_task(pin, shared.clone())).unwrap();
    spawner.spawn(sensor_task(sensor, shared.clone())).unwrap();
    spawner.spawn(oled_task(display, shared.clone())).unwrap();
    spawner.spawn(storage_task(flash, shared.clone(), seq, next_slot)).unwrap();
    println!("birdburner: 12-bit DS18B20 averaged over 5 s, PID 2s window, adjustable cutoff, settings persisted");

    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().unwrap());
    let (mut wifi, interfaces) = wifi::new(radio, p.WIFI, wifi::Config::default()).unwrap();
    wifi.set_config(&ModeConfig::Client(ClientConfig::default()
        .with_ssid(SSID.into()).with_password(PASSWORD.into()).with_auth_method(AuthMethod::WpaWpa2Personal))).unwrap();
    wifi.start().unwrap();
    // Limit TX power: the board browned out on full-power WiFi bursts.
    unsafe { esp_wifi_sys::include::esp_wifi_set_max_tx_power(52); }

    // DHCP + one TCP socket per HTTP worker.
    static RESOURCES: StaticCell<StackResources<{ HTTP_TASKS + 1 }>> = StaticCell::new();
    let seed = (now_ms() as u64) ^ 0xe8f6_0af8_bbd0;
    let (stack, runner) = embassy_net::new(interfaces.sta, embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()), seed);
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(wifi_task(wifi, stack, shared.clone())).unwrap();
    for id in 0..HTTP_TASKS {
        spawner.spawn(http_task(id, stack, shared.clone())).unwrap();
    }
    loop { Timer::after_secs(3600).await; }
}
