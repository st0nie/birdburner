#![no_std]
#![no_main]

//! GPIO15 logic-level test ONLY: LOW for 5s, HIGH for 5s, repeat.
//! No WiFi, temperature setpoint, sensor protection, or heater regulation.
//! Disconnect heater/load power before testing and verify SSR input polarity.
//! LOW is a voltage level, NOT a guaranteed OFF state for an unknown SSR.
//! GPIO15 is a strapping pin: external circuitry affects its reset-time level.
//! Firmware cannot ensure a safe output level during boot/reset/power loss.

use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // A hung/panicked MCU can leave the output at its last level.
    // This test is not a replacement for an independent thermal cutoff.
    println!("panic: {:?}; GPIO15 may retain its last level -- disconnect load power", info);
    loop {}
}

#[main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 32 * 1024);
    let mut ssr = Output::new(p.GPIO15, Level::Low, OutputConfig::default());
    let start = Instant::now();
    let period = Duration::from_millis(5000);
    let delay = Delay::new();
    let mut high = false;

    println!("GPIO15 SSR logic test: WiFi OFF; no temperature control/protection");
    println!("WARNING: test with heater/load power disconnected; verify SSR polarity");
    println!("[SSR] t={} ms GPIO15=LOW", start.elapsed().as_millis());

    let mut next_toggle = start + period;
    loop {
        let now = Instant::now();
        if now < next_toggle {
            delay.delay(next_toggle - now);
        }
        high = !high;
        ssr.set_level(if high { Level::High } else { Level::Low });
        // Schedule from the actual edge so there are no short catch-up pulses.
        let edge = Instant::now();
        next_toggle = edge + period;
        println!(
            "[SSR] t={} ms GPIO15={}",
            edge.duration_since_epoch().as_millis() - start.duration_since_epoch().as_millis(),
            if high { "HIGH" } else { "LOW" },
        );
    }
}
