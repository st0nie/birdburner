//! Diagnostic: DS18B20 on GPIO23 printed over serial. WiFi and SSR untouched
//! (GPIO15 is driven LOW). Reuses the main firmware's driver.
#![no_std]
#![no_main]

#[path = "../ds18b20.rs"]
mod ds18b20;
#[path = "../ds18b20_data.rs"]
mod ds18b20_data;
#[path = "../sensor_display.rs"]
#[allow(dead_code)] // diagnostic only uses the formatter
mod sensor_display;

use esp_hal::{delay::Delay, gpio::{Flex, Level, Output, OutputConfig}, main};
use esp_println::println;
use sensor_display::DisplayTemperature;

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", info);
    loop {}
}

#[main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 16 * 1024);
    let _ssr_off = Output::new(p.GPIO15, Level::Low, OutputConfig::default());
    let mut sensor = ds18b20::Ds18b20::new(Flex::new(p.GPIO23));
    let delay = Delay::new();
    println!("DS18B20 diagnostic: DATA=GPIO23, 12-bit; SSR held LOW");
    loop {
        match sensor.start_conversion() {
            Ok(rom) => {
                let result = loop {
                    delay.delay_millis(10);
                    if let Some(r) = sensor.poll_temperature() { break r; }
                };
                match result {
                    Ok(mc) => println!("[DS18B20] {:02x?} {} C (CRC OK)", rom, DisplayTemperature(mc)),
                    Err(e) => println!("[DS18B20] ERROR ({}): {}", e.kind(), e),
                }
            }
            Err(e) => println!("[DS18B20] ERROR ({}): {}", e.kind(), e),
        }
        delay.delay_millis(250);
    }
}
