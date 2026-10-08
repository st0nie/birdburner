//! Single externally powered DS18B20; conversion waiting never blocks WiFi.
//!
//! The probe's VDD comes from a GPIO, not from the 3.3 V rail. A probe that latches up keeps
//! answering on the bus but never converts again, and no 1-Wire command can clear that: only
//! removing VDD can. `power_off` / `power_on` let the sensor task do exactly that.

use crate::ds18b20_data::{DataError, temperature_millicelsius, validate_crc};
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Flex, Output, OutputConfig, Pull},
    time::Instant,
};

#[derive(Debug)]
pub enum SensorError {
    BusStuckLow,
    NoPresence,
    Data(DataError),
    WrongFamily(u8),
    ParasitePowerUnsupported,
    ConversionTimeout,
    NotConverting,
    ResolutionMismatch,
    PowerOnValue,
    EarlyCompletion,
}

impl SensorError {
    /// Short label for metrics; groups errors by likely physical cause.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::BusStuckLow | Self::NoPresence => "no_presence",
            Self::Data(DataError::Crc { .. }) => "crc",
            Self::Data(_) | Self::WrongFamily(_) => "bad_data",
            Self::PowerOnValue | Self::ResolutionMismatch => "sensor_reset",
            Self::ConversionTimeout | Self::NotConverting | Self::EarlyCompletion => "timing",
            Self::ParasitePowerUnsupported => "power",
        }
    }
}

impl core::fmt::Display for SensorError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BusStuckLow => f.write_str("bus stuck LOW; check short/pull-up"),
            Self::NoPresence => f.write_str("no presence; check DATA (GPIO23), VDD (GPIO22), GND, 4.7k pull-up"),
            Self::Data(error) => write!(f, "{:?}", error),
            Self::WrongFamily(family) => write!(f, "wrong family 0x{:02x}, expected 0x28", family),
            Self::ParasitePowerUnsupported => f.write_str("parasite power unsupported; connect VDD to GPIO22 (3.3V when on)"),
            Self::ConversionTimeout => f.write_str("12-bit conversion timeout (>1000ms)"),
            Self::NotConverting => f.write_str("no conversion started"),
            Self::ResolutionMismatch => f.write_str("sensor lost 12-bit configuration; reinitializing"),
            Self::PowerOnValue => f.write_str("85.00 C power-on value: sensor reset/brownout, check VDD/GND"),
            Self::EarlyCompletion => f.write_str("conversion 'done' too early: noise on DATA line"),
        }
    }
}

/// Datasheet: 12-bit conversion typ. ~600 ms, max 750 ms. Anything faster is noise.
const MIN_CONVERSION_MS: u64 = 300;
/// Extra scratchpad reads after a CRC error (same conversion, no new 750 ms wait).
const SCRATCHPAD_RETRIES: usize = 2;

pub struct Ds18b20<'d> {
    pin: Flex<'d>,
    /// Probe VDD. Held high; dropped only to power-cycle a probe that stopped converting.
    power: Output<'d>,
    delay: Delay,
    conversion_started: Option<Instant>,
    configured_rom: Option<[u8; 8]>,
}

impl<'d> Ds18b20<'d> {
    /// `pin` is DATA, `power` drives the probe's VDD (it is switched on here).
    pub fn new(mut pin: Flex<'d>, mut power: Output<'d>) -> Self {
        power.set_high();
        pin.set_high();
        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        Self { pin, power, delay: Delay::new(), conversion_started: None, configured_rom: None }
    }

    /// Cut the probe's power. DATA is pulled low first and stays low: with DATA released, the
    /// pull-ups would feed the chip through its input protection diode and a latched-up probe
    /// would never lose power. Forgets the 12-bit configuration; the next conversion re-inits.
    pub fn power_off(&mut self) {
        self.conversion_started = None;
        self.configured_rom = None;
        self.pin.set_low();
        self.power.set_low();
    }

    /// Restore the probe's power. DATA stays low until `release_data`, so the pull-ups do not
    /// race the supply ramp.
    pub fn power_on(&mut self) {
        self.power.set_high();
    }

    /// Let DATA float high again (open drain, pulled up) once VDD is stable.
    pub fn release_data(&mut self) {
        self.pin.set_high();
    }

    fn reset(&mut self) -> Result<(), SensorError> {
        // Protect only timing-sensitive slots, NOT the up-to-750ms conversion.
        critical_section::with(|_| {
            self.pin.set_high();
            self.delay.delay_micros(10);
            if self.pin.is_low() {
                return Err(SensorError::BusStuckLow);
            }
            self.pin.set_low();
            self.delay.delay_micros(480);
            self.pin.set_high();
            self.delay.delay_micros(70);
            let presence = self.pin.is_low();
            self.delay.delay_micros(410);
            if self.pin.is_low() {
                Err(SensorError::BusStuckLow)
            } else if !presence {
                Err(SensorError::NoPresence)
            } else {
                Ok(())
            }
        })
    }

    fn write_bit(&mut self, bit: bool) {
        critical_section::with(|_| {
            self.pin.set_low();
            self.delay.delay_micros(if bit { 6 } else { 60 });
            self.pin.set_high();
            self.delay.delay_micros(if bit { 64 } else { 10 });
        });
    }

    fn read_bit(&mut self) -> bool {
        critical_section::with(|_| {
            self.pin.set_low();
            self.delay.delay_micros(3);
            self.pin.set_high();
            self.delay.delay_micros(10);
            let bit = self.pin.is_high();
            self.delay.delay_micros(57);
            bit
        })
    }

    fn write_byte(&mut self, byte: u8) {
        for bit in 0..8 {
            self.write_bit(byte & (1 << bit) != 0);
        }
    }

    fn read_byte(&mut self) -> u8 {
        let mut byte = 0;
        for bit in 0..8 {
            if self.read_bit() {
                byte |= 1 << bit;
            }
        }
        byte
    }

    fn read_rom(&mut self) -> Result<[u8; 8], SensorError> {
        self.reset()?;
        self.write_byte(0x33); // Read ROM: ONE sensor only
        let mut rom = [0; 8];
        for byte in &mut rom {
            *byte = self.read_byte();
        }
        validate_crc(&rom).map_err(SensorError::Data)?;
        if rom[0] != 0x28 {
            return Err(SensorError::WrongFamily(rom[0]));
        }
        Ok(rom)
    }

    fn initialize_12_bit(&mut self) -> Result<[u8; 8], SensorError> {
        let rom = self.read_rom()?;
        self.reset()?;
        self.write_byte(0xcc);
        self.write_byte(0xb4);
        if !self.read_bit() { return Err(SensorError::ParasitePowerUnsupported); }
        let scratchpad = self.read_scratchpad_bytes()?;
        self.reset()?;
        self.write_byte(0xcc);
        self.write_byte(0x4e); // Write scratchpad; preserve alarm bytes.
        self.write_byte(scratchpad[2]);
        self.write_byte(scratchpad[3]);
        self.write_byte(0x7f); // R1:R0=11 => 12 bit / 0.0625 C / max 750ms.
        // No Copy Scratchpad (0x48): avoid wearing EEPROM every boot/error.
        let verify = self.read_scratchpad_bytes()?;
        if verify[4] != 0x7f { return Err(SensorError::ResolutionMismatch); }
        Ok(rom)
    }

    pub fn start_conversion(&mut self) -> Result<[u8; 8], SensorError> {
        self.conversion_started = None;
        let result = (|| {
            let rom = match self.configured_rom {
                Some(rom) => rom,
                None => self.initialize_12_bit()?,
            };
            self.reset()?;
            self.write_byte(0xcc);
            self.write_byte(0x44);
            self.configured_rom = Some(rom);
            self.conversion_started = Some(Instant::now());
            Ok(rom)
        })();
        if result.is_err() { self.configured_rom = None; }
        result
    }

    /// Poll about every 1-2ms. None means conversion is still in progress.
    /// Do not wait a full 100ms after each success; pipeline the next conversion.
    pub fn poll_temperature(&mut self) -> Option<Result<i32, SensorError>> {
        let Some(start) = self.conversion_started else {
            return Some(Err(SensorError::NotConverting));
        };
        let elapsed_ms = start.elapsed().as_millis();
        if elapsed_ms >= 1000 {
            self.conversion_started = None;
            self.configured_rom = None;
            return Some(Err(SensorError::ConversionTimeout));
        }
        if elapsed_ms < 10 || !self.read_bit() {
            return None;
        }
        self.conversion_started = None;
        // A real 12-bit conversion takes hundreds of ms. A "done" bit this
        // early is a glitch on DATA, and the scratchpad still holds old data.
        if elapsed_ms < MIN_CONVERSION_MS {
            self.configured_rom = None;
            return Some(Err(SensorError::EarlyCompletion));
        }
        // Reading the scratchpad is non-destructive: on a CRC error (noise on a
        // long line) re-read the same conversion result instead of failing.
        let mut read = self.read_scratchpad_bytes();
        for _ in 0..SCRATCHPAD_RETRIES {
            if !matches!(read, Err(SensorError::Data(DataError::Crc { .. }))) { break; }
            read = self.read_scratchpad_bytes();
        }
        let result = read.and_then(|scratchpad| {
            if scratchpad[4] != 0x7f { return Err(SensorError::ResolutionMismatch); }
            // 0x0550 = 85.000 C is the register's power-on reset value: the sensor
            // rebooted (supply glitch) and never converted. Never treat it as real.
            if scratchpad[0] == 0x50 && scratchpad[1] == 0x05 { return Err(SensorError::PowerOnValue); }
            temperature_millicelsius(&scratchpad).map_err(SensorError::Data)
        });
        if result.is_err() { self.configured_rom = None; }
        Some(result)
    }

    fn read_scratchpad_bytes(&mut self) -> Result<[u8; 9], SensorError> {
        self.reset()?;
        self.write_byte(0xcc);
        self.write_byte(0xbe);
        let mut scratchpad = [0; 9];
        for byte in &mut scratchpad {
            *byte = self.read_byte();
        }
        validate_crc(&scratchpad).map_err(SensorError::Data)?;
        // Validate fixed bits/range as well; never accept a stuck-low CRC=0.
        temperature_millicelsius(&scratchpad).map_err(SensorError::Data)?;
        Ok(scratchpad)
    }
}
