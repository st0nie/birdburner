//! DS18B20 data validation; also testable on the host without the ESP HAL.

#[derive(Debug, PartialEq, Eq)]
pub enum DataError {
    Crc { expected: u8, received: u8 },
    InvalidScratchpad,
    OutOfRange,
}

pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        let mut value = byte;
        for _ in 0..8 {
            let mix = (crc ^ value) & 1;
            crc >>= 1;
            if mix != 0 {
                crc ^= 0x8c;
            }
            value >>= 1;
        }
    }
    crc
}

pub fn validate_crc(bytes: &[u8]) -> Result<(), DataError> {
    let expected = crc8(&bytes[..bytes.len() - 1]);
    let received = bytes[bytes.len() - 1];
    if expected != received {
        return Err(DataError::Crc { expected, received });
    }
    Ok(())
}

pub fn temperature_millicelsius(scratchpad: &[u8; 9]) -> Result<i32, DataError> {
    validate_crc(scratchpad)?;
    // These fixed bits distinguish a real scratchpad from a stuck-low bus,
    // whose all-zero data would otherwise pass the CRC check.
    if scratchpad[4] & 0x9f != 0x1f || scratchpad[5] != 0xff || scratchpad[7] != 0x10 {
        return Err(DataError::InvalidScratchpad);
    }
    let raw = i16::from_le_bytes([scratchpad[0], scratchpad[1]]);
    // At reduced resolution the low bits are undefined.
    let mask = match (scratchpad[4] >> 5) & 3 {
        0 => !7i16,
        1 => !3i16,
        2 => !1i16,
        _ => !0i16,
    };
    let raw = raw & mask;
    if !(-55 * 16..=125 * 16).contains(&raw) {
        return Err(DataError::OutOfRange);
    }
    Ok(i32::from(raw) * 1000 / 16)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratchpad(raw: i16, config: u8) -> [u8; 9] {
        let [low, high] = raw.to_le_bytes();
        let mut bytes = [low, high, 0x4b, 0x46, config, 0xff, 0x0c, 0x10, 0];
        bytes[8] = crc8(&bytes[..8]);
        bytes
    }

    #[test]
    fn known_datasheet_crc_and_temperature() {
        let bytes = [0x50, 0x05, 0x4b, 0x46, 0x7f, 0xff, 0x0c, 0x10, 0x1c];
        assert_eq!(crc8(&bytes[..8]), 0x1c);
        assert_eq!(temperature_millicelsius(&bytes), Ok(85_000));
    }

    #[test]
    fn signed_temperatures() {
        assert_eq!(temperature_millicelsius(&scratchpad(401, 0x7f)), Ok(25_062));
        assert_eq!(temperature_millicelsius(&scratchpad(-162, 0x7f)), Ok(-10_125));
        assert_eq!(temperature_millicelsius(&scratchpad(-8, 0x7f)), Ok(-500));
    }

    #[test]
    fn rejects_corrupted_data() {
        let mut bytes = scratchpad(400, 0x7f);
        bytes[0] ^= 1;
        assert!(matches!(temperature_millicelsius(&bytes), Err(DataError::Crc { .. })));
    }

    #[test]
    fn rejects_stuck_bus_and_out_of_range() {
        assert_eq!(temperature_millicelsius(&[0; 9]), Err(DataError::InvalidScratchpad));
        assert_eq!(temperature_millicelsius(&scratchpad(126 * 16, 0x7f)), Err(DataError::OutOfRange));
        assert_eq!(temperature_millicelsius(&scratchpad(-56 * 16, 0x7f)), Err(DataError::OutOfRange));
    }

    #[test]
    fn masks_undefined_resolution_bits() {
        assert_eq!(temperature_millicelsius(&scratchpad(407, 0x1f)), Ok(25_000));
        assert_eq!(temperature_millicelsius(&scratchpad(407, 0x3f)), Ok(25_250));
        assert_eq!(temperature_millicelsius(&scratchpad(407, 0x5f)), Ok(25_375));
    }
}
