//! Power-loss-safe settings: two alternating 4 KiB flash sectors, each record
//! sequence-numbered and CRC-checked. The newest valid record wins, so a power
//! cut mid-write falls back to the previous record. Measurements are never stored.
//! v2 added the safety limit, v3 the max heater output, v4 the PID gains, v5 the derivative filter;
//! older records still load (with defaults for the missing fields).

use crate::heater_control::{
    DEFAULT_D_FILTER_MILLI, DEFAULT_KD_MILLI, DEFAULT_KI_MILLI, D_FILTER_RANGE_MILLI, DEFAULT_KP_MILLI, DEFAULT_MAX_OUTPUT_PCT, KD_RANGE_MILLI, KI_RANGE_MILLI,
    KP_RANGE_MILLI, LIMIT_RANGE_MC, MAX_OUTPUT_RANGE_PCT, STEP_MC, TARGET_MARGIN_MC, TARGET_RANGE_MC,
};

/// Region after the factory partition (ends at 0xFB0000 on this 16 MB flash).
pub const SLOT_ADDR: [u32; 2] = [0xFF0000, 0xFF1000];
const MAGIC_V1: u32 = 0x4244_5331; // "BDS1": target + on/off, CRC at 16
const MAGIC_V2: u32 = 0x4244_5332; // "BDS2": + safety limit, CRC at 20
const MAGIC_V3: u32 = 0x4244_5333; // "BDS3": + max output in byte 13, CRC at 20
const MAGIC_V4: u32 = 0x4244_5334; // "BDS4": + Kp/Ki/Kd (thousandths, u32 at 20/24/28), CRC at 32
const MAGIC_V5: u32 = 0x4244_5335; // "BDS5": + derivative filter Tf (thousandths of s, u32 at 32), CRC at 36
pub const RECORD_LEN: usize = 40;
/// Used until the user saves a limit (and when loading v1 records).
pub const DEFAULT_MAX_MC: i32 = 35_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Settings {
    pub target_mc: i32,
    pub desired_enabled: bool,
    pub max_temperature_mc: i32,
    pub max_output_pct: u8,
    /// PID gains in thousandths.
    pub kp_milli: u32,
    pub ki_milli: u32,
    pub kd_milli: u32,
    /// Derivative low-pass time constant in thousandths of a second.
    pub d_filter_milli: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            target_mc: 25_000, desired_enabled: false, max_temperature_mc: DEFAULT_MAX_MC, max_output_pct: DEFAULT_MAX_OUTPUT_PCT,
            kp_milli: DEFAULT_KP_MILLI, ki_milli: DEFAULT_KI_MILLI, kd_milli: DEFAULT_KD_MILLI,
            d_filter_milli: DEFAULT_D_FILTER_MILLI,
        }
    }
}

pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 { crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 }; }
    }
    !crc
}

pub fn encode(seq: u32, s: Settings) -> [u8; RECORD_LEN] {
    let mut r = [0u8; RECORD_LEN];
    r[0..4].copy_from_slice(&MAGIC_V5.to_le_bytes());
    r[4..8].copy_from_slice(&seq.to_le_bytes());
    r[8..12].copy_from_slice(&s.target_mc.to_le_bytes());
    r[12] = s.desired_enabled as u8;
    r[13] = s.max_output_pct;
    // r[14..16] reserved, zero.
    r[16..20].copy_from_slice(&s.max_temperature_mc.to_le_bytes());
    r[20..24].copy_from_slice(&s.kp_milli.to_le_bytes());
    r[24..28].copy_from_slice(&s.ki_milli.to_le_bytes());
    r[28..32].copy_from_slice(&s.kd_milli.to_le_bytes());
    r[32..36].copy_from_slice(&s.d_filter_milli.to_le_bytes());
    let crc = crc32(&r[..36]);
    r[36..40].copy_from_slice(&crc.to_le_bytes());
    r
}

pub fn decode(r: &[u8; RECORD_LEN]) -> Option<(u32, Settings)> {
    let word = |i: usize| u32::from_le_bytes([r[i], r[i + 1], r[i + 2], r[i + 3]]);
    let defaults = (DEFAULT_KP_MILLI, DEFAULT_KI_MILLI, DEFAULT_KD_MILLI);
    let tf = DEFAULT_D_FILTER_MILLI;
    let (max_temperature_mc, max_output_pct, (kp_milli, ki_milli, kd_milli), d_filter_milli) = match word(0) {
        MAGIC_V1 if word(16) == crc32(&r[..16]) => (DEFAULT_MAX_MC, DEFAULT_MAX_OUTPUT_PCT, defaults, tf),
        MAGIC_V2 if word(20) == crc32(&r[..20]) => (word(16) as i32, DEFAULT_MAX_OUTPUT_PCT, defaults, tf),
        MAGIC_V3 if word(20) == crc32(&r[..20]) => (word(16) as i32, r[13], defaults, tf),
        // User-saved v4 gains are kept; the filter they were tuned without gets the default.
        MAGIC_V4 if word(32) == crc32(&r[..32]) => (word(16) as i32, r[13], (word(20), word(24), word(28)), tf),
        MAGIC_V5 if word(36) == crc32(&r[..36]) => (word(16) as i32, r[13], (word(20), word(24), word(28)), word(32)),
        _ => return None,
    };
    let target_mc = word(8) as i32;
    let on_grid = |v: i32| v % STEP_MC == 0;
    if r[12] > 1 || !TARGET_RANGE_MC.contains(&target_mc) || !on_grid(target_mc)
        || !LIMIT_RANGE_MC.contains(&max_temperature_mc) || !on_grid(max_temperature_mc)
        || max_temperature_mc < target_mc + TARGET_MARGIN_MC
        || !MAX_OUTPUT_RANGE_PCT.contains(&max_output_pct)
        || !KP_RANGE_MILLI.contains(&kp_milli) || !KI_RANGE_MILLI.contains(&ki_milli) || !KD_RANGE_MILLI.contains(&kd_milli)
        || !D_FILTER_RANGE_MILLI.contains(&d_filter_milli) { return None; }
    Some((word(4), Settings { target_mc, desired_enabled: r[12] == 1, max_temperature_mc, max_output_pct, kp_milli, ki_milli, kd_milli, d_filter_milli }))
}

/// Pick the newest valid slot. Returns (settings, seq, slot index of newest).
pub fn choose(slots: [Option<(u32, Settings)>; 2]) -> Option<(Settings, u32, usize)> {
    match slots {
        [Some(a), Some(b)] => Some(if b.0.wrapping_sub(a.0) as i32 > 0 { (b.1, b.0, 1) } else { (a.1, a.0, 0) }),
        [Some(a), None] => Some((a.1, a.0, 0)),
        [None, Some(b)] => Some((b.1, b.0, 1)),
        [None, None] => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const S: Settings = Settings { target_mc: 27_500, desired_enabled: true, max_temperature_mc: 33_000, max_output_pct: 70, kp_milli: 12_500, ki_milli: 250, kd_milli: 1_500, d_filter_milli: 12_000 };

    #[test]
    fn roundtrip_and_corruption() {
        let r = encode(7, S);
        assert_eq!(decode(&r), Some((7, S)));
        for i in 0..RECORD_LEN {
            let mut bad = r;
            bad[i] ^= 0x10;
            assert_eq!(decode(&bad), None, "byte {i}");
        }
        assert_eq!(decode(&[0xff; RECORD_LEN]), None); // erased flash
    }

    #[test]
    fn v1_record_loads_with_default_limit_and_ignores_old_fault_byte() {
        let mut r = [0u8; RECORD_LEN];
        r[0..4].copy_from_slice(&0x4244_5331u32.to_le_bytes());
        r[4..8].copy_from_slice(&9u32.to_le_bytes());
        r[8..12].copy_from_slice(&27_500i32.to_le_bytes());
        r[12] = 1;
        r[13] = 1; // old latched over-temperature code
        let crc = crc32(&r[..16]);
        r[16..20].copy_from_slice(&crc.to_le_bytes());
        r[20..24].copy_from_slice(&[0xff; 4]); // rest of the erased sector
        let d = Settings::default();
        assert_eq!(decode(&r), Some((9, Settings { max_temperature_mc: DEFAULT_MAX_MC, max_output_pct: DEFAULT_MAX_OUTPUT_PCT, kp_milli: d.kp_milli, ki_milli: d.ki_milli, kd_milli: d.kd_milli, d_filter_milli: d.d_filter_milli, ..S })));
    }

    #[test]
    fn v2_record_loads_with_default_max_output() {
        let mut r = encode(4, S);
        r[0..4].copy_from_slice(&0x4244_5332u32.to_le_bytes());
        r[13] = 0; // v2 left this byte zero
        let crc = crc32(&r[..20]);
        r[20..24].copy_from_slice(&crc.to_le_bytes());
        let d = Settings::default();
        assert_eq!(decode(&r), Some((4, Settings { max_output_pct: DEFAULT_MAX_OUTPUT_PCT, kp_milli: d.kp_milli, ki_milli: d.ki_milli, kd_milli: d.kd_milli, d_filter_milli: d.d_filter_milli, ..S })));
    }

    #[test]
    fn v3_record_loads_with_default_gains() {
        let mut r = [0xffu8; RECORD_LEN]; // rest of the erased sector
        r[..24].copy_from_slice(&encode(5, S)[..24]);
        r[0..4].copy_from_slice(&0x4244_5333u32.to_le_bytes());
        r[20..24].fill(0);
        let crc = crc32(&r[..20]);
        r[20..24].copy_from_slice(&crc.to_le_bytes());
        let d = Settings::default();
        assert_eq!(decode(&r), Some((5, Settings { kp_milli: d.kp_milli, ki_milli: d.ki_milli, kd_milli: d.kd_milli, d_filter_milli: d.d_filter_milli, ..S })));
        assert_eq!((d.kp_milli, d.ki_milli, d.kd_milli, d.d_filter_milli), (20_000, 20, 120_000, 30_000));
    }

    #[test]
    fn v4_record_keeps_gains_and_gets_default_filter() {
        let mut r = [0xffu8; RECORD_LEN];
        r[..32].copy_from_slice(&encode(6, S)[..32]);
        r[0..4].copy_from_slice(&0x4244_5334u32.to_le_bytes());
        let crc = crc32(&r[..32]);
        r[32..36].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode(&r), Some((6, Settings { d_filter_milli: DEFAULT_D_FILTER_MILLI, ..S })));
    }

    #[test]
    fn d_filter_roundtrips_and_is_range_checked() {
        for tf in [0, 30_000, 300_000] {
            let s = Settings { d_filter_milli: tf, ..S };
            assert_eq!(decode(&encode(2, s)), Some((2, s)), "{tf}");
        }
        assert_eq!(decode(&encode(2, Settings { d_filter_milli: 300_001, ..S })), None);
    }

    #[test]
    fn v4_gains_roundtrip_and_are_range_checked_even_with_valid_crc() {
        for gains in [(0, 0, 0), (100_000, 2_000, 200_000), (10_000, 100, 0)] {
            let s = Settings { kp_milli: gains.0, ki_milli: gains.1, kd_milli: gains.2, ..S };
            assert_eq!(decode(&encode(3, s)), Some((3, s)), "{gains:?}");
        }
        for gains in [(100_001, 100, 0), (10_000, 2_001, 0), (10_000, 100, 200_001), (u32::MAX, 0, 0)] {
            let s = Settings { kp_milli: gains.0, ki_milli: gains.1, kd_milli: gains.2, ..S };
            assert_eq!(decode(&encode(3, s)), None, "{gains:?}");
        }
    }

    #[test]
    fn rejects_invalid_max_output() {
        for pct in [0, 9, 101, 255] {
            assert_eq!(decode(&encode(1, Settings { max_output_pct: pct, ..S })), None, "{pct}");
        }
    }

    #[test]
    fn rejects_invalid_limit_even_with_valid_crc() {
        for max in [24_000, 41_000, 33_200, 29_000] { // last: below target 27.5 + 2
            assert_eq!(decode(&encode(1, Settings { max_temperature_mc: max, ..S })), None, "{max}");
        }
    }

    #[test]
    fn rejects_out_of_range_target_even_with_valid_crc() {
        assert_eq!(decode(&encode(1, Settings { target_mc: 40_000, ..S })), None);
        assert_eq!(decode(&encode(1, Settings { target_mc: 25_100, ..S })), None);
    }

    #[test]
    fn newest_wins_with_wraparound_and_torn_write() {
        let a = Some((5, Settings { target_mc: 20_000, ..S }));
        let b = Some((6, S));
        assert_eq!(choose([a, b]), Some((S, 6, 1)));
        assert_eq!(choose([b, a]), Some((S, 6, 0)));
        assert_eq!(choose([Some((u32::MAX, S)), Some((0, Settings { target_mc: 15_000, ..S }))]).unwrap().1, 0);
        assert_eq!(choose([a, None]), Some((a.unwrap().1, 5, 0)));
        assert_eq!(choose([None, None]), None);
    }
}
