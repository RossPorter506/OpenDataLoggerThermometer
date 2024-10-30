use crate::{constants::*, pcb_mapping::{TempPowerPins, TempSensePins}};

/// (Temperate in milli-celcius, pulse counts)
const LMT01_LUT: [(i32, u16); LUT_SIZE] = [
    (-50_000, 26),
    (-40_000, 181),
    (-30_000, 338),
    (-20_000, 494),
    (-10_000, 651),
    (  0_000, 808),
    ( 10_000, 966),
    ( 20_000, 1125),
    ( 30_000, 1284),
    ( 40_000, 1443),
    ( 50_000, 1602),
    ( 60_000, 1762),
    ( 70_000, 1923),
    ( 80_000, 2084),
    ( 90_000, 2245),
    (100_000, 2407),
    (110_000, 2569),
    (120_000, 2731),
    (130_000, 2893),
    (140_000, 3057),
    (150_000, 3218)];

pub struct TempSensors {
    pub power: TempPowerPins,
    pub sense: TempSensePins,
    lut: LUTInterpolator,
}
impl TempSensors {
    pub fn new(power: TempPowerPins, sense: TempSensePins) -> Self {
        let lut = LUTInterpolator::new(LMT01_LUT);
        Self {power, sense, lut}
    }
    /// Read temperatures from sensors. Values returned in milli-celcius.
    pub fn read_temperatures(&self) -> [i32; NUM_SENSOR_CHANNELS] {
        let counts = self.read_counts();
        counts.map(|c| self.conv_count_to_temp_lut(c))
    }
    fn read_counts(&self) -> [u16; NUM_SENSOR_CHANNELS] {
        todo!()
    }
    
    /// Input: 0-3218 counts
    ///
    /// Output: Temperature in millicelcius (e.g. -50_000mC -> 150_000mC)
    fn conv_count_to_temp_lut(&self, pulse_count: u16) -> i32 {
        let pulse_count = pulse_count.min(3218);
        self.lut.interpolate(pulse_count)
    }
}


pub const CHARS_PER_READING: usize = 8;
// Does this fn belong here?
/// Input: -99_999 to 999_999 mC
///
/// Output: `[u8;8]`, e.g. "-50.012C", "002.901C", "234.750C"
pub fn temp_to_string(tempr: i32) -> [u8; CHARS_PER_READING] {
    let neg = tempr < 0;
    let mut tempr = tempr.clamp(-99_999, 999_999).unsigned_abs();
    let mut out = [0u8; CHARS_PER_READING];

    for i in 0..CHARS_PER_READING - 1 {
        if CHARS_PER_READING - 2 - i == 3 {
            continue;
        } // decimal place
        let digit = (tempr % 10) as u8;
        tempr /= 10;
        out[CHARS_PER_READING - 2 - i] = digit + b'0';
    }
    out[CHARS_PER_READING - 5] = b'.';
    out[CHARS_PER_READING - 1] = b'C';

    // Remove leading zeroes
    let mut i = 0;
    for _ in 0..2 {
        if out[i] == b'0' {
            out[i] = b' ';
            i += 1;
            continue;
        }
        break;
    }

    if neg {
        out[i.saturating_sub(1)] = b'-';
    }

    out
}

struct LUTEntry {
    count: u16,
    value: i32,
}
impl LUTEntry {
    fn new(count: u16, value: i32) -> LUTEntry {
        LUTEntry { count, value }
    }
}
const LUT_SIZE: usize = 21;
struct LUTInterpolator {
    lut: [LUTEntry; LUT_SIZE],
}
impl LUTInterpolator {
    fn new(arr: [(i32, u16); LUT_SIZE]) -> LUTInterpolator {
        let lut = arr.map(|x| LUTEntry::new(x.1, x.0));
        LUTInterpolator { lut }
    }
    fn interpolate(&self, test_count: u16) -> i32 {
        let test_count = test_count
            .max(self.lut[LUT_SIZE - 1].count)
            .min(self.lut[0].count);
        let mut below_index: usize = 0;
        for i in 0..LUT_SIZE {
            if test_count >= self.lut[i].count {
                below_index = i;
            } else {
                break;
            }
        }
        let above_index = below_index + 1;
        let below_count = self.lut[below_index].count;
        let above_count = self.lut[above_index].count;
        let below_temp = self.lut[below_index].value;
        let above_temp = self.lut[above_index].value;
        let ratio = (test_count - below_count) as f32 / (above_count - below_count) as f32;
        let result = above_temp as f32 * ratio + below_temp as f32 * (1.0 - ratio);
        
        num_traits::float::FloatCore::round(result)
            .max(self.lut[0].value as f32)
            .min(self.lut[LUT_SIZE - 1].value as f32) as i32
    }
}