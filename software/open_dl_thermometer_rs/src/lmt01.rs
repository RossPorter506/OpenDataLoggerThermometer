use crate::{constants::*, pcb_mapping::{TempPowerPins, TempSensePins}, pio::PioStateMachines};

/// (Temperate in milli-celcius, pulse counts)
const LMT01_LUT: [(i32, u32); LUT_SIZE] = [
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
    pub _sense: TempSensePins,
    pub pios: PioStateMachines,
    lut: LUTInterpolator,
}
impl TempSensors {
    pub fn new(power: TempPowerPins, _sense: TempSensePins, pios: PioStateMachines) -> Self {
        let lut = LUTInterpolator::new(LMT01_LUT);
        Self {power, _sense, pios, lut}
    }
    /// Read temperatures from sensors. Values returned in milli-celcius.
    pub fn read_temperatures(&mut self) -> [Option<i32>; NUM_SENSOR_CHANNELS] {
        let counts = self.read_counts();
        counts.map(|opt| opt.map(|c| self.conv_count_to_temp_lut(c)))
    }
    fn read_counts(&mut self) -> [Option<u32>; NUM_SENSOR_CHANNELS] {
        self.pios.p0sm0.tx.write(0);
        self.pios.p0sm1.tx.write(0);
        self.pios.p0sm2.tx.write(0);
        self.pios.p0sm3.tx.write(0);
        self.pios.p1sm0.tx.write(0);
        self.pios.p1sm1.tx.write(0);
        self.pios.p1sm2.tx.write(0);
        self.pios.p1sm3.tx.write(0);

       [self.pios.p0sm0.rx.read(),
        self.pios.p0sm1.rx.read(),
        self.pios.p0sm2.rx.read(),
        self.pios.p0sm3.rx.read(),
        self.pios.p1sm0.rx.read(),
        self.pios.p1sm1.rx.read(),
        self.pios.p1sm2.rx.read(),
        self.pios.p1sm3.rx.read()]
    }
    
    /// Input: 0-3218 counts
    ///
    /// Output: Temperature in millicelcius (e.g. -50_000mC -> 150_000mC)
    fn conv_count_to_temp_lut(&self, pulse_count: u32) -> i32 {
        let pulse_count = pulse_count.min(3218);
        self.lut.interpolate(pulse_count)
    }
}


pub const CHARS_PER_READING: usize = 8;
// Does this fn belong here?
/// Input: -99_999 to 999_999 mC
///
/// Output: `[u8;8]`, e.g. "-50.012C", "002.901C", "234.750C"
/// 
/// If input is `None` output is empty string
pub fn temp_to_string(tempr: Option<i32>) -> [u8; CHARS_PER_READING] {
    let mut out = [0u8; CHARS_PER_READING];
    if let Some(tempr) = tempr {
        let neg = tempr < 0;
        let mut tempr = tempr.clamp(-99_999, 999_999).unsigned_abs();

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
    }
    else {
        out = *b"        ";
    }
    out
}

struct LUTEntry {
    count: u32,
    value: i32,
}
impl LUTEntry {
    fn new(count: u32, value: i32) -> LUTEntry {
        LUTEntry { count, value }
    }
}
const LUT_SIZE: usize = 21;
struct LUTInterpolator {
    lut: [LUTEntry; LUT_SIZE],
}
impl LUTInterpolator {
    fn new(arr: [(i32, u32); LUT_SIZE]) -> LUTInterpolator {
        let lut = arr.map(|x| LUTEntry::new(x.1, x.0));
        LUTInterpolator { lut }
    }
    fn interpolate(&self, test_count: u32) -> i32 {
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