use rp_pico::{hal::pio::{InstalledProgram, PIOExt, UninitStateMachine}, pac};

use crate::{constants::*, pcb_mapping::{TempPowerPins, TempSensePins}, pio::{AllPioStateMachines, PioStateMachine}};

/// Maximum time required for an LMT01 to convert and communicate one sensor reading
pub const SENSOR_MAX_TIME_FOR_READING_MS: u32 = 104;

/// (Temperature in milli-celcius, pulse counts)
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

/// Top-level struct for controlling the LMT01 temperature sensors
pub struct TempSensors {
    power: TempPowerPins,
    _sense: TempSensePins,
    pios: AllPioStateMachines,
    lut: LUTInterpolator,
}
impl TempSensors {
    pub fn new(power: TempPowerPins, _sense: TempSensePins, pios: AllPioStateMachines) -> Self {
        let lut = LUTInterpolator::new(LMT01_LUT);
        Self {power, _sense, pios, lut}
    }
    /// Try to read temperatures from all sensors. Values returned in milli-celcius.
    /// 
    /// Returns `None` if the sensor did not respond. Powers off sensors and pauses PIOs afterwards.
    pub fn read_temperatures_and_end_conversion(&mut self) -> [Option<i32>; NUM_SENSOR_CHANNELS] {
        let counts = self.read_counts_and_end_conversion();
        
        counts.map(|opt| opt.map(|c| self.conv_count_to_temp_lut(c)))
    }
    /// Try to read counts from all sensors. Powers off sensors and pauses PIOs afterwards.
    fn read_counts_and_end_conversion(&mut self) -> [Option<u32>; NUM_SENSOR_CHANNELS] {
        self.pios.write_all(0);
        // The pulse counter counts downwards from 0xFFFF_FFFF, so invert the bits to get the actual number of pulses
        let vals = self.pios.read_all();
        self.pios.pause_all();
        self.power.turn_off();
        
        vals.map(|opt| opt.map(|c| !c))
    }
    
    /// Input: 26-3218 counts
    ///
    /// Output: Temperature in millicelcius (e.g. -50_000mC -> 150_000mC)
    fn conv_count_to_temp_lut(&self, pulse_count: u32) -> i32 {
        let pulse_count = pulse_count.clamp(26, 3218);
        self.lut.interpolate(pulse_count)
    }

    /// Enables the LMT01 sensors and ensures the PIOs are prepared to read the result
    pub fn begin_conversion(&mut self) {
        self.power.turn_on();
        self.pios.restart_all();
    }

    /// Try to get a reading from all sensor channels. If a channel does not send any pulses then `false`, otherwise `true`.
    /// 
    /// Blocks for `SENSOR_MAX_TIME_FOR_READING_MS`. Turns off sensors and pauses PIOs afterwards.
    pub fn autodetect_sensor_channels(&mut self, delay: &mut impl embedded_hal::delay::DelayNs) -> [bool; NUM_SENSOR_CHANNELS] {
        self.begin_conversion();

        delay.delay_ms(SENSOR_MAX_TIME_FOR_READING_MS+1);
        let connected = self.read_counts_and_end_conversion().map(|opt| opt.is_some());

        self.pios.pause_all();
        self.power.turn_off();

        connected
    }
}

pub const CHARS_PER_READING: usize = 8;
// Does this fn belong here?
/// Input: -99_999 to 999_999 mC
///
/// Output: `[u8;8]`, e.g. `b"-50.012C"`, `b"  2.901C"`, `b"234.750C"`
/// 
/// If input is `None` output is `b"        "`
pub fn temp_to_string(tempr: Option<i32>) -> Option<[u8; CHARS_PER_READING]> {
    let tempr = tempr?;
    let mut out = [0u8; CHARS_PER_READING];

    let neg = tempr < 0;
    let mut tempr = tempr.clamp(-99_999, 999_999).unsigned_abs();

    for i in 0..CHARS_PER_READING - 1 {
        if CHARS_PER_READING - 2 - i == 3 { // Decimal place
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
    
    Some(out)
}

/// Look up table entry
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
    /// Interpolate between values in a lookup table using a weighted average.
    fn interpolate(&self, test_count: u32) -> i32 {
        // Restrict input to within the extremes of the LUT
        let test_count = test_count.clamp(self.lut[0].count, self.lut[LUT_SIZE - 1].count);

        // Find the two neighbouring points that are above and below the test value
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

        // Do a weighted average
        let ratio = (test_count - below_count) as f32 / (above_count - below_count) as f32;
        let test_temp = (above_temp as f32 * ratio) + (below_temp as f32 * (1.0 - ratio));
        
        // Round to nearest integer. TODO: Second clamp probably not necessary. 
        num_traits::float::FloatCore::round(test_temp)
            .clamp(self.lut[0].value as f32, self.lut[LUT_SIZE - 1].value as f32) as i32
    }
}

pub fn configure_pios_for_lmt01(pio0: pac::PIO0, pio1: pac::PIO1, resets: &mut pac::RESETS, sense_pins: &TempSensePins) -> AllPioStateMachines {
    let program = pio_proc::pio_file!("src/lmt01_pulse_count.pio");
    
    let (mut pio0, p0sm0, p0sm1, p0sm2, p0sm3) = pio0.split(resets);
    let (mut pio1, p1sm0, p1sm1, p1sm2, p1sm3) = pio1.split(resets);
    let Ok(installed0) = pio0.install(&program.program) else { unreachable!() };
    let Ok(installed1) = pio1.install(&program.program) else { unreachable!() };

    // Cloning program handle is unsafe in the case where uninstall() is called while another machine is still running the program
    // Safety: uninstall() is simply never called
    let p0sm0 = configure_sm_for_lmt01(p0sm0, sense_pins.vn1.id().num, unsafe{installed0.share()});
    let p0sm1 = configure_sm_for_lmt01(p0sm1, sense_pins.vn2.id().num, unsafe{installed0.share()});
    let p0sm2 = configure_sm_for_lmt01(p0sm2, sense_pins.vn3.id().num, unsafe{installed0.share()});
    let p0sm3 = configure_sm_for_lmt01(p0sm3, sense_pins.vn4.id().num, unsafe{installed0.share()});

    let p1sm0 = configure_sm_for_lmt01(p1sm0, sense_pins.vn5.id().num, unsafe{installed1.share()});
    let p1sm1 = configure_sm_for_lmt01(p1sm1, sense_pins.vn6.id().num, unsafe{installed1.share()});
    let p1sm2 = configure_sm_for_lmt01(p1sm2, sense_pins.vn7.id().num, unsafe{installed1.share()});
    let p1sm3 = configure_sm_for_lmt01(p1sm3, sense_pins.vn8.id().num, unsafe{installed1.share()});

    AllPioStateMachines::new(p0sm0, p0sm1, p0sm2, p0sm3, p1sm0, p1sm1, p1sm2, p1sm3)
}

fn configure_sm_for_lmt01<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex>(uninit_sm: UninitStateMachine<(PIO,SM)>, id_num: u8, prog: InstalledProgram<PIO>) -> crate::pio::PioStateMachine<PIO,SM> {
    let (mut state_machine, rx, tx) = rp_pico::hal::pio::PIOBuilder::from_installed_program(prog)
        .set_pins(id_num, 1)
        .build(uninit_sm);
    // The GPIO pin needs to be configured as an input
    state_machine.set_pindirs([(id_num, rp_pico::hal::pio::PinDir::Input)]);
    PioStateMachine::new(state_machine, rx, tx)
}
