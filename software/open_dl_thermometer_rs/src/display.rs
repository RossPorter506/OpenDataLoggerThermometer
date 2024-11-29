use arrayvec::{ArrayString, ArrayVec};
use liquid_crystal::BusBits::Bus4Bits;
use liquid_crystal::LCD20X4;

use crate::config::Config;
use crate::pcb_mapping::{DisplayScl, DisplaySda};
use crate::sd_card::SdCardInfo;
use crate::{lmt01::CHARS_PER_READING, NUM_SENSOR_CHANNELS};

pub const DISPLAY_I2C_ADDRESS: u8 = 0x27;

type I2CInterface = liquid_crystal::I2C<rp_pico::hal::I2C<rp_pico::pac::I2C0, (DisplaySda, DisplayScl)>>;
/// Top-level component for managing the display. 
pub struct IncrementalDisplayWriter<'a> {
    screen: Screen,
    next_pos: Option<(usize, usize)>,
    driver: liquid_crystal::LiquidCrystal<'a, I2CInterface, {SCREEN_COLS as u8}, SCREEN_ROWS>,
}
impl<'a> IncrementalDisplayWriter<'a> {
    pub fn new(config: &Config, sdcard_info: SdCardInfo, delay: &mut impl liquid_crystal::DelayNs, liquid_crystal_i2c_interface: &'a mut I2CInterface) -> Self {
        let mut driver = liquid_crystal::LiquidCrystal::new(liquid_crystal_i2c_interface, Bus4Bits, LCD20X4);
        driver.begin(delay);
        Self {screen: determine_screen(config, sdcard_info, [None;NUM_SENSOR_CHANNELS]).0, next_pos: Some((0,0)), driver}
    }
    /// If the display isn't up to date then send one character to the display.
    /// 
    /// Done one character at a time to maximise responsiveness in a polling loop.
    /// 
    /// Returns `true` if another character needs to be sent, `false` if the display is up to date.
    pub fn incremental_update(&mut self, delay: &mut impl liquid_crystal::DelayNs) -> bool {
        if let Some((row,col)) = self.next_pos {
            let next_u8 = self.screen[row][col];
            self.driver.write(delay, liquid_crystal::Text(core::str::from_utf8(&[next_u8]).unwrap_or("#")));
            self.next_pos = match (row, col) {
                (SCREEN_MAX_ROW, SCREEN_MAX_COL) => None,               // Done
                (_             , SCREEN_MAX_COL) => {                   // Next row
                    self.driver.set_cursor(delay, row+1, 0); // Our display uses different row layouts in memory. If this is removed the rows are printed out in the wrong order.
                    Some((row+1, 0))
                },   
                (_             , _             ) => Some((row, col+1)), // Next column
            };
        }
        self.next_pos.is_some()
    }

    /// Change what we want to be displayed on the screen, and marks the screen as needing updating. 
    /// Does not actually update the display. You must call `incremental_update()`
    fn update_display_buffer(&mut self, screen: Screen) {
        self.screen = screen;
        self.next_pos = Some((0,0));
    }
    /// Determine what the screen should look like based on system state, then update the internal display buffer.
    /// 
    /// Does not actually redraw the display. You must call `incremental_update()`
    pub fn determine_new_screen(&mut self, delay: &mut impl liquid_crystal::DelayNs, config: &Config, sdcard_info: SdCardInfo, sensor_values: DisplayValues) {
        let (screen, cursor_pos) = determine_screen(config, sdcard_info, sensor_values);
        self.update_display_buffer(screen);

        // Set up cursor if needed
        if let Some((cursor_row, cursor_col)) = cursor_pos {
            self.driver.enable_cursor();
            self.driver.enable_blink();
            self.driver.set_cursor(delay, cursor_row, cursor_col as u8);
        }
        else {
            self.driver.disable_cursor();
            self.driver.disable_blink();
        }
        self.driver.update_config(delay);
    }

    /// Loads all our custom characters into the display's memory
    pub fn load_custom_chars(&mut self, delay: &mut impl liquid_crystal::DelayNs) {
        for char in [TICK, CROSS] {
            self.driver.custom_char(delay, &char.bitmap, char.address);
        }
    }
}

const SCREEN_COLS: usize = 20;
const SCREEN_ROWS: usize = 4;
const SCREEN_MAX_COL: usize = SCREEN_COLS-1;
const SCREEN_MAX_ROW: usize = SCREEN_ROWS-1;

type Line = [u8; SCREEN_COLS];
type Screen = [Line; SCREEN_ROWS];

/// The sensor values we display on the screen. Not all channels may be producing values, hence `Option`.
pub type DisplayValues = [Option<ArrayString<CHARS_PER_READING>>; NUM_SENSOR_CHANNELS];

/// Placeholder for elements that may change based on configuration or system state.
/// Designed to be overwritten at runtime with actual values.
const DYNAMIC_PLACEHOLDER:    u8 = b'@';

/// Placeholder element to designate positions where the 'cursor' could be displayed.
/// Exactly one of these should be replaced with '>' at runtime, all others should be replaced with ' '.
/// If an element is selectable then it's assumed to be dynamic too.
const SELECTABLE_PLACEHOLDER: u8 = b'%';

/// For elements that are both dynamic and selectable, namely the SD filename characters.
/// These should be filled out as dynamic elements, but if selected the blinking cursor should be activated
const DYNAMIC_AND_SELECTABLE_PLACEHOLDER: u8 = b'#';

const MAINMENU_SCREEN: Screen = [
    *b"OpenDL Thermometer  ",
    *b"%Configure          ",
    *b"%View               ",
    *b"%View and Log Data  "];

const CONFIG_OUTPUTS_SCREEN: Screen = [
    *b"Datalogger Outputs  ",
    *b"%SD Card:    @@@    ",
    *b"%USB Serial: @@@    ",
    *b"               %NEXT"];

const CONFIG_SD_STATUS_SCREEN: Screen = [
    *b"SD Detected: @@@    ",
    *b"Writable:    @@@    ",
    *b"Formatted:   @@@    ",
    *b"Free: @@@@@MB  %NEXT"];

const CONFIG_SD_FILENAME_SCREEN: Screen = [
    *b"SD Card Filename:   ",
    *b"     ##########     ",
    *b"%Type: @@@          ",
    *b"               %NEXT"];

const CONFIG_CHANNEL_SELECT_SCREEN: Screen = [
    *b"Record Channels:    ",
    *b"1: @@@ 2: @@@ 3: @@@",
    *b"4: @@@ 5: @@@ 6: @@@",
    *b"7: @@@ 8: @@@  %NEXT"];

const CONFIG_SAMPLE_RATE_SCREEN: Screen = [
    *b"%Sample Rate:@@@/sec",
    *b"             @@@kB/h",
    *b"                    ",
    *b"               %NEXT"];

const VIEW_SCREEN: Screen = [
    *b"1: @@@@@@  2: @@@@@@",
    *b"3: @@@@@@  4: @@@@@@",
    *b"5: @@@@@@  6: @@@@@@",
    *b"7: @@@@@@  8: @@@@@@"];

const CONFIRM_STOP_LOGGING_SCREEN: Screen = [
    *b"Stop logging?       ",
    *b"               %NO  ",
    *b"                    ",
    *b"               %YES "];

const SD_FULL_SCREEN: Screen = [
    *b"ERROR: SD Card Full!",
    *b"Continue without SD?",
    *b"               %YES ",
    *b"               %NO  "];

const SD_REMOVED_SCREEN: Screen = [
    *b"ERR: SD Card Removed",
    *b"Continue without SD?",
    *b"               %YES ",
    *b"               %NO  "];

const SD_ERROR_SCREEN: Screen = [
    *b"ERR: Unknwn SD Error",
    *b"Continue without SD?",
    *b"               %YES ",
    *b"               %NO  "];

const SD_WRITING_SCREEN: Screen = [
    *b"Writing to SD card  ",
    *b"Do not remove...    ",
    *b"                    ",
    *b"                    "];

const SD_WRITE_COMPLETE_SCREEN: Screen = [
    *b"SD write complete!  ",
    *b"Now safe to remove  ",
    *b"                    ",
    *b"               %NEXT"];

const MAX_DYNAMIC_ELEMENTS: usize = 8;


struct CustomCharacter {
    address: u8,
    bitmap: [u8; 8]
}

const TICK: CustomCharacter = CustomCharacter{
    address:0, 
    bitmap: [
        0b11111, //
        0b11111, // 
        0b11110, //     #
        0b11101, //    #
        0b01011, // # #
        0b10111, //  #
        0b11111, // 
        0b11111, // 
    ]
};

const CROSS: CustomCharacter = CustomCharacter{
    address:1, 
    bitmap: [
        0b11111, // 
        0b11111, // 
        0b01110, // #   #
        0b10101, //  # #
        0b11011, //   #
        0b10101, //  # #
        0b01110, // #   #
        0b11111, // 
    ]
};

/// Sets all placeholder selectables to ' ', except for the one current selected, ordered by standard reading order (top left to bottom right)
fn substitute_selected_elements(screen: &mut Screen, selected_pos: Option<usize>) -> Option<(usize, usize)> {
    let mut cursor_pos = None;
    if let Some(selected_pos) = selected_pos {
        // Iterate over the screen. Filter out non-placeholders. Replace placeholders with either '>' or ' ' depending on index 
        // sel_pos = index of selectable (i.e. this is the nth selectable), idx = position in array (i.e. row 2 column 14 = index 54), col = contents of column
        for (sel_pos, (idx, col)) in 
            screen.as_flattened_mut().iter_mut().enumerate() // flatten 2D array to 1D, and enumerate all elements
            .filter(|&(_, &mut col)| [SELECTABLE_PLACEHOLDER, DYNAMIC_AND_SELECTABLE_PLACEHOLDER].contains(&col) ) // filter out non-selectable elements
            .enumerate() { // enumerate selectable elements
            match (sel_pos, *col) {
                (pos, SELECTABLE_PLACEHOLDER            ) if pos == selected_pos    => *col = b'>',
                (_  , SELECTABLE_PLACEHOLDER            )                           => *col = b' ',
                (pos, DYNAMIC_AND_SELECTABLE_PLACEHOLDER) if pos == selected_pos    => cursor_pos = Some((idx/SCREEN_COLS, idx%SCREEN_COLS)), // cursor on
                (_  , DYNAMIC_AND_SELECTABLE_PLACEHOLDER)                           => (), // dealt with by the dynamic
                (_  , _                                 )                           => (), // do nothing
            };
        }
    }
    cursor_pos
}

#[derive(Default)]
struct DynamicElement {
    row: usize,
    start_col: usize,
    end_col: usize,
}

/// Find the locations of dynamic elements within a Screen. 
fn find_dynamic_element_placeholders(screen: &Screen) -> ArrayVec<DynamicElement, MAX_DYNAMIC_ELEMENTS> {
    let mut elements: ArrayVec<DynamicElement, MAX_DYNAMIC_ELEMENTS> = ArrayVec::new();
    let mut dyn_elem = DynamicElement::default();
    for (i, &rw) in screen.iter().enumerate() {
        let mut is_in_dynamic_placeholder = false;
        for (j, &col) in rw.iter().enumerate() {
            if col == DYNAMIC_PLACEHOLDER || col == DYNAMIC_AND_SELECTABLE_PLACEHOLDER {
                if is_in_dynamic_placeholder {
                    dyn_elem.end_col += 1;
                }
                else {
                    is_in_dynamic_placeholder = true;
                    dyn_elem.row = i;
                    dyn_elem.start_col = j;
                    dyn_elem.end_col = j+1;
                }
            }
            else if is_in_dynamic_placeholder {
                elements.push(dyn_elem);
                dyn_elem = DynamicElement::default();
                is_in_dynamic_placeholder = false;
            }
        }
        if is_in_dynamic_placeholder{
            // Dynamic elements can't span more than one row, so end it now
            elements.push(dyn_elem);
            dyn_elem = DynamicElement::default();
        }
    }
    elements
}

fn right_align(arr: Vec<u8>, total_size: usize) -> Vec<u8> {
    let pad_amount = total_size - arr.len();
    let padding = core::iter::repeat_n(b' ', pad_amount);
    padding.chain(arr).collect()
}

fn int_to_vec_u8(n: usize) -> Vec<u8> {
    let mut v = Vec::new();
    write!(v.as_mut_slice(), "{n}").unwrap();
    v
}

trait ToStr { fn to_str(&self) -> Vec<u8>; }
impl<T: ToStr> ToStr for Option<T> {
    fn to_str(&self) -> Vec<u8> {
        match self {
            Some(t) => t.to_str(),
            None => b"N/A".into(),
        }
    }
}
impl ToStr for bool {
    fn to_str(&self) -> Vec<u8> {  if *self {b"YES"} else {b"NO "}.into()  }
}

use usbd_serial::embedded_io::Write;
extern crate alloc;
use alloc::vec::Vec;
/// Replace dynamic placeholders with actual values
fn substitute_dynamic_elements(screen: &mut Screen, config: &Config, sdcard_info: SdCardInfo, sensor_values: DisplayValues) {
    let dynamic_element_locations = find_dynamic_element_placeholders(screen);
    // Inner Vec<u8> can't be an ArrayVec because it doesn't handle different length strings correctly for some reason
    let mut dynamic_strs = ArrayVec::<Vec<u8>, MAX_DYNAMIC_ELEMENTS>::new();
    match config.curr_state {
        ConfigOutputs(_) => {
            dynamic_strs.push(config.serial.selected_for_use.to_str());
            dynamic_strs.push(config.sd.selected_for_use.to_str());
        },
        ConfigSDStatus(_) => {
            dynamic_strs.push(sdcard_info.is_inserted.to_str());   // SD detected
            dynamic_strs.push(sdcard_info.is_writable.to_str());   // SD writable
            dynamic_strs.push(sdcard_info.is_formatted.to_str());   // SD formatted
            // Free space
            const MAX_FREE_SPACE_CHARS: usize = 5;
            match sdcard_info.free_space_bytes {
                Some(free_bytes) => {let free_space_megabytes: usize = free_bytes as usize / 1_000_000;
                let free_space_str = int_to_vec_u8(free_space_megabytes);
                if free_space_str.len() < MAX_FREE_SPACE_CHARS {
                    dynamic_strs.push(right_align(free_space_str, MAX_FREE_SPACE_CHARS));
                }
                else {
                    dynamic_strs.push(b">9999".into());
                }},
                None => dynamic_strs.push(b" N/A ".into()),
            }
             
        },
        ConfigSDFilename(_) => {
            dynamic_strs.push(config.sd.filename.into());
            dynamic_strs.push(config.sd.filetype.as_str().into());
        },
        ConfigChannelSelect(_) => {
            for is_enabled in config.enabled_channels {
                dynamic_strs.push(is_enabled.to_str());
            }
        },
        ConfigSampleRate(_) => {
            dynamic_strs.push( right_align(int_to_vec_u8(config.samples_per_sec as usize), 3) );
            // Samples per sec * bytes per sample * seconds per hour * number of active channels * kB per B = kB/hr
            let kb_per_hr = config.samples_per_sec as usize * CHARS_PER_READING * 3600 * config.enabled_channels.iter().filter(|&&x| x).count() / 1000 ;
            dynamic_strs.push( right_align(int_to_vec_u8(kb_per_hr), 3) );
        },
        ViewTemperatures(_) | DatalogTemperatures(_) => {
            /// How many characters we have to display a reading
            const NUM_DISP_CHARS: usize = 6; 
            for opt_value in sensor_values {
                // Truncate the sensor reading to fit on the display and reappend the 'C' on the end, if enabled
                let sensor_str: Vec<u8> = if let Some(sensor_value) = opt_value {
                    let mut v = Vec::from(&sensor_value[..NUM_DISP_CHARS-1]); 
                    v.push(b'C'); 
                    v
                } 
                else { [b' '; NUM_DISP_CHARS].into() };
                dynamic_strs.push(sensor_str);
            }
        },
        _ => (),
    }
    for (loc, str) in dynamic_element_locations.iter().zip(dynamic_strs) {
        screen[loc.row][loc.start_col..loc.end_col].clone_from_slice(&str);
    }
}

use crate::state_machine::State::*;
/// Determine what the screen should look like based on system state
fn determine_screen(config: &Config, sdcard_info: SdCardInfo, sensor_values: DisplayValues) -> (Screen, Option<(usize, usize)>) {
    let (selected_element_pos, mut display_buffer) = match config.curr_state {
        Mainmenu(sel) =>                    (Some(sel as usize), MAINMENU_SCREEN),
        ConfigOutputs(sel) =>               (Some(sel as usize), CONFIG_OUTPUTS_SCREEN),
        ConfigSDStatus(sel) =>              (Some(sel as usize), CONFIG_SD_STATUS_SCREEN),
        ConfigSDFilename(sel) =>            (Some(sel as usize), CONFIG_SD_FILENAME_SCREEN),
        ConfigChannelSelect(sel) =>         (Some(sel as usize), CONFIG_CHANNEL_SELECT_SCREEN),
        ConfigSampleRate(sel) =>            (Some(sel as usize), CONFIG_SAMPLE_RATE_SCREEN),
        ViewTemperatures(_) =>              (None,               VIEW_SCREEN),
        DatalogTemperatures(_) =>           (None,               VIEW_SCREEN),
        DatalogConfirmStop(sel) =>          (Some(sel as usize), CONFIRM_STOP_LOGGING_SCREEN),
        DatalogErrorSDFull(sel) =>          (Some(sel as usize), SD_FULL_SCREEN),
        DatalogSDWriting(_) =>              (None,               SD_WRITING_SCREEN),
        DatalogSDSafeToRemove(sel) =>       (Some(sel as usize), SD_WRITE_COMPLETE_SCREEN),
        DatalogSDUnexpectedRemoval(sel) =>  (Some(sel as usize), SD_REMOVED_SCREEN),
        DatalogSDError(sel) =>              (Some(sel as usize), SD_ERROR_SCREEN),
    };

    let cursor_pos = substitute_selected_elements(&mut display_buffer, selected_element_pos);
    
    substitute_dynamic_elements(&mut display_buffer, config, sdcard_info, sensor_values);

    (display_buffer, cursor_pos)
}
