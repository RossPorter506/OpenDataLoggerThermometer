use arrayvec::ArrayVec;

use crate::config::Config;
use crate::{lmt01::CHARS_PER_READING, NUM_SENSOR_CHANNELS};

const SCREEN_WIDTH: usize = 20;
const SCREEN_ROWS:  usize = 4;

type Line = [u8; SCREEN_WIDTH];
type Screen = [Line; SCREEN_ROWS];

/// Placeholder for elements that may change based on configuration or system state.
/// Designed to be overwritten at runtime with actual values.
const DYNAMIC_PLACEHOLDER:    u8 = b'@';

/// Placeholder element to designate positions where the 'cursor' could be displayed.
/// Exactly one of these should be replaced with '>' at runtime, all others should be replaced with ' '.
/// If an element is selectable then it's assumed to  be dynamic too.
const SELECTABLE_PLACEHOLDER: u8 = b'%';

/// For elements that are both dynamic and selectable, namely the SD filename characters.
/// These should be filled out as dynamic elements, but one may need to be selected too.
const DYNAMIC_AND_SELECTABLE_PLACEHOLDER: u8 = b'#';

struct DynamicElement {
    length: usize,
    row: usize,
    start_col: usize,
}

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
    *b"Format:      @@@    ",
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
    *b"           @@@kB/hr ",
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

/// Sets all placeholder selectables to ' ', except for the one current selected, ordered by standard reading order (top left to bottom right)
fn set_selected_elements(screen: &mut Screen, selected_pos: Option<usize>) {
    if let Some(selected_pos) = selected_pos {
        // Iterate over the screen. Filter out non-placeholders. Replace placeholders with either '>' or ' ' depending on index 
        for (idx, col) in screen.as_flattened_mut().iter_mut().filter(|&&mut col| col.eq(&SELECTABLE_PLACEHOLDER) ).enumerate() {
            if idx == selected_pos {*col = b'>';} else {*col = b' ';}
        }
    }
}
fn find_dynamic_element_placeholders(screen: &Screen) -> ArrayVec<DynamicElement, MAX_DYNAMIC_ELEMENTS> {
    let mut elements: ArrayVec<DynamicElement, MAX_DYNAMIC_ELEMENTS> = ArrayVec::new();
    let mut is_in_dynamic_placeholder = false;
    let mut length = 0;
    let mut row = 0;
    let mut start_col = 0;
    for (i, rw) in screen.iter().enumerate() {
        for (j, col) in rw.iter().enumerate() {
            if *col == DYNAMIC_PLACEHOLDER || *col == DYNAMIC_AND_SELECTABLE_PLACEHOLDER {
                if is_in_dynamic_placeholder == false {
                    is_in_dynamic_placeholder = true;
                    length = 1;
                    row = i;
                    start_col = j;
                }
                else {
                    length += 1;
                }
            }
            else if is_in_dynamic_placeholder {
                elements.push(DynamicElement{length, row, start_col});
                is_in_dynamic_placeholder = false;
            }
        }
    }
    elements
}

fn right_align(arr: Vec<u8>, total_size: usize) -> Vec<u8> {
    let pad_amount = total_size - arr.len();
    let padding = core::iter::repeat_n(b' ', pad_amount).into_iter();
    padding.chain(arr).collect()
}

fn int_to_vec_u8(n: usize) -> Vec<u8> {
    let mut v = Vec::new();
    write!(v.as_mut_slice(), "{n}").unwrap();
    v
}

use usbd_serial::embedded_io::Write;
extern crate alloc;
use alloc::vec::Vec;
fn set_dynamic_elements(screen: &mut Screen, config: &Config, sensor_values: &[[u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS]) {
    let dynamic_element_locations = find_dynamic_element_placeholders(screen);
    // Inner Vec<u8> can't be an ArrayVec because it doesn't handle different length strings correctly for some reason
    let mut dynamic_strs = ArrayVec::<Vec<u8>, MAX_DYNAMIC_ELEMENTS>::new();
    match config.curr_state {
        ConfigOutputs(_) => {
            dynamic_strs.push(if config.serial.enabled {b"YES"} else {b"NO "}.into());
            dynamic_strs.push(if config.sd.enabled     {b"YES"} else {b"NO "}.into());
        },
        ConfigSDStatus(_) => {
            dynamic_strs.push(if todo!() {b"YES"} else {b"NO "}.into());   // SD detected
            dynamic_strs.push(if todo!() {b"YES"} else {b"NO "}.into());   // SD writable
            dynamic_strs.push(if todo!() {b"YES"} else {b"NO "}.into());   // SD formatted
            // Free space
            const MAX_FREE_SPACE_CHARS: usize = 5;
            let free_space_megabytes: usize = todo!();
            let free_space_str = int_to_vec_u8(free_space_megabytes);
            dynamic_strs.push( if free_space_str.len() < MAX_FREE_SPACE_CHARS {right_align(free_space_str, MAX_FREE_SPACE_CHARS)} else {b"9999+".into()} ); 
        },
        ConfigSDFilename(_) => {
            dynamic_strs.push(config.sd.filename.into());
            dynamic_strs.push(config.sd.filetype.as_str().into());
        },
        ConfigChannelSelect(_) => {
            for is_enabled in config.enabled_channels {
                dynamic_strs.push(if is_enabled {b"YES"} else {b"NO "}.into());
            }
        },
        ConfigSampleRate(_) => {
            dynamic_strs.push( right_align(int_to_vec_u8(config.samples_per_sec as usize), 3) );
            // Samples per sec * bytes per sample * seconds per hour * number of active channels * kB per B = kB/hr
            let kb_per_hr = config.samples_per_sec as usize * CHARS_PER_READING * 3600 * config.enabled_channels.iter().filter(|&&x| x).count() / 1000 ;
            dynamic_strs.push( right_align(int_to_vec_u8(kb_per_hr), 3) );
        },
        ViewTemperatures(_) | DatalogTemperatures(_) => {
            for (&is_enabled, &sensor_value) in config.enabled_channels.iter().zip(sensor_values) {
                // Truncate the sensor reading to fit on the display and reappend the 'C' on the end, if enabled
                let sensor_str: Vec<u8> = if is_enabled {let mut v = sensor_value[..6].to_vec(); v.push(b'C'); v} else {b"      ".into()};
                dynamic_strs.push(sensor_str);
            }
        },
        _ => (),
    }
    for (loc, str) in dynamic_element_locations.iter().zip(dynamic_strs) {
        screen[loc.row][loc.start_col..loc.start_col+loc.length].clone_from_slice(&str);
    }
}

use crate::state_machine::State::*;
pub fn update_display(config: &Config, sensor_values: &[[u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS], ) {
    let (selected_element_pos, mut display_buffer) = match config.curr_state {
        Mainmenu(sel) =>                (Some(sel as usize), MAINMENU_SCREEN),
        ConfigOutputs(sel) =>           (Some(sel as usize), CONFIG_OUTPUTS_SCREEN),
        ConfigSDStatus(sel) =>          (Some(sel as usize), CONFIG_SD_STATUS_SCREEN),
        ConfigSDFilename(sel) =>        (Some(sel as usize), CONFIG_SD_FILENAME_SCREEN),
        ConfigChannelSelect(sel) =>     (Some(sel as usize), CONFIG_CHANNEL_SELECT_SCREEN),
        ConfigSampleRate(sel) =>        (Some(sel as usize), CONFIG_SAMPLE_RATE_SCREEN),
        ViewTemperatures(_) =>          (None,               VIEW_SCREEN),
        DatalogTemperatures(_) =>       (None,               VIEW_SCREEN),
        DatalogConfirmStop(sel) =>      (Some(sel as usize), CONFIRM_STOP_LOGGING_SCREEN),
        DatalogErrorSDFull(sel) =>      (Some(sel as usize), SD_FULL_SCREEN),
        DatalogSDWriting(_) =>          (None,               SD_WRITING_SCREEN),
        DatalogSDSafeToRemove(sel) =>   (Some(sel as usize), SD_WRITE_COMPLETE_SCREEN),
    }.clone();

    set_selected_elements(&mut display_buffer, selected_element_pos);
    
    set_dynamic_elements(&mut display_buffer, config, sensor_values);

    todo!()
    //send_buffer_to_display(&mut display_buffer, i2c_bus);
}