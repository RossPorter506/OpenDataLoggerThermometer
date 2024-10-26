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


const MAX_DYNAMIC_ELEMENTS: usize = 8;

struct DynamicElement {
    length: usize,
    row: usize,
    start_col: usize,
}

pub const MAINMENU_SCREEN: Screen = [
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
    *b"Free:  @@@M    %NEXT"];

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

/// Sets all placeholder selectables to ' ', except for the one current selected, ordered by standard reading order (top left to bottom right)
fn set_selected_elements(screen: &mut Screen, selected_pos: Option<u8>) {
    if selected_pos.is_none() {return;} // If there are no selectable elements we are done
    let selected_pos = selected_pos.unwrap();

    let mut current_index = 0;
    for row in screen.iter_mut() {
        for col in row.iter_mut() {
            if *col == SELECTABLE_PLACEHOLDER {
                if current_index == selected_pos {
                    *col = b'>';
                }
                else {
                    *col = b' ';
                    current_index += 1;
                }
            }
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
use usbd_serial::embedded_io::Write;
fn set_dynamic_elements(screen: &mut Screen, config: &Config, sensor_values: &[[u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS]) {
    let mut dynamic_element_locations = find_dynamic_element_placeholders(screen);
    type DynamicElementStr<'a> = [u8; 3];
    // I can't figure out a way to have dynamic_strs accept multiple string lengths, so the majority of states use this, but will 
    // manually deal with dynamic elements
    let mut dynamic_strs = ArrayVec::<DynamicElementStr, MAX_DYNAMIC_ELEMENTS>::new();
    match config.curr_state {
        ConfigOutputs(_) => {
            dynamic_strs.push(if config.serial.enabled {*b"YES"} else {*b"NO "});
            dynamic_strs.push(if config.sd.enabled     {*b"YES"} else {*b"NO "});
        },
        ConfigSDStatus(_) => {
            dynamic_strs.push(if todo!() {*b"YES"} else {*b"NO "});   // SD detected
            dynamic_strs.push(if todo!() {*b"YES"} else {*b"NO "});   // SD writable
            dynamic_strs.push(if todo!() {*b"YES"} else {*b"NO "});   // SD formatted
            dynamic_strs.push(todo!());                             // free space
        },
        ConfigSDFilename(_) => {
            // Deal with filename ourselves, since it's not 3-sized.
            let filename_loc = dynamic_element_locations.get(0).unwrap(); 
            screen[filename_loc.row][filename_loc.start_col..filename_loc.start_col+filename_loc.length].clone_from_slice(&config.sd.filename);
            dynamic_element_locations.remove(0);

            // Filetype can be handled the same as the rest
            dynamic_strs.push(config.sd.filetype.as_str());   // Filetype
        },
        ConfigChannelSelect(_) => {
            for is_enabled in config.enabled_channels {
                dynamic_strs.push(if is_enabled {*b"YES"} else {*b"NO "});
            }
        },
        ConfigSampleRate(_) => {
            dynamic_strs.push([b' ', b' ', config.samples_per_sec + b'0']);
            let mut buf = [0u8;3];
            let kb_per_hr = config.samples_per_sec as usize * CHARS_PER_READING /*bytes per sample*/ * 3600 /*seconds per hour*/ * config.enabled_channels.iter().filter(|&&x| x).count() /* num active sensors */ / 1000 /*kB per B*/;
            write!(buf.as_mut_slice(), "{}", kb_per_hr).unwrap();
            dynamic_strs.push(buf);
        },
        ViewTemperatures(_) | DatalogTemperatures(_) => {
            let mut dynamic_strs = ArrayVec::<[u8; 6], MAX_DYNAMIC_ELEMENTS>::new(); 
            let mut buf = [0u8;6];
            for (&is_enabled, sensor_value) in config.enabled_channels.iter().zip(sensor_values) {
                if is_enabled {
                    // Truncate the sensor value a bit to fit the display
                    buf.copy_from_slice(&sensor_value[..6]);
                    buf[6] = b'C';
                }
                else {
                    dynamic_strs.push(*b"      ");
                }
            }
            for (loc, str) in dynamic_element_locations.iter().zip(dynamic_strs) {
                screen[loc.row][loc.start_col..loc.start_col+loc.length].clone_from_slice(&str);
            }
            return;
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
        Mainmenu(sel) => (Some(sel as u8), MAINMENU_SCREEN),
        ConfigOutputs(sel) => (Some(sel as u8), CONFIG_OUTPUTS_SCREEN),
        ConfigSDStatus(sel) => (Some(sel as u8), CONFIG_SD_STATUS_SCREEN),
        ConfigSDFilename(sel) => (Some(sel as u8), CONFIG_SD_FILENAME_SCREEN),
        ConfigChannelSelect(sel) => (Some(sel as u8), CONFIG_CHANNEL_SELECT_SCREEN),
        ConfigSampleRate(sel) => (Some(sel as u8), CONFIG_SAMPLE_RATE_SCREEN),
        ViewTemperatures(_) => (None, VIEW_SCREEN),
        DatalogTemperatures(_) => (None, VIEW_SCREEN),
        DatalogConfirmStop(sel) => (Some(sel as u8), CONFIRM_STOP_LOGGING_SCREEN),
        DatalogErrorSDFull(sel) => (Some(sel as u8), SD_FULL_SCREEN),
        DatalogSDWriting(_) => (None, SD_WRITING_SCREEN),
        DatalogSDSafeToRemove(sel) => (Some(sel as u8), SD_WRITE_COMPLETE_SCREEN),
    }.clone();

    set_selected_elements(&mut display_buffer, selected_element_pos);
    
    set_dynamic_elements(&mut display_buffer, config, sensor_values);

    todo!()
    //send_buffer_to_display(&mut display_buffer, i2c_bus);
}