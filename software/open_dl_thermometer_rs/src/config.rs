use crate::state_machine::State;
use crate::{constants, RelaxedIO, SAMPLE_RATE_TIMER};

use crate::state_machine::{
    ConfigChannelSelectSelectables as ConChanSel, 
    ConfigOutputsSelectables::{self as ConOutSel, *}, 
    ConfigSDFilenameSelectables::{self as ConSDName, *}, 
    ConfigSDStatusSelectables as ConSdStatSel, 
    ConfigSampleRateSelectables::{self as ConRateSel, *}, 
    DatalogConfirmStopSelectables::*, 
    DatalogErrorSDFullSelectables as SDFullSel, 
    DatalogErrorSDUnexpectedRemovalSelectables as DlogSdRemovSel, 
    DatalogSDSafeToRemoveSelectables as DlogSafeRemovSel, 
    DatalogSDWritingSelectables as DlogSdWrtSel, 
    DatalogSDErrorSelectables as DlogSdErrSel, 
    DatalogTemperaturesSelectables as DlogTempSel, 
    MainmenuSelectables::*, UpdateReason, 
    ViewTemperaturesSelectables as ViewTemp
};

/// Contains information about current system configuration
pub struct Config {
    pub status: Status,
    pub curr_state: State,
    /// Which sensor channels are enabled
    pub enabled_channels: [bool; constants::NUM_SENSOR_CHANNELS],
    /// Sample rate
    pub samples_per_sec: u8,
    pub sd: SDConfig,
    pub serial: SerialConfig,
}

impl Config {
    pub fn new() -> Config {
        Self { status: Default::default(), 
            curr_state: Default::default(), 
            enabled_channels: Default::default(), 
            samples_per_sec: crate::DEFAULT_SAMPLE_FREQ_HZ, 
            sd: Default::default(), 
            serial: Default::default() }
    }

    /// Process inputs, update state, and update any outputs that depend on state
    pub fn process_events(&mut self, update_reason: &UpdateReason, sd_ready: bool) {
        self.process_button_inputs(update_reason);
        self.update_state(update_reason, sd_ready);
        self.update_outputs();
    }

    fn process_button_inputs(&mut self, update_reason: &UpdateReason) {
        if *update_reason != UpdateReason::SelectButton { return }
    
        use crate::state_machine::State::*;
        // If we press select while we have a configurable item selected we need to toggle it
        match self.curr_state {
            Mainmenu(View)                          => (),
            Mainmenu(Datalog)                       => (),
            Mainmenu(Configure)                     => (),
    
            ViewTemperatures(ViewTemp::None)        => (), 
            DatalogTemperatures(DlogTempSel::None)  => (),
    
            ConfigOutputs(Serial)                   => self.serial.selected_for_use ^= true,
            ConfigOutputs(SDCard)                   => self.sd.selected_for_use ^= true,
            ConfigOutputs(ConOutSel::Next)          => (),
    
            ConfigSDStatus(ConSdStatSel::Next)      => (),
    
            ConfigSDFilename(ConSDName::Next)       => (), // don't accidentally capture this case in the catch-all `char_num` pattern below.
            ConfigSDFilename(Filetype)              => self.sd.filetype = self.sd.filetype.next(),
            ConfigSDFilename(char_num)              => cycle_ascii_char(&mut self.sd.filename[char_num as usize]), //filename characters 0-9
    
            ConfigChannelSelect(ConChanSel::Next)   => (), // don't accidentally capture this case in the catch-all `ch_num` pattern below.
            ConfigChannelSelect(ch_num)             => self.enabled_channels[ch_num as usize] ^= true, // selected channels 1-8
    
            ConfigSampleRate(SampleRate)            => {cycle_sample_rate(&mut self.samples_per_sec); crate::WRAPS_PER_SAMPLE.set(8/self.samples_per_sec);},
            ConfigSampleRate(ConRateSel::Next)      => (),
    
            DatalogConfirmStop(ConfirmStop)         => (),
            DatalogConfirmStop(CancelStop)          => (),
    
            DatalogErrorSDFull(SDFullSel::ContinueWithoutSD)                => self.sd.selected_for_use = false,
            DatalogErrorSDFull(SDFullSel::StopDatalogging)                  => (),
            
            DatalogSDWriting(DlogSdWrtSel::None)                            => (),
    
            DatalogSDSafeToRemove(DlogSafeRemovSel::Next)                   => (),
    
            DatalogSDUnexpectedRemoval(DlogSdRemovSel::ContinueWithoutSD)   => self.sd.selected_for_use = false,
            DatalogSDUnexpectedRemoval(DlogSdRemovSel::StopDatalogging)     => (),
            
            DatalogSDError(DlogSdErrSel::ContinueWithoutSD)                 => (),
            DatalogSDError(DlogSdErrSel::StopDatalogging)                   => (),
        };
    }

    fn update_state(&mut self, update_reason: &UpdateReason, sd_ready: bool) {
        use crate::state_machine::State::*;
        use crate::state_machine::UpdateReason::*;
        use Default as d;
        let currently_using_sd_card = self.sd.selected_for_use && self.status == Status::SamplingAndDatalogging;
        // Special transitions
        if *update_reason != SelectButton {
            self.curr_state = match update_reason {
                NextButton                                          => self.curr_state.next_selectable(),
                SDSafeToRemove                                      => DatalogSDSafeToRemove(d::default()),
                SDFull                                              => DatalogErrorSDFull(d::default()),
                SDRemovedUnexpectedly if currently_using_sd_card    => DatalogSDUnexpectedRemoval(d::default()),
                SDError(_)                                          => DatalogSDError(d::default()),
                _                                                   => self.curr_state,
            };
            return;
        }

        // Select button transitions
        let sd_conf_complete = !self.sd.selected_for_use || sd_ready;
        let valid_sd_filename = self.sd.filename != [b' '; FILENAME_MAX_LEN];
        self.curr_state = match self.curr_state {
            Mainmenu(Configure) => ConfigOutputs(d::default()),
            Mainmenu(View)      => ViewTemperatures(d::default()),
            Mainmenu(Datalog)   => DatalogTemperatures(d::default()),

            ConfigOutputs(ConOutSel::Next) if self.sd.selected_for_use      => ConfigSDStatus(d::default()), 
            ConfigOutputs(ConOutSel::Next) if !self.sd.selected_for_use     => ConfigChannelSelect(d::default()),

            ConfigSDStatus(ConSdStatSel::Next) if sd_conf_complete          => ConfigSDFilename(d::default()),
            ConfigSDStatus(ConSdStatSel::Next) if !sd_conf_complete         => ConfigSDStatus(d::default()),

            ConfigSDFilename(ConSDName::Next) if valid_sd_filename          => ConfigChannelSelect(d::default()),
            ConfigSDFilename(ConSDName::Next) if !valid_sd_filename         => ConfigSDFilename(d::default()),

            ConfigChannelSelect(ConChanSel::Next)                           => ConfigSampleRate(d::default()),

            ConfigSampleRate(ConRateSel::Next)                              => Mainmenu(d::default()),

            ViewTemperatures(_)                                             => Mainmenu(d::default()),

            DatalogTemperatures(_)                                          => DatalogConfirmStop(d::default()),

            DatalogConfirmStop(ConfirmStop)                                 => DatalogSDWriting(d::default()),
            DatalogConfirmStop(CancelStop)                                  => DatalogTemperatures(d::default()),

            DatalogErrorSDFull(SDFullSel::ContinueWithoutSD)                => DatalogTemperatures(d::default()),
            DatalogErrorSDFull(SDFullSel::StopDatalogging)                  => DatalogSDWriting(d::default()),

            DatalogSDSafeToRemove(DlogSafeRemovSel::Next)                   => Mainmenu(d::default()),
            
            DatalogSDUnexpectedRemoval(DlogSdRemovSel::ContinueWithoutSD)   => DatalogTemperatures(d::default()),
            DatalogSDUnexpectedRemoval(DlogSdRemovSel::StopDatalogging)     => Mainmenu(d::default()),

            DatalogSDError(DlogSdErrSel::ContinueWithoutSD)                 => DatalogTemperatures(d::default()),
            DatalogSDError(DlogSdErrSel::StopDatalogging)                   => Mainmenu(d::default()),

            _ => self.curr_state,
        }
    }
    
    fn update_outputs(&mut self) {
        use crate::config::Status::*;
        use crate::state_machine::State::*;
        self.status = match self.curr_state {
            Mainmenu(_)                     => Idle,
            ConfigOutputs(_)                => Idle,
            ConfigSDStatus(_)               => Idle,
            ConfigSDFilename(_)             => Idle,
            ConfigChannelSelect(_)          => Idle,
            ConfigSampleRate(_)             => Idle,
            ViewTemperatures(_)             => Sampling,
            DatalogTemperatures(_)          => SamplingAndDatalogging,
            DatalogConfirmStop(_)           => SamplingAndDatalogging,
            DatalogErrorSDFull(_)           => SamplingAndDatalogging,
            DatalogSDUnexpectedRemoval(_)   => SamplingAndDatalogging,
            DatalogSDError(_)               => SamplingAndDatalogging,
            DatalogSDWriting(_)             => Idle,
            DatalogSDSafeToRemove(_)        => Idle,
        };

        critical_section::with(|cs| {
            let Some(ref mut timer) = *SAMPLE_RATE_TIMER.borrow_ref_mut(cs) else { unreachable!() };
            if self.status == Idle { timer.disable(); } 
            else { timer.enable(); }
        });
    }
}

/// Cycle through alphanumeric ASCII values.
/// a -> b -> ... z -> 0 -> ... 9 -> ' ' ->  a
fn cycle_ascii_char(char: &mut u8) {
    *char = match *char {
        b'a'..=b'y' => *char + 1,
        b'z' => b'0',
        b'0'..=b'8' => *char + 1,
        b'9' => b' ',
        _ => b'a',
    };
}
/// Cycle between 1, 2, 4, 8
fn cycle_sample_rate(rate: &mut u8) {
    *rate = match *rate {
        1 => 2,
        2 => 4,
        4 => 8,
        _ => 1,
    }
}

#[derive(Default, PartialEq)]
pub enum Status {
    #[default]
    /// Not sampling sensors
    Idle,
    /// Sampling and displaying result
    Sampling,
    /// Sampling and also logging values
    SamplingAndDatalogging,
}

/// USB serial configuration info
#[derive(Default)]
pub struct SerialConfig {
    pub selected_for_use: bool,
}

pub const FILENAME_MAX_LEN: usize = 10;
/// SD card configuration info
pub struct SDConfig {
    pub selected_for_use: bool,
    pub filename: [u8; FILENAME_MAX_LEN],
    pub filetype: Filetype,
}
impl Default for SDConfig {
    fn default() -> Self {
        Self {
            selected_for_use: false,
            filetype: Filetype::Csv,
            filename: [b' '; FILENAME_MAX_LEN],
        }
    }
}

// TODO: Actually format output differently depending on filetype
/// SD card file type info
pub enum Filetype {
    Csv, // comma separated
    Tsv, // tab separated
    Raw, // null separated
}
impl Filetype {
    pub fn next(&self) -> Filetype {
        use Filetype::*;
        match self {
            Csv => Tsv,
            Tsv => Raw,
            Raw => Csv,
        }
    }
    pub fn as_str(&self) -> &str {
            match self {
            Filetype::Csv => "CSV",
            Filetype::Tsv => "TSV",
            Filetype::Raw => "RAW",
        }
    }
}
