#![allow(dead_code)]

/// Encode both the state and what things on the screen can be selected into one item
/// This ensures that there can't be any mismatches between states and selected items
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum State {
    /// Main menu
    Mainmenu(MainmenuSelectables),
    /// Configure which datalogging outputs are used
    ConfigOutputs(ConfigOutputsSelectables),
    /// Check SD card ready before configuration
    ConfigSDStatus(ConfigSDStatusSelectables),
    /// Set SD card filename
    ConfigSDFilename(ConfigSDFilenameSelectables),
    /// Configure which channels to display/datalog
    ConfigChannelSelect(ConfigChannelSelectSelectables),
    /// Configure sensor sample rate
    ConfigSampleRate(ConfigSampleRateSelectables),
    /// View sensors without logging data
    ViewTemperatures(ViewTemperaturesSelectables),
    /// View sensors and log data
    DatalogTemperatures(DatalogTemperaturesSelectables),
    /// Confirmation dialog to end datalogging
    DatalogConfirmStop(DatalogConfirmStopSelectables),
    /// SD Card Full
    DatalogErrorSDFull(DatalogErrorSDFullSelectables),
    /// Wait for SD card writes to complete
    DatalogSDWriting(DatalogSDWritingSelectables),
    /// SD Card writes complete, safe to remove now
    DatalogSDSafeToRemove(DatalogSDSafeToRemoveSelectables),
    /// SD Card removed unexpectedly while in use
    DatalogSDUnexpectedRemoval(DatalogErrorSDUnexpectedRemovalSelectables),
    // Generic SD card error
    DatalogSDError(DatalogSDErrorSelectables),
}
impl State {
    // Not proud of this one. Still, beats the alternatives...
    /// Iterate to the next selectable element in this state
    pub fn next_selectable(&mut self) -> Self {
        match self {
            Mainmenu(sel) => Mainmenu(sel.next()),
            ConfigOutputs(sel) => ConfigOutputs(sel.next()),
            ConfigSDStatus(sel) => ConfigSDStatus(sel.next()),
            ConfigSDFilename(sel) => ConfigSDFilename(sel.next()),
            ConfigChannelSelect(sel) => ConfigChannelSelect(sel.next()),
            ConfigSampleRate(sel) => ConfigSampleRate(sel.next()),
            ViewTemperatures(sel) => ViewTemperatures(sel.next()),
            DatalogTemperatures(sel) => DatalogTemperatures(sel.next()),
            DatalogConfirmStop(sel) => DatalogConfirmStop(sel.next()),
            DatalogErrorSDFull(sel) => DatalogErrorSDFull(sel.next()),
            DatalogSDError(sel) => DatalogSDError(sel.next()), 
            DatalogSDWriting(sel) => DatalogSDWriting(sel.next()),
            DatalogSDSafeToRemove(sel) => DatalogSDSafeToRemove(sel.next()),
            DatalogSDUnexpectedRemoval(sel) => DatalogSDUnexpectedRemoval(sel.next()),
        }
    }
}
impl Default for State {
    fn default() -> Self {
        Self::Mainmenu(Default::default())
    }
}
use embedded_sdmmc::SdCardError;
use State::*;

use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
// Macro to generate selectables in a state
macro_rules! create_selectables {
    ($name:ident, [$first_variant:ident, $($variant:ident),*]) => {
        #[derive(Default, FromPrimitive, Copy, Clone, PartialEq, Debug)]
        pub enum $name {
            #[default]
            $first_variant = 0,
            $($variant),*
        }
        impl $name {
            const fn len() -> usize {
                core::mem::variant_count::<Self>()
            }
            pub fn next(self) -> Self{
                FromPrimitive::from_usize((self as usize + 1) % Self::len()).unwrap_or_else(|| unreachable!())
            }
        }
    };
    ($name:ident, []) => {
        #[derive(Default, Copy, Clone, PartialEq, Debug)]
        pub enum $name {
            #[default]
            None
        }
        impl $name {
            pub fn next(self) -> Self {
                Self::None
            }
        }
    };
}

// What can be selected on each screen
create_selectables!(MainmenuSelectables, [Configure, View, Datalog]);

create_selectables!(ConfigOutputsSelectables, [SDCard, Serial, Next]);
create_selectables!(ConfigSDStatusSelectables, [Next,]);
create_selectables!(ConfigSDFilenameSelectables, [
        FilenameChar0, FilenameChar1, FilenameChar2, FilenameChar3,
        FilenameChar4, FilenameChar5, FilenameChar6, FilenameChar7,
        FilenameChar8, FilenameChar9, Filetype, Next]);

create_selectables!(
    ConfigChannelSelectSelectables,
    [Channel1, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7, Channel8, Next]
);
create_selectables!(ConfigSampleRateSelectables, [SampleRate, Next]);

create_selectables!(ViewTemperaturesSelectables, []);

create_selectables!(DatalogTemperaturesSelectables, []);
create_selectables!(DatalogConfirmStopSelectables, [CancelStop, ConfirmStop]);
create_selectables!(
    DatalogErrorSDFullSelectables,
    [ContinueWithoutSD, StopDatalogging]
);
create_selectables!(
    DatalogErrorSDUnexpectedRemovalSelectables,
    [ContinueWithoutSD, StopDatalogging]
);
create_selectables!(DatalogSDWritingSelectables, []);
create_selectables!(DatalogSDSafeToRemoveSelectables, [Next,]);
create_selectables!(DatalogSDErrorSelectables, [ContinueWithoutSD, StopDatalogging]);

// Shorthand
use core::default::Default as d;
use ConfigChannelSelectSelectables as ConChanSel;
use ConfigOutputsSelectables as ConOut;
use ConfigSDFilenameSelectables as ConSDName;
use ConfigSDStatusSelectables as ConSDStat;
use ConfigSampleRateSelectables as ConSampRate;
use DatalogConfirmStopSelectables::*;
use DatalogErrorSDFullSelectables as SDFullSel;
use DatalogErrorSDUnexpectedRemovalSelectables::*;
use DatalogSDSafeToRemoveSelectables as DlSDSafe;
use DatalogSDErrorSelectables as DlSDErrSel;
use MainmenuSelectables::*;

use crate::{config::Status::SamplingAndDatalogging, sd_card::SdManager};
/// Determine the next state based on the current state and config
pub fn next_state(config: &mut crate::config::Config, sd_manager: &mut SdManager, update_reason: &UpdateReason) {
    // Special transitions
    if *update_reason != SelectButton {
        config.curr_state = match update_reason {
            SDSafeToRemove => DatalogSDSafeToRemove(d::default()),
            SDFull => DatalogErrorSDFull(d::default()),
            SDRemovedUnexpectedly => 
                if config.status == SamplingAndDatalogging && config.sd.selected_for_use {DatalogSDUnexpectedRemoval(d::default())}
                else {config.curr_state},
            NextButton => config.curr_state.next_selectable(),
            SDError(_) => DatalogSDError(d::default()),
            _ => config.curr_state,
        };
        return;
    }

    // Select button transitions
    config.curr_state = match &config.curr_state {
        Mainmenu(Configure) => ConfigOutputs(d::default()),
        Mainmenu(View)      => ViewTemperatures(d::default()),
        Mainmenu(Datalog)   => DatalogTemperatures(d::default()),

        ConfigOutputs(ConOut::Next) => {
            if config.sd.selected_for_use { ConfigSDStatus(d::default()) } 
            else { ConfigChannelSelect(d::default()) }
        }

        ConfigSDStatus(ConSDStat::Next)         => {
            let sd_configuration_complete = !config.sd.selected_for_use || ( sd_manager.is_card_inserted() && sd_manager.is_card_writable() && sd_manager.is_card_formatted() );
            if sd_configuration_complete { ConfigSDFilename(d::default()) } 
            else { ConfigSDStatus(d::default()) }},

        ConfigSDFilename(ConSDName::Next)                   => ConfigChannelSelect(d::default()),

        ConfigChannelSelect(ConChanSel::Next)               => ConfigSampleRate(d::default()),

        ConfigSampleRate(ConSampRate::Next)                 => Mainmenu(d::default()),

        ViewTemperatures(_)                                 => Mainmenu(d::default()),

        DatalogTemperatures(_)                              => DatalogConfirmStop(d::default()),

        DatalogConfirmStop(ConfirmStop)                     => DatalogSDWriting(d::default()),
        DatalogConfirmStop(CancelStop)                      => DatalogTemperatures(d::default()),

        DatalogErrorSDFull(SDFullSel::ContinueWithoutSD)    => DatalogTemperatures(d::default()),
        DatalogErrorSDFull(SDFullSel::StopDatalogging)      => DatalogSDWriting(d::default()),

        DatalogSDSafeToRemove(DlSDSafe::Next)               => Mainmenu(d::default()),
        
        DatalogSDUnexpectedRemoval(ContinueWithoutSD)       => DatalogTemperatures(d::default()),
        DatalogSDUnexpectedRemoval(StopDatalogging)         => Mainmenu(d::default()),

        DatalogSDError(DlSDErrSel::ContinueWithoutSD)       => DatalogTemperatures(d::default()),
        DatalogSDError(DlSDErrSel::StopDatalogging)         => Mainmenu(d::default()),

        _ => config.curr_state,
    }
}

/// Compute Moore-type state outputs that depend only on the current state
pub fn state_outputs(config: &mut crate::config::Config) {
    use crate::config::Status::*;
    config.status = match config.curr_state {
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
    }
}

/// Information about why we are updaing
#[derive(Clone)]
pub enum UpdateReason {
    NextButton,
    SelectButton,
    NewSensorValues,
    SDSafeToRemove,
    SDFull,
    SDStateChange,
    SDRemovedUnexpectedly,
    SDError(embedded_sdmmc::Error<SdCardError>),
}
use UpdateReason::*;

// embedded_sdmmc::Error and embedded_sdmmc::sdcard::SdCardError don't implement PartialEq...
impl PartialEq for UpdateReason {
    fn eq(&self, other: &Self) -> bool {
        use embedded_sdmmc::Error::DeviceError;
        // If the error has internal values match those
        match (self, other) {
            (SDError(DeviceError(e0)), SDError(DeviceError(e1)))    => core::mem::discriminant(e0) == core::mem::discriminant(e1),
            (SDError(error0), SDError(error1))                      => core::mem::discriminant(error0) == core::mem::discriminant(error1),
            _                                                       => core::mem::discriminant(self) == core::mem::discriminant(other),
        }
    }
}