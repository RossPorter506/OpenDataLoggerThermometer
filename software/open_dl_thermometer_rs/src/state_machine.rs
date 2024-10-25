#![allow(dead_code)]

/// Encode both the state and what things on the screen can be selected into one item
/// This ensures that there can't be any mismatches between states and selected items
#[derive(Copy, Clone)]
pub enum State {
    /// Main menu
    Mainmenu(MainmenuSelectables),
    /// Configure which datalogging outputs are used 
    ConfigOutputs(ConfigOutputsSelectables),
    /// Configure UART options
    ConfigUART(ConfigUARTSelectables),
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
}
impl State {
    // Not proud of this one. Still, beats the alternatives...
    /// Iterate to the next selectable element in this state
    pub fn next_selectable(&mut self) -> Self{
        match self {
            Mainmenu(sel) => Mainmenu(sel.next()),
            ConfigOutputs(sel) => ConfigOutputs(sel.next()),
            ConfigUART(sel) => ConfigUART(sel.next()),
            ConfigSDStatus(sel) => ConfigSDStatus(sel.next()),
            ConfigSDFilename(sel) => ConfigSDFilename(sel.next()),
            ConfigChannelSelect(sel) => ConfigChannelSelect(sel.next()),
            ConfigSampleRate(sel) => ConfigSampleRate(sel.next()),
            ViewTemperatures(sel) => ViewTemperatures(sel.next()),
            DatalogTemperatures(sel) => DatalogTemperatures(sel.next()),
            DatalogConfirmStop(sel) => DatalogConfirmStop(sel.next()),
            DatalogErrorSDFull(sel) => DatalogErrorSDFull(sel.next()),
            DatalogSDWriting(sel) => DatalogSDWriting(sel.next()),
            DatalogSDSafeToRemove(sel) => DatalogSDSafeToRemove(sel.next()),
        }
    }
} 
impl Default for State {
    fn default() -> Self {
        Self::Mainmenu(Default::default())
    }
}
use State::*;

use num_traits::FromPrimitive;
use num_derive::FromPrimitive;
// Macro to generate selectables in a state 
macro_rules! create_selectables {
    ($name:ident, [$first_variant:ident, $($variant:ident),*]) => {
        #[derive(Default, FromPrimitive, Copy, Clone)]
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
                FromPrimitive::from_usize((self as usize + 1) % Self::len()).unwrap()
            }
        }
    };
    ($name:ident, []) => {
        #[derive(Default, Copy, Clone)]
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

create_selectables!(ConfigOutputsSelectables, [UART, SDCard, Next]);
create_selectables!(ConfigUARTSelectables, [BaudRate, Parity, StopBits, Next]);
create_selectables!(ConfigSDStatusSelectables, [Next,]);
create_selectables!(ConfigSDFilenameSelectables, [FilenameChar0, FilenameChar1, FilenameChar2, FilenameChar3, FilenameChar4, FilenameChar5, FilenameChar6, FilenameChar7, FilenameChar8, FilenameChar9, Filetype, Next]);
create_selectables!(ConfigChannelSelectSelectables, [Channel1, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7, Channel8, Next]);
create_selectables!(ConfigSampleRateSelectables, [SampleRate, Next]);

create_selectables!(ViewTemperaturesSelectables, []);

create_selectables!(DatalogTemperaturesSelectables, []);
create_selectables!(DatalogConfirmStopSelectables, [ConfirmStop, CancelStop]);
create_selectables!(DatalogErrorSDFullSelectables, [ContinueWithoutSDCard, StopDatalogging]);
create_selectables!(DatalogSDWritingSelectables, []);
create_selectables!(DatalogSDSafeToRemoveSelectables, [Next,]);

// Shorthand
use MainmenuSelectables::*;
use ConfigOutputsSelectables as ConOut;
use ConfigUARTSelectables as ConUART;
use ConfigSDStatusSelectables as ConSDStat;
use ConfigSDFilenameSelectables as ConSDName;
use ConfigChannelSelectSelectables as ConChanSel;
use ConfigSampleRateSelectables as ConSampRate;
use ViewTemperaturesSelectables as ViewTemp;
use DatalogTemperaturesSelectables as DlogTemp;
use DatalogConfirmStopSelectables::*;
use DatalogErrorSDFullSelectables::*;
use DatalogSDWritingSelectables as DlSDWriting;
use DatalogSDSafeToRemoveSelectables as DlSDSafe;
use core::default::Default as d;

/// Determine the next state based on the current state and config 
pub fn next_state(config: &mut crate::config::Config, update_reason: &UpdateReason) {
    if *update_reason != SelectButton {
        match update_reason {
            SDSafeToRemove => config.curr_state = DatalogSDSafeToRemove(d::default()),
            SDFull => config.curr_state = DatalogErrorSDFull(d::default()),
            _ => (),
        };
        return;
    }
  
    config.curr_state = match &config.curr_state {
        Mainmenu(Configure) => ConfigOutputs(d::default()),
        Mainmenu(View)      => ViewTemperatures(d::default()),
        Mainmenu(Datalog)   => DatalogTemperatures(d::default()),

        ConfigOutputs(ConOut::Next) => if config.uart.enabled {ConfigUART(d::default()) } else if config.sd.enabled { ConfigSDStatus(d::default()) } else { ConfigChannelSelect(d::default()) },

        ConfigUART(ConUART::Next) => if config.sd.enabled { ConfigSDStatus(d::default()) } else { ConfigChannelSelect(d::default()) },
        
        ConfigSDStatus(ConSDStat::Next)           => ConfigSDFilename(d::default()),

        ConfigSDFilename(ConSDName::Next)         => ConfigChannelSelect(d::default()),

        ConfigChannelSelect(ConChanSel::Next)     => ConfigSampleRate(d::default()),

        ConfigSampleRate(ConSampRate::Next)       => Mainmenu(d::default()),

        ViewTemperatures(ViewTemp::None)          => Mainmenu(d::default()),

        DatalogTemperatures(DlogTemp::None)       => Mainmenu(d::default()),

        DatalogConfirmStop(ConfirmStop)           => DatalogSDWriting(d::default()),
        DatalogConfirmStop(CancelStop)            => DatalogTemperatures(d::default()),

        DatalogErrorSDFull(ContinueWithoutSDCard) => DatalogTemperatures(d::default()),
        DatalogErrorSDFull(StopDatalogging)       => DatalogSDWriting(d::default()),

        DatalogSDWriting(DlSDWriting::None)       => DatalogSDSafeToRemove(d::default()),

        DatalogSDSafeToRemove(DlSDSafe::Next)     => Mainmenu(d::default()),

        _ => config.curr_state,
    }
}

/// Information about why we are updaing
#[derive(Copy, Clone, PartialEq)]
pub enum UpdateReason {
    NextButton,
    SelectButton,
    NewSensorValues,
    SDSafeToRemove,
    SDFull,
} use UpdateReason::*;