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