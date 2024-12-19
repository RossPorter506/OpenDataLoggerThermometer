use crate::constants;
use crate::state_machine::State;

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
