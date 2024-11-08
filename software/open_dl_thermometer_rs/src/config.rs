use crate::constants;
use crate::state_machine::State;

/// Contains information about current system configuration
#[derive(Default)]
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
        Config::default()
    }
}
#[derive(Default, PartialEq)]
// TODO: Maybe define this based on the system state? Seems like we're duplicating logic here
/// Whether we are currently polling temperatures or not
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
    pub enabled: bool,
}

const FILENAME_MAX_LEN: usize = 10;
/// SD card configuration info
pub struct SDConfig {
    pub enabled: bool,
    pub filename: [u8; FILENAME_MAX_LEN],
    pub filetype: Filetype,
    pub card_detected: bool,
    pub card_writable: bool,
    pub card_formatted: bool,
    pub free_space_bytes: u32,
    pub safe_to_remove: bool,
}
impl Default for SDConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            card_detected: false,
            card_writable: false,
            card_formatted: false,
            safe_to_remove: false,
            free_space_bytes: 0,
            filetype: Filetype::Csv,
            filename: [b'\0'; FILENAME_MAX_LEN],
        }
    }
}
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
    pub fn as_str(&self) -> [u8;3] {
        match self {
            Filetype::Csv => *b"CSV",
            Filetype::Tsv => *b"TSV",
            Filetype::Raw => *b"RAW",
        }
    }
}
