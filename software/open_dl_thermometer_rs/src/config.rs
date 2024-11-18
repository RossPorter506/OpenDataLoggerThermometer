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
    pub card_detected: bool,
    pub card_writable: bool,
    pub card_formatted: bool,
    pub free_space_bytes: u64,
    pub safe_to_remove: bool,
}
impl Default for SDConfig {
    fn default() -> Self {
        Self {
            selected_for_use: false,
            card_detected: false,
            card_writable: false,
            card_formatted: false,
            safe_to_remove: false,
            free_space_bytes: 0,
            filetype: Filetype::Csv,
            filename: [b' '; FILENAME_MAX_LEN],
        }
    }
}
impl SDConfig {
    /// Whether SD card configuration is complete: Either not selected for use, or selected and ready.
    pub fn configuration_complete(&self) -> bool {
        !self.selected_for_use || (self.card_detected && self.card_writable && self.card_formatted)
    }
    /// Reset card information. Note: This only resets information related to the inserted SD card, not system settings like whether the SD card is selected for use, the chosen filename, etc
    pub fn reset_card_information(&mut self) {
        self.card_detected = false;
        self.card_writable = false;
        self.card_formatted = false;
        self.free_space_bytes = 0;
        self.safe_to_remove = false;
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
