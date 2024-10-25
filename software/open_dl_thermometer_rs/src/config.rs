use crate::state_machine::State;
use crate::constants;

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
    pub uart: UARTConfig,
}
impl Config {
    pub fn new() -> Config {
        Config::default()
    }
}
#[derive(Default, PartialEq)]
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

/// UART configuration info
pub struct UARTConfig{
    pub enabled: bool,
    pub baud_rate: u32,
    pub parity: bool,
    pub double_stop_bit: bool,
}
impl Default for UARTConfig {
    fn default() -> Self {
        Self { enabled: false, baud_rate: 9600, parity: false, double_stop_bit: false }
    }
}

const FILENAME_MAX_LEN: usize = 10;
/// SD card configuration info
pub struct SDConfig{
    pub enabled: bool,
    pub filename: [u8; FILENAME_MAX_LEN],
    pub filetype: Filetype,
}
impl Default for SDConfig {
    fn default() -> Self {
        Self { enabled: false, filetype: Filetype::CSV, filename: [0; FILENAME_MAX_LEN]}
    }
}
/// SD card file type info
pub enum Filetype{
    CSV, // comma separated
    TSV, // tab separated
    RAW, // null separated
}
impl Filetype {
    pub fn next(&self) -> Filetype {
        use Filetype::*;
        match self {
            CSV => TSV,
            TSV => RAW,
            RAW => CSV,
        }
    }
}