use embedded_hal::spi::Spi;
use embedded_hal::blocking::delay::DelayMs;
use embedded_sdmmc::{Controller, SdMmcSpi, VolumeIdx};
mod pcb_v1_mapping;
use pcb_v1_mapping::SdCardPins;

// let sd_card = SdMmcSpi::new(
//     spi,
//     sd_card_pins,
//     &mut delay,
//     &mut delay,
//     VolumeIdx(0),
// ).unwrap();
