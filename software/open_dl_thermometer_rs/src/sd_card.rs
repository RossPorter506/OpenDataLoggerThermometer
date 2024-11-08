use embedded_hal::digital::InputPin;
use embedded_hal::{delay::DelayNs, spi};
use embedded_hal::spi::SpiDevice;
use embedded_sdmmc::*;
use rp_pico::hal::{Timer};

pub struct SdCard {
    pub card: embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>,
}

impl SdCard {
    pub fn new(spi: crate::SDCardSPIDriver, delay: Timer) -> Self {
        let new_card = embedded_sdmmc::SdCard::new(spi, delay);
        Self {
            card: new_card,
        }
    }

    pub fn get_card(&mut self) -> &mut embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer> {
        &mut self.card
    }
}