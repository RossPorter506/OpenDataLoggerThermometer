use embedded_sdmmc::*;
use embedded_sdmmc::filesystem::TimeSource;
use rp_pico::hal::Timer;
use embedded_hal::digital::InputPin;

pub struct SdManager {
    pub card: embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>,
    // pub ts: embedded_sdmmc::filesystem::Timestamp,
    pub extra_pins: crate::pcb_mapping::SdCardExtraPins,
}

impl SdManager {
    pub fn new(spi: crate::SDCardSPIDriver, delay: Timer, extra_pins: crate::pcb_mapping::SdCardExtraPins) -> Self {
        let new_card = embedded_sdmmc::SdCard::new(spi, delay);
        // let ts: embedded_sdmmc::filesystem::Timestamp = TimeSource.get_timestamp();
        Self {
            card: new_card,
            // ts: ts,
            extra_pins: extra_pins
        }
    }

    pub fn is_inserted(&mut self) -> bool {
        self.extra_pins.card_detect.is_low().unwrap()
    }

    pub fn release(self) -> (embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>, crate::SdCardExtraPins) {
        (self.card, self.extra_pins)
    }
}