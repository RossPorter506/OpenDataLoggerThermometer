use embedded_sdmmc::*;
use embedded_sdmmc::filesystem::TimeSource;
use rp_pico::hal::Timer;
use embedded_hal::digital::InputPin;
use rp_pico::pac::RTC;
use rp_pico::hal::rtc::RealTimeClock;

use crate::RtcWrapper;

pub struct SdManager {
    pub card: embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>,
    // pub ts: embedded_sdmmc::filesystem::Timestamp,
    pub extra_pins: crate::pcb_mapping::SdCardExtraPins,
    pub timestamp: Timestamp,
}

// impl BlockDevice for SdCard {
//     type Error: Debug;

//     // Required methods
//     fn read(
//         &self,
//         blocks: &mut [Block],
//         start_block_idx: BlockIdx,
//         reason: &str,
//     ) -> Result<(), Self::Error> {
//         self.read(blocks, start_block_idx)
//     }
//     fn write(
//         &self,
//         blocks: &[Block],
//         start_block_idx: BlockIdx,
//     ) -> Result<(), Self::Error> {
//         self.write(blocks, start_block_idx)
//     }
//     fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
//         self.num_blocks()
//     }
// }

pub struct Timestamp {
    pub year_since_1970: u8,
    pub zero_indexed_month: u8,
    pub zero_indexed_day: u8,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
}

impl SdManager {
    pub fn init() {
    }
    pub fn new(spi: crate::SDCardSPIDriver, delay: Timer, extra_pins: crate::pcb_mapping::SdCardExtraPins) -> Self {
        let new_card = embedded_sdmmc::SdCard::new(spi, delay);
        // let ts: embedded_sdmmc::filesystem::Timestamp = TimeSource.get_timestamp();
        Self {
            card: new_card,
            // ts: ts,
            extra_pins: extra_pins,

            timestamp: Timestamp {
                year_since_1970: 0,
                zero_indexed_month: 0,
                zero_indexed_day: 0,
                hours: 0,
                minutes: 0,
                seconds: 0,
            },
        }
    }

    pub fn is_card_formated(sd_card: embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>, ts: crate::RtcWrapper) -> bool {
        
        let mut volume_manager = VolumeManager::new(sd_card, ts);
        for i in 0..=4 {
            let volume = volume_manager.open_volume(VolumeIdx(i));
            match volume {
                Ok(_) => {
                    return true;
                }
                Err(_) => {
                    continue;
                }
            }
        }
        false
    }
}