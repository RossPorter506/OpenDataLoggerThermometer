use embedded_sdmmc::*;
use rp_pico::hal::Timer;

pub struct SdManager {
    pub extra_pins: crate::pcb_mapping::SdCardExtraPins,
    pub vmgr: VolumeManager<embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>, crate::RtcWrapper>,
}

impl SdManager {
    pub fn new(spi: crate::SDCardSPIDriver, delay: Timer, extra_pins: crate::pcb_mapping::SdCardExtraPins, rtc_wrapper: crate::RtcWrapper) -> Self {
        let new_card = embedded_sdmmc::SdCard::new(spi, delay);
        // let ts: embedded_sdmmc::filesystem::Timestamp = TimeSource.get_timestamp();
        Self {
            // ts: ts,
            extra_pins,
            vmgr: VolumeManager::new(new_card, rtc_wrapper),
        }
    }

    pub fn is_card_formatted(vol_mgr: &mut VolumeManager<embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>, crate::RtcWrapper>,) -> bool {
        for i in 0..=4 {
            let volume = vol_mgr.open_volume(VolumeIdx(i));
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