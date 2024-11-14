use embedded_sdmmc::*;
use rp_pico::hal::Timer;

pub struct SdManager {
    pub extra_pins: crate::pcb_mapping::SdCardExtraPins,
    pub vmgr: VolumeManager<embedded_sdmmc::SdCard<crate::SDCardSPIDriver, Timer>, crate::RtcWrapper>,
    pub file: Option<embedded_sdmmc::filesystem::RawFile>,
}

impl SdManager {
    pub fn new(spi: crate::SDCardSPIDriver, delay: Timer, extra_pins: crate::pcb_mapping::SdCardExtraPins, rtc_wrapper: crate::RtcWrapper) -> Self {
        let new_card = embedded_sdmmc::SdCard::new(spi, delay);
        // let ts: embedded_sdmmc::filesystem::Timestamp = TimeSource.get_timestamp();
        Self {
            // ts: ts,
            extra_pins,
            vmgr: VolumeManager::new(new_card, rtc_wrapper),
            file: None,
        }
    }

    pub fn is_card_formatted(&mut self) -> bool {
        for i in 0..=4 {
            let volume = self.vmgr.open_volume(VolumeIdx(i));
            if volume.is_ok() {return true}
        }
        false
    }

    pub fn open_file(&mut self, name: &str) {
        for i in 0..=4 {
            let Ok(volume) = self.vmgr.open_raw_volume(VolumeIdx(i)) else {continue};
            let Ok(root_dir) = self.vmgr.open_root_dir(volume) else {continue};
            let Ok(file) = self.vmgr.open_file_in_dir(root_dir, name, Mode::ReadWriteCreate) else {continue};
            self.file = Some(file);
            return;
        }
    }

    pub fn is_file_open(&mut self, filename: &str) -> bool { 
        todo!()
    }

    pub fn close_file(&mut self) {
        todo!()
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        todo!()
    }

    pub fn is_safe_to_remove(&self) -> bool {
        // Not sure if the library will have support for this, may have to query the SD card directly
        todo!()
    }
}