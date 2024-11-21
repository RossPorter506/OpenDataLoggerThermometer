use embedded_hal::{digital::{InputPin, OutputPin}, spi::{ErrorType, SpiBus, SpiDevice}};
use embedded_sdmmc::*;
use rp_pico::{hal::{clocks::PeripheralClock, rtc::RealTimeClock, spi::Enabled, Clock, Spi, Timer}, pac::SPI0};
use rp_pico::hal::fugit::RateExtU32;

use crate::{eprintln, pcb_mapping::{SdCardCs, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardSck}};


pub struct SdManager {
    extra_pins: SdCardExtraPins,
    vmgr: VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>,
    file: Option<embedded_sdmmc::filesystem::RawFile>,
}
const MBR_MAX_PARTITIONS: usize = 4;
type SpiBus0Enabled = Spi<Enabled,SPI0,(SdCardMosi, SdCardMiso, SdCardSck), {crate::BITS_PER_SPI_PACKET}>;
impl SdManager {
    pub fn new(spi_bus: SpiBus0Enabled, cs: SdCardCs, delay: Timer, extra_pins: SdCardExtraPins, rtc: RealTimeClock) -> Self {
        let rtc_wrapper = RtcWrapper{rtc};
        let spi_driver = SDCardSPIDriver{spi_bus, cs};
        let new_card = embedded_sdmmc::SdCard::new(spi_driver, delay);
        // let ts: embedded_sdmmc::filesystem::Timestamp = TimeSource.get_timestamp();
        Self {
            // ts: ts,
            extra_pins,
            vmgr: VolumeManager::new(new_card, rtc_wrapper),
            file: None,
        }
    }

    /// Whether there is a volume on the SD card that is formatted in a way we can understand
    pub fn is_card_formatted(&mut self) -> bool {
        for i in 0..MBR_MAX_PARTITIONS {
            let volume = self.vmgr.open_volume(VolumeIdx(i));
            if volume.is_ok() {return true}
        }
        false
    }

    /// Open a file for writing, creating it if it doesn't exist already. If it does exist the contents are cleared.
    pub fn open_file(&mut self, name: &str) {
        for i in 0..MBR_MAX_PARTITIONS {
            let Ok(volume) = self.vmgr.open_raw_volume(VolumeIdx(i)) else {continue};
            let Ok(root_dir) = self.vmgr.open_root_dir(volume) else {continue};
            let Ok(file) = self.vmgr.open_file_in_dir(root_dir, name, Mode::ReadWriteCreateOrTruncate) else {continue};
            self.file = Some(file);
            return;
        }
        // TODO: Error handling
        eprintln!("Could not open file: '{}'", name);
    }

    /// Whether we have a particular file open at the moment
    pub fn is_file_open(&mut self) -> bool { 
        self.file.is_some()
    }

    /// Closes all open files
    pub fn close_file(&mut self) {
        if self.file.is_some() {
            let my_file = embedded_sdmmc::filesystem::RawFile::to_file(self.file.unwrap_or_else(|| unreachable!()), &mut self.vmgr);
            let error = embedded_sdmmc::filesystem::File::close(my_file);
            if error.is_err() {
                eprintln!("Error closing file!");
            }
        } else {
            eprintln!("No file open to close");
        }
    }

    /// Write bytes to the opened file
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        let Some(file) = self.file else {
            eprintln!("No file open to write to");
            return
        };
        let mut my_file = embedded_sdmmc::filesystem::RawFile::to_file(file, &mut self.vmgr);
        if my_file.is_eof() {
            eprintln!("File is at EOF, cannot write more data");
            return
        }
        let error = embedded_sdmmc::filesystem::File::write(&mut my_file, bytes);
        if error.is_err() {
            eprintln!("Error writing to file!");
        }
    }

    /// Whether the SD card can be safely removed right now
    pub fn is_safe_to_remove(&self) -> bool {
        if self.vmgr.has_open_handles() {
            return false;
        } 
        true
    }

    /// Get the number of bytes free on the SD card
    pub fn get_free_space_bytes(&mut self) -> u64 {
        let Some(file) = self.file else {
            eprintln!("No file open");
            return 0;
        };
        let mut my_file = embedded_sdmmc::filesystem::RawFile::to_file(file, &mut self.vmgr);
        let total_space = my_file.length();
        
        // Create a buffer to read the file contents
        let mut buffer = alloc::vec![0u8; total_space as usize];
        let read_result = my_file.read(&mut buffer);
        
        let used_space = match read_result {
            Ok(bytes_read) => bytes_read as u32,
            Err(_) => {
                eprintln!("Error reading file");
                return 0;
            }
        };
        
        let result = total_space - used_space;
        result as u64
    }

    /// Initialise the SD card after insertion. Must be done before the SD card can be communicated with
    pub fn initialise_card(&mut self, peripheral_clock: &PeripheralClock) {
        // Ensure SPI bus is clocked at 400kHz for initialisation, then
        // send at least 74 clock pulses (without chip select) to wake up card,
        // then reconfigure SPI back to 25MHz
        self.vmgr.device().spi(|driver| {
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), 400.kHz());
            let _ = driver.spi_bus.write(&[0;10]); 
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), 25.MHz())
        });
    }

    /// Whether the SD card is physically present in the SD card slot
    pub fn is_card_inserted(&mut self) -> bool {
        let Ok(detected) = self.extra_pins.card_detect.is_low();
        detected
    }

    /// Whether the SD card is writable due to the external write protect latch
    pub fn is_card_writable(&mut self) -> bool {
        let Ok(writable) = self.extra_pins.write_protect.is_low();
        writable
    }

    /// Prepare the card for safe removal. The card is safe to remove after this function finishes.
    pub fn prepare_for_removal(&mut self) {
        todo!()
    }

    /// Reset the state of the sd_manager and all subcomponents after an SD card is unexpectedly removed. 
    /// 
    /// This includes dealing with files that should have been closed, etc.
    pub fn reset_after_unexpected_removal(&mut self) {
        // The volume manager will likely be unhappy about files remaining open, but probably can't close them either. Figure out what to do.
        todo!()
    }

    /// Whether we are ready to write data to the SD card 
    pub fn ready_to_write(&mut self) -> bool {
        self.is_card_inserted() && self.is_card_writable() && self.is_file_open()
    }

    pub fn get_card_info(&mut self) -> SdCardInfo {
        let is_inserted = self.is_card_inserted();
        if !is_inserted {
            return SdCardInfo::default();
        }
        SdCardInfo {is_inserted, is_writable: Some(self.is_card_writable()), 
            is_formatted: Some(self.is_card_formatted()), 
            free_space_bytes: Some(self.get_free_space_bytes())
        }
    }
}

#[derive(Default)]
pub struct SdCardInfo {
    pub is_inserted: bool,
    pub is_writable: Option<bool>,
    pub is_formatted: Option<bool>,
    pub free_space_bytes: Option<u64>,
}


/// Wrapper layer that implements embedded_hal SpiDevice for the rp2040-hal SPI bus (confusingly also called SpiDevice)
struct SDCardSPIDriver {
    spi_bus: Spi<Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), { crate::BITS_PER_SPI_PACKET }>,
    cs: SdCardCs,
}
impl ErrorType for SDCardSPIDriver{
    type Error = embedded_hal::spi::ErrorKind; // For now.
}
impl SpiDevice for SDCardSPIDriver {
    fn transaction(&mut self, operations: &mut [embedded_hal::spi::Operation<'_, u8>]) -> Result<(), Self::Error> {
        let _ = self.cs.set_low();
        use embedded_hal::spi::Operation::*;
        for op in operations {
            let _ = match op {
                Read(buf) =>                self.spi_bus.read(buf),
                Write(buf) =>               self.spi_bus.write(buf),
                Transfer(rd_buf, wr_buf) => self.spi_bus.transfer(rd_buf, wr_buf),
                TransferInPlace(buf) =>     self.spi_bus.transfer_in_place(buf),
                DelayNs(_) =>               return Err(embedded_hal::spi::ErrorKind::Other), // embedded_sdmmc uses a separate delay object
            };
        }
        let _ = self.spi_bus.flush();
        let _ = self.cs.set_high();
        Ok(())
    }
}

struct RtcWrapper {
    rtc: rp_pico::hal::rtc::RealTimeClock,
}
impl TimeSource for RtcWrapper {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        let timestamp = self.rtc.now().unwrap();
        embedded_sdmmc::Timestamp {
            year_since_1970: timestamp.year as u8,
            zero_indexed_month: timestamp.month,
            zero_indexed_day: timestamp.day,
            hours: timestamp.hour,
            minutes: timestamp.minute,
            seconds: timestamp.second,
        }
    }
}