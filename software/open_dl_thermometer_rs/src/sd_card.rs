use alloc::vec::Vec;
use embedded_hal::{digital::{InputPin, OutputPin}, spi::{ErrorType, SpiBus, SpiDevice}};
use embedded_sdmmc::*;
use rp_pico::{hal::{clocks::PeripheralClock, rtc::RealTimeClock, spi::Enabled, Clock, Spi, Timer}, pac::SPI0};
use rp_pico::hal::fugit::RateExtU32;

use crate::{eprintln, pcb_mapping::{SdCardCs, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardSck}};


pub struct SdManager {
    extra_pins: SdCardExtraPins,
    vmgr: VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>,
    file: Option<embedded_sdmmc::filesystem::RawFile>,
    root_dir: Option<RawDirectory>,
    volume: Option<RawVolume>,
    // What the insertion state of the SD card was in the previous loop. Used to compare for insertions/removals.
    card_previously_inserted: bool,
}
const MBR_MAX_PARTITIONS: usize = 4;
type SpiBus0Enabled = Spi<Enabled,SPI0,(SdCardMosi, SdCardMiso, SdCardSck), {crate::BITS_PER_SPI_PACKET}>;
impl SdManager {
    pub fn new(spi_bus: SpiBus0Enabled, cs: SdCardCs, delay: Timer, mut extra_pins: SdCardExtraPins, rtc: RealTimeClock) -> Self {
        let rtc_wrapper = RtcWrapper{rtc};
        let spi_driver = SDCardSPIDriver{spi_bus, cs};
        let new_card = embedded_sdmmc::SdCard::new(spi_driver, delay);

        Self {
            card_previously_inserted: extra_pins.card_detect.is_low().unwrap(),
            extra_pins,
            vmgr: VolumeManager::new(new_card, rtc_wrapper),
            file: None,
            root_dir: None,
            volume: None,
        }
    }

    /// Whether there is a volume on the SD card that is formatted in a way we can understand
    pub fn is_card_formatted(&mut self) -> bool {
        if self.volume.is_some() { return true }
        // Check for any readable volumes
        for i in 0..MBR_MAX_PARTITIONS {
            if let Ok(vol) = self.vmgr.open_volume(VolumeIdx(i)) {
                self.volume = Some(vol.to_raw_volume());
                return true;
            }
        }
        false
    }

    /// Open a file for writing, creating it if it doesn't exist already. If it does exist the contents are cleared.
    pub fn try_open_file(&mut self, name: &str) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        // This should be infallible, as we check for readable volumes during the setup screen.
        let vol = self.volume.ok_or(embedded_sdmmc::Error::FormatError("No readable volumes"))?;
        // These could totally fail though
        let root_dir = self.vmgr.open_root_dir(vol)?;
        let file = self.vmgr.open_file_in_dir(root_dir, name, Mode::ReadWriteCreateOrTruncate)?;
        self.root_dir = Some(root_dir);
        self.file = Some(file);
        Ok(())
    }

    /// Whether we have a particular file open at the moment
    pub fn is_file_open(&mut self) -> bool { 
        self.file.is_some()
    }

    /// Write bytes to the opened file
    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        let Some(file) = self.file else { 
            eprintln!("No file open to write to."); return Err(embedded_sdmmc::Error::NotFound);
        };
        
        file.to_file(&mut self.vmgr).write(bytes)?;
        Ok(())
    }

    /// Whether the SD card can be safely removed right now
    pub fn is_safe_to_remove(&self) -> bool {
        !self.vmgr.has_open_handles()
    }

    /// Get the number of bytes free on the SD card
    pub fn get_free_space_bytes(&mut self) -> u64 {
        let total_space = self.get_partition_size();
        let used_space: u64 = self.get_volume_used_space();

        total_space - used_space
    }

    fn get_partition_size(&mut self) -> u64 {
        // Read MBR (first 512 bytes on disk)
        let mbr: [u8; 512] = todo!();
        /// Volume information begins at address 0x1BE in the MBR. 
        const PARTITION_DATA_OFFSET: usize = 0x1BE;
        /// The MBR data for each partition is 16 bytes.
        const PARTITION_DATA_SIZE: usize = 16;

        let our_partition_index: usize = todo!(); // Which partition we have open - 0, 1, 2, 3
        let our_partition_offset = PARTITION_DATA_OFFSET + PARTITION_DATA_SIZE*our_partition_index; // 0x1BE if index = 0, 0x1CE if index = 1, etc.
        let our_partition_data: [u8; 16] = mbr[our_partition_offset..our_partition_offset+PARTITION_DATA_SIZE].try_into().unwrap();
        // Number of sectors in volume is the last 4 bytes of the volume information
        let our_partition_length_data: [u8; 4] = our_partition_data[12..16].try_into().unwrap();
        // Use u32::from_le_bytes() to convert little endian [u8; 4] to u32
        let our_partition_length_sectors = u32::from_le_bytes(our_partition_length_data);
        let total_space_bytes: u64 = our_partition_length_sectors as u64 * 512; // TODO: Is it always 512 bytes/sector?
        total_space_bytes
    } 

    /// Calculate used space in volume. Walk the filesystem, get the length of each file. Sum them.
    fn get_volume_used_space(&mut self) -> u64 {
        let Some(root_dir) = self.root_dir else { todo!() };
        self.get_folder_size(root_dir)
    }

    /// Recursively get the size of a folder
    fn get_folder_size(&mut self, raw_dir: RawDirectory) -> u64 {
        let mut total: u64 = 0;
        let (mut files, mut folders) = (Vec::new(), Vec::new());
        
        self.vmgr.iterate_dir(raw_dir, |dir_entry: &DirEntry| {
            if dir_entry.attributes.is_directory() {
                folders.push(dir_entry.clone());
            }
            else {
                files.push(dir_entry.clone());
            }
        }).unwrap();
        self.vmgr.close_dir(raw_dir).unwrap();

        for file_info in files {
            total += file_info.size as u64;
        }
        for folder_info in folders {
            let folder = self.vmgr.open_dir(raw_dir, folder_info.name).unwrap(); // TODO: Potentially fallible, we may hit the max number of open folders
            // Recurse
            total += self.get_folder_size(folder);
            self.vmgr.close_dir(folder).unwrap();
        }
        total
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
        // Tell SD card to initialise next time it's used
        self.vmgr.device().mark_card_uninit();
    }

    /// Whether the SD card is physically present in the SD card slot
    pub fn is_card_inserted(&mut self) -> bool {
        let Ok(detected) = self.extra_pins.card_detect.is_low();
        detected
    }

    /// Whether a card has been physically inserted or removed
    pub fn get_physical_card_events(&mut self) -> SdCardEvent {
        let currently_inserted = self.is_card_inserted();
        let out = match (self.card_previously_inserted, currently_inserted) {
            (true, false)   => SdCardEvent::WasJustRemoved,
            (false, true)   => SdCardEvent::WasJustInserted,
            _               => SdCardEvent::NoChange,
        };
        self.card_previously_inserted = currently_inserted;
        out
    }

    /// Whether the SD card is writable due to the external write protect latch
    pub fn is_card_writable(&mut self) -> bool {
        let Ok(writable) = self.extra_pins.write_protect.is_low();
        writable
    }

    /// Prepare the card for safe removal. The card is safe to remove after this function finishes.
    pub fn prepare_for_removal(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>>{
        self.close_file()?;
        self.close_root_dir()?;
        self.close_volume()
    }

    fn close_file(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>>{
        let Some(file) = self.file.take() else { return Ok(()) };
        self.vmgr.close_file(file)
    }

    fn close_root_dir(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        let Some(root_dir) = self.root_dir.take() else { return Ok(()) };
        self.vmgr.close_dir(root_dir)
    }

    fn close_volume(&mut self) -> Result<(), embedded_sdmmc::Error<SdCardError>>{
        let Some(raw_vol) = self.volume.take() else { return Ok(()) };
        self.vmgr.close_volume(raw_vol)
    }

    /// Reset the state of the sd_manager and all subcomponents after an SD card is unexpectedly removed. 
    /// 
    /// This includes dealing with files that should have been closed, etc.
    pub fn reset_after_unexpected_removal(self) -> SdManager {
        // Can't close open files or volumes as this requires communication with the SD card. Rebuild internals.
        let (sd_card, rtc_wrapper) = self.vmgr.free();
        sd_card.mark_card_uninit();
        SdManager{ extra_pins: self.extra_pins, vmgr: VolumeManager::new(sd_card, rtc_wrapper), file: None, root_dir: None, volume: None, card_previously_inserted: self.card_previously_inserted }
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

pub enum SdCardEvent {
    WasJustInserted,
    WasJustRemoved,
    NoChange,
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