use alloc::vec::Vec;

use embedded_hal::{digital::{InputPin, OutputPin}, spi::{ErrorType, SpiBus, SpiDevice}};
use embedded_sdmmc::*;
use rp_pico::{hal::{clocks::PeripheralClock, rtc::RealTimeClock, spi::Enabled, Clock, Spi, Timer}, pac::SPI0};
use rp_pico::hal::fugit::RateExtU32;
use unwrap_infallible::UnwrapInfallible;

use crate::{eprintln, pcb_mapping::{SdCardCs, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardSck}};

pub const SDCARD_INITIAL_FREQ_KHZ: u32 = 400;
pub const SDCARD_FULL_SPEED_FREQ_KHZ: u32 = 25_000;
#[derive(Copy, Clone)]
struct VolumeInfo {
    root_dir: RawDirectory,
    volume: RawVolume,
    volume_id: usize,
    free_space: Option<u64>,
}

pub struct SdManager {
    extra_pins: SdCardExtraPins,
    vmgr: VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>,
    volume_info: Option<VolumeInfo>,
    file: Option<RawFile>,
    // What the insertion state of the SD card was in the previous loop. Used to compare for insertions/removals.
    card_previously_inserted: bool,
}
const MBR_MAX_PARTITIONS: usize = 4;
type SpiBus0Enabled = Spi<Enabled,SPI0,(SdCardMosi, SdCardMiso, SdCardSck), {crate::board::BITS_PER_SPI_PACKET}>;
impl SdManager {
    pub fn new(spi_bus: SpiBus0Enabled, cs: SdCardCs, delay: Timer, mut extra_pins: SdCardExtraPins, rtc: RealTimeClock) -> Self {
        let rtc_wrapper = RtcWrapper{rtc};
        let spi_driver = SDCardSPIDriver{spi_bus, cs};
        let new_card = embedded_sdmmc::SdCard::new(spi_driver, delay);

        Self {
            card_previously_inserted: extra_pins.card_detect.is_low().unwrap_infallible(),
            extra_pins,
            vmgr: VolumeManager::new(new_card, rtc_wrapper),
            volume_info: None,
            file: None,
        }
    }

    /// Whether there is a volume on the SD card that is formatted in a way we can understand.
    /// 
    /// If yes, opens the volume.
    pub fn is_card_formatted(&mut self) -> bool {
        self.try_open_volume().is_ok()
    }

    fn try_open_volume(&mut self) -> Result<VolumeInfo, embedded_sdmmc::Error<SdCardError>> {
        if let Some(volume_info) = self.volume_info {
            return Ok(volume_info)
        }

        for i in 0..MBR_MAX_PARTITIONS {
            let Ok(volume) = self.vmgr.open_raw_volume(VolumeIdx(i)) else { continue };
            let root_dir = self.vmgr.open_root_dir(volume).unwrap(); // TODO
            let volume_info = VolumeInfo{root_dir, volume, volume_id: i, free_space: None};
            self.volume_info = Some(volume_info);
            return Ok(volume_info)
        }
        Err(embedded_sdmmc::Error::FormatError("No readable volumes"))
    }

    /// Open a file for writing, creating it if it doesn't exist already. If it does exist the contents are cleared.
    pub fn try_open_file(&mut self, name: &str) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
        let volume_info = self.try_open_volume()?;
        let file = self.vmgr.open_file_in_dir(volume_info.root_dir, name, Mode::ReadWriteCreateOrTruncate)?;
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
        
        self.vmgr.write(file, bytes)?;

        // Update free space, if cached
        if let Some(VolumeInfo{free_space: Some(ref mut space), ..}) = self.volume_info {
            *space -= bytes.len() as u64;
        }
        Ok(())
    }

    /// Whether the SD card can be safely removed right now
    pub fn is_safe_to_remove(&self) -> bool {
        !self.vmgr.has_open_handles()
    }

    /// Get the number of bytes free on the SD card
    pub fn get_free_space_bytes(&mut self) -> Option<u64> {
        let volume_info = self.try_open_volume().ok()?;
        if volume_info.free_space.is_some() {
            return volume_info.free_space;
        }
        let total_space = Self::get_volume_size(volume_info.volume_id, &mut self.vmgr);
        let used_space: u64 = Self::get_volume_used_space(volume_info.root_dir, &mut self.vmgr);
        let free_space = total_space - used_space;
        self.volume_info = Some(VolumeInfo {free_space: Some(free_space), ..volume_info });
        Some(free_space)
    }

    fn get_volume_size(volume_id: usize, vmgr: &mut VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>) -> u64 {
        // Read MBR (first 512 bytes on disk)
        let mut block = [Block::new()]; 
        vmgr.device().read(&mut block, BlockIdx(0), "Reading MBR").unwrap(); // TODO
        let mbr: [u8; 512] = block[0].contents;
        
        /// Volume information begins at address 0x1BE in the MBR. 
        const PARTITION_DATA_OFFSET: usize = 0x1BE;
        /// The MBR data for each partition is 16 bytes.
        const PARTITION_DATA_SIZE: usize = 16;

        let our_partition_offset = PARTITION_DATA_OFFSET + PARTITION_DATA_SIZE*volume_id; // 0x1BE if index = 0, 0x1CE if index = 1, etc.
        let our_partition_data: [u8; 16] = mbr[our_partition_offset..our_partition_offset+PARTITION_DATA_SIZE].try_into().unwrap_or_else(|_| unreachable!());
        // Number of sectors in volume is the last 4 bytes of the volume information
        let our_partition_length_data: [u8; 4] = our_partition_data[12..16].try_into().unwrap_or_else(|_| unreachable!());
        // Use u32::from_le_bytes() to convert little endian [u8; 4] to u32
        let our_partition_length_sectors = u32::from_le_bytes(our_partition_length_data);
        let total_space_bytes: u64 = our_partition_length_sectors as u64 * Block::LEN as u64;
        total_space_bytes
    } 

    /// Calculate used space in volume. Walk the filesystem, get the length of each file. Sum them.
    fn get_volume_used_space(root_dir: RawDirectory, vmgr: &mut VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>) -> u64 {
        // If we have the right volume open already, use that
        Self::get_folder_size(root_dir, vmgr)
    }

    /// Recursively get the size of a folder
    fn get_folder_size(raw_dir: RawDirectory, vmgr: &mut VolumeManager<embedded_sdmmc::SdCard<SDCardSPIDriver, Timer>, RtcWrapper>) -> u64 {
        let mut total: u64 = 0;
        let (mut files, mut folders) = (Vec::new(), Vec::new());
        
        const THIS_DIR: ShortFileName = ShortFileName::this_dir();
        const PARENT_DIR: ShortFileName = ShortFileName::parent_dir();
        vmgr.iterate_dir(raw_dir, |dir_entry: &DirEntry| {
            if matches!(dir_entry.name, THIS_DIR | PARENT_DIR) {}
            else if dir_entry.attributes.is_directory() {
                folders.push(dir_entry.clone());
            }
            else {
                files.push(dir_entry.clone());
            }
        }).unwrap(); // TODO
        
        for file_info in files {
            total += file_info.size as u64;
        }
        for folder_info in folders {
            let folder = vmgr.open_dir(raw_dir, folder_info.name).unwrap(); // TODO: Potentially fallible, we may hit the max number of open folders
            // Recurse
            total += Self::get_folder_size(folder, vmgr);
            vmgr.close_dir(folder).unwrap(); // TODO
        }
        total
    }

    /// Initialise the SD card after insertion. Must be done before the SD card can be communicated with
    pub fn initialise_card(&mut self, peripheral_clock: &PeripheralClock) {
        // Ensure SPI bus is clocked at 400kHz for initialisation, then
        // send at least 74 clock pulses (without chip select) to wake up card,
        // then reconfigure SPI back to 25MHz
        self.vmgr.device().spi(|driver| {
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), SDCARD_INITIAL_FREQ_KHZ.kHz());
            driver.spi_bus.write(&[0;10]).unwrap_infallible(); 
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), SDCARD_FULL_SPEED_FREQ_KHZ.kHz())
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
        if let Some(file) = self.file.take() {
            self.vmgr.close_file(file)?;
        }
        if let Some(volume_info) = self.volume_info.take() {
            self.vmgr.close_dir(volume_info.root_dir)?;
            self.vmgr.close_volume(volume_info.volume)?;
        }
        Ok(())
    }

    /// Reset the state of the sd_manager and all subcomponents after an SD card is unexpectedly removed. 
    /// 
    /// This includes dealing with files that should have been closed, etc.
    pub fn reset_after_unexpected_removal(self) -> SdManager {
        // Can't close open files or volumes as this requires communication with the SD card. Rebuild internals.
        let (sd_card, rtc_wrapper) = self.vmgr.free();
        sd_card.mark_card_uninit();
        SdManager{ extra_pins: self.extra_pins, vmgr: VolumeManager::new(sd_card, rtc_wrapper), volume_info: None, file: None, card_previously_inserted: self.card_previously_inserted }
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
            free_space_bytes: self.get_free_space_bytes()
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
impl SdCardInfo {
    pub fn is_ready(&self) -> bool {
        self.is_inserted && self.is_writable == Some(true) && self.is_formatted == Some(true)
    }
}

/// Wrapper layer that implements embedded_hal SpiDevice for the rp2040-hal SPI bus (confusingly also called SpiDevice)
struct SDCardSPIDriver {
    spi_bus: Spi<Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), { crate::board::BITS_PER_SPI_PACKET }>,
    cs: SdCardCs,
}
impl ErrorType for SDCardSPIDriver{
    type Error = embedded_hal::spi::ErrorKind; // For now.
}
impl SpiDevice for SDCardSPIDriver {
    fn transaction(&mut self, operations: &mut [embedded_hal::spi::Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.cs.set_low().unwrap_infallible();
        use embedded_hal::spi::Operation::*;
        for op in operations {
            match op {
                Read(buf) =>                self.spi_bus.read(buf),
                Write(buf) =>               self.spi_bus.write(buf),
                Transfer(rd_buf, wr_buf) => self.spi_bus.transfer(rd_buf, wr_buf),
                TransferInPlace(buf) =>     self.spi_bus.transfer_in_place(buf),
                DelayNs(_) =>               return Err(embedded_hal::spi::ErrorKind::Other), // embedded_sdmmc uses a separate delay object
            }.unwrap_infallible();
        }
        self.spi_bus.flush().unwrap_infallible();
        self.cs.set_high().unwrap_infallible();
        Ok(())
    }
}

struct RtcWrapper {
    rtc: rp_pico::hal::rtc::RealTimeClock,
}
impl TimeSource for RtcWrapper {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        let timestamp = self.rtc.now().unwrap(); // TODO
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