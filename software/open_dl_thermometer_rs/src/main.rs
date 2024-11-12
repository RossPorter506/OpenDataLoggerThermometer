#![no_std]
#![no_main]
#![feature(variant_count)]
use core::{cell::RefCell, sync::atomic::{AtomicU8,AtomicBool, Ordering}};
use embedded_sdmmc::TimeSource;
use panic_halt as _;



use rp_pico::{
    entry,
    hal::{
        clocks::{ClocksManager, PeripheralClock, UsbClock}, 
        fugit::{ExtU32, RateExtU32}, 
        pwm::{self, FreeRunning, Pwm0, Slice},
        timer::{Alarm, Alarm0}, 
        gpio, spi, usb::UsbBus, Clock, Sio, Spi, Timer, Watchdog, I2C
    },
    pac::{self, interrupt, CLOCKS, I2C0, IO_BANK0, PADS_BANK0, PLL_SYS, PLL_USB, PWM, RESETS, SIO, SPI0, TIMER, USBCTRL_DPRAM, USBCTRL_REGS, WATCHDOG, XOSC},
};
use embedded_hal::{digital::{InputPin, OutputPin}, spi::{ErrorType, SpiBus, SpiDevice}};
use critical_section::Mutex;
use sd_card::SdManager;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use arrayvec::ArrayVec;
use atomic_enum::atomic_enum;

mod config; use config::{Config, Status::*};
mod constants; use constants::*;
mod pcb_mapping {include!("pcb_v1_mapping.rs");} 
use pcb_mapping::{ButtonPins, DisplayPins, DisplayScl, DisplaySda, SdCardCs, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardPins, SdCardSPIPins, SdCardSck, TempPowerPins, TempSensePins};
mod lmt01; use lmt01::{TempSensors, CHARS_PER_READING};
mod display; use display::IncrementalDisplayWriter;
mod sd_card;
mod pio;
mod state_machine;
use state_machine::{
    ConfigChannelSelectSelectables as ConChanSel, 
    ConfigOutputsSelectables::{self as ConOutSel, *}, 
    ConfigSDFilenameSelectables::{self as ConSDName, *}, 
    ConfigSDStatusSelectables as ConSdStatSel, 
    ConfigSampleRateSelectables::{self as ConRateSel, *}, 
    DatalogConfirmStopSelectables::*, 
    DatalogErrorSDFullSelectables::*, 
    DatalogErrorSdUnexpectedRemovalSelectables as DlogSdRemovSel, 
    DatalogSDSafeToRemoveSelectables as DlogSafeRemovSel, 
    DatalogSDWritingSelectables as DlogSdWrtSel, 
    DatalogTemperaturesSelectables as DlogTempSel, 
    MainmenuSelectables::*, State::*, UpdateReason, 
    ViewTemperaturesSelectables as ViewTemp
};

extern crate alloc;

/* TODO:
Display driver
SD card driver
Determine when SD card is safe to remove
Maybe wrap everything up into a nice struct
Stretch goal: Synchronise time with Pico W and NTP. Maybe not doable - cyw43 doesn't seem to support EAP WAP?
*/

/*
struct SystemPeripherals {
    temp_sensors: TempSensors,
    display: Display,
    sd_card: SDCard,
    usb: USB,
}
impl SystemPeripherals {
}*/

/// Implements embedded_hal SpiDevice
struct SDCardSPIDriver {
    spi_bus: Spi<spi::Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), BITS_PER_SPI_PACKET>,
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
struct RtcWrapper {
    rtc: rp_pico::hal::rtc::RealTimeClock,
}

// Heap so we can use Vec, etc.. Try to use ArrayVec where possible though.
use embedded_alloc::LlffHeap as Heap;
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
fn configure_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024; // TODO
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
}

#[entry]
fn main() -> ! {
    configure_heap();
    // Take ownership of peripherals
    let mut registers = pac::Peripherals::take().unwrap();
    
    // GPIO pin groups
    let (temp_power, temp_sense, sdcard_pins, display_pins) = collect_pins(registers.SIO, registers.IO_BANK0, registers.PADS_BANK0, &mut registers.RESETS);

    // PIO
    let pio_state_machines = lmt01::configure_pios_for_lmt01(registers.PIO0, registers.PIO1, &mut registers.RESETS, &temp_sense);
    let mut temp_sensors = TempSensors::new(temp_power, temp_sense, pio_state_machines);

    // System clocks
    let (_watchdog, clocks) = configure_clocks(registers.WATCHDOG, registers.XOSC, registers.CLOCKS, registers.PLL_SYS, registers.PLL_USB, &mut registers.RESETS);

    // System configuration
    let mut config = Config::new();
    let str = alloc::string::String::from_utf8_lossy(&config.sd.filename);

    // Timers
    let (mut system_timer, mut sample_rate_timer, mut sensors_ready_timer) = configure_timers(registers.PWM, registers.TIMER, &mut registers.RESETS, &clocks);

    // I2C and Display
    let mut i2c = configure_i2c(registers.I2C0, display_pins, &mut registers.RESETS, &clocks);
    let mut display_manager = IncrementalDisplayWriter::new(&config, &mut i2c);
    display_manager.load_custom_chars(&mut system_timer);

    // SPI
    let spi_bus = configure_spi(registers.SPI0, sdcard_pins.spi, &mut registers.RESETS, &clocks);

    // RTC
    let rtc = rp_pico::hal::rtc::RealTimeClock::new(registers.RTC, clocks.rtc_clock, &mut registers.RESETS, rp_pico::hal::rtc::DateTime{ year: 2024, month: 1, day: 1, day_of_week: rp_pico::hal::rtc::DayOfWeek::Monday, hour: 1, minute: 1, second: 1 }).unwrap();
    let rtc_wrapper = RtcWrapper{rtc};

    // SD card manager
    let mut sd_manager = crate::sd_card::SdManager::new(SDCardSPIDriver{spi_bus, cs: sdcard_pins.cs}, system_timer, sdcard_pins.extra, rtc_wrapper);

    // USB
    let usb_bus = configure_usb_bus(registers.USBCTRL_REGS, registers.USBCTRL_DPRAM, &mut registers.RESETS, clocks.usb_clock);
    let (mut usb_serial, mut usb_device) = configure_usb(&usb_bus);

    // Whether we are going to update state and redraw screen
    let mut update_available: Option<UpdateReason>;

    // Write controls for SD card. false when transmissions are complete, for now
    let mut write_to_sd = false;

    // Buffers
    let mut sensor_values = [[0u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS]; // Most recent sensor values, to be written to display, serial, or SD card
    let mut serial_buffer = ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS}>::new(); // We will write to serial as we receive, so buffer need only be big enough for one lot of readings. ASCII.
    let mut spi_buffer = ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS*16 /*Arbitrary size*/}>::new(); // We store values until we're some multiple of the SD card block size. ASCII.

    // Autodetect any connected sensors
    config.enabled_channels = autodetect_sensor_channels(&mut system_timer, &mut temp_sensors);

    loop {
        update_available = None;

        if SENSOR_READINGS_AVAILABLE.load(Ordering::Relaxed) {
            read_sensors(&mut temp_sensors, &mut sensor_values, &mut serial_buffer, &mut spi_buffer, &config, &mut sample_rate_timer, &mut write_to_sd);
            update_available = Some(UpdateReason::NewSensorValues);
        }
        
        if READY_TO_START_NEXT_READING.load(Ordering::Relaxed) {
            start_next_sensor_reading(&mut temp_sensors, &mut sensors_ready_timer);
        }

        // Check for button presses. This does override the `update_available` value from the above sensor readings, but
        // button presses always prompt a screen redraw anyway, so any new sensor values will be displayed regardless. 
        update_available = match BUTTON_STATE.load(Ordering::Relaxed) {
            ButtonState::NextButton => Some(UpdateReason::NextButton),
            ButtonState::SelectButton => Some(UpdateReason::SelectButton),
            _ => update_available, // no change
        };

        // Deal with SD card
        if write_to_sd {
            // Is SD card full?
            // Is SD card safe to remove? (if relevant)
            //if sd_buffer.is_empty() {write_to_sd = false;}
            //else {
            //  if spi_tx_buffer.has_room() {spi_tx_buffer.push(sd_buffer.pop())}
            //}

            todo!();
        }
        
        // Check if the card is inserted or removed
        monitor_sdcard_state(&mut sd_manager, &mut config, &mut update_available, &clocks.peripheral_clock);

        // Serial
        manage_serial_comms(&mut usb_device, &mut usb_serial, &mut serial_buffer);

        // Service events and update available state, if required
        if let Some(update_reason) = update_available {
            service_button_event(&mut config, &update_reason);
            state_machine::next_state(&mut config, &update_reason);
            display_manager.determine_new_screen(&mut system_timer, &config, Some(&sensor_values));
            if config.status == Idle { sample_rate_timer.disable(); } else { sample_rate_timer.enable(); }
        }

        // Screen updates
        display_manager.incremental_update(&mut system_timer);
    }
}

/// Listen for an SD card being inserted or removed and initialise, if required
fn monitor_sdcard_state(sd_manager: &mut crate::SdManager, config: &mut Config, update_available: &mut Option<UpdateReason>, peripheral_clock: &PeripheralClock) {
    let sd_present = sd_manager.extra_pins.card_detect.is_low().unwrap();
    if config.sd.card_detected && !sd_present { // SD card removed
        config.sd.card_detected = false;
        config.sd.card_writable = false;
        config.sd.card_formatted = false;
        config.sd.free_space_bytes = 0;

        if config.sd.safe_to_remove {
            // SD card removed safely
            // Anything extra to do here?
            todo!();
        }
        else {
            // SD card removed unexpectedly
            // Move to SD card error screen
            *update_available = Some(UpdateReason::SDRemovedUnexpectedly);
            todo!();
        }
    }
    else if !config.sd.card_detected && sd_present { // SD card inserted
        // Ensure SPI bus is clocked at 400kHz for initialisation, then
        // send at least 74 clock pulses (without chip select) to wake up card,
        // then reconfigure SPI back to 25MHz
        sd_manager.vmgr.device().spi(|driver| {
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), 400.kHz());
            let _ = driver.spi_bus.write(&[0;10]); 
            driver.spi_bus.set_baudrate(peripheral_clock.freq(), 25.MHz())
        });
        config.sd.card_detected = true;
        config.sd.card_writable = sd_manager.extra_pins.write_protect.is_low().unwrap();
        config.sd.card_formatted = crate::SdManager::is_card_formatted(&mut sd_manager.vmgr);
        config.sd.free_space_bytes = todo!();
        todo!();
    }
}

/// Read values from the LMT01 sensors. 
fn read_sensors(temp_sensors: &mut TempSensors, 
        sensor_values: &mut [[u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS], 
        serial_buffer: &mut ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS}>, 
        spi_buffer: &mut ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS*16}>, 
        config: &Config, sample_rate_timer: &mut Slice<Pwm0, FreeRunning>,
        write_to_sd: &mut bool) {

    *sensor_values = temp_sensors.read_temperatures().map(lmt01::temp_to_string);
    temp_sensors.pios.pause_all();
    let flattened_values = sensor_values.iter().flatten().copied(); // SD card and serial (right now...) don't care about char grouping, so flatten [[u8; _]; _] into [u8; _]
    if config.sd.enabled {
        spi_buffer.extend(flattened_values.clone());
        if spi_buffer.is_full() { *write_to_sd = true; }
    }
    
    if config.serial.enabled {
        *serial_buffer = flattened_values.collect();
    }

    SENSOR_READINGS_AVAILABLE.store(false, Ordering::Relaxed);
    sample_rate_timer.clear_interrupt(); // Clear the interrupt flag for the 125ms timer. Should be done in the PWM interrupt, but this is good enough 
}

/// If it's time to get another reading then restart the LMT01 sensors.
fn start_next_sensor_reading(temp_sensors: &mut TempSensors, sensors_ready_timer: &mut Alarm0) {
    temp_sensors.power.turn_off();
    // May need a delay here
    temp_sensors.power.turn_on();
    temp_sensors.pios.restart_all();
    sensors_ready_timer.clear_interrupt(); // Clear the interrupt flag for the 105ms timer. Should be done in the TIMER0 interrupt, but this is good enough 
    let _ = sensors_ready_timer.schedule((lmt01::SENSOR_MAX_TIME_FOR_READING_MS+1).millis());
    READY_TO_START_NEXT_READING.store(false, Ordering::Relaxed);
}

/// Try to send data over serial.
fn manage_serial_comms(usb_device: &mut UsbDevice<UsbBus>, usb_serial: &mut SerialPort<UsbBus>, serial_buffer: &mut ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS}>) {
    if !serial_buffer.is_empty() && usb_device.poll(&mut [usb_serial]) {
        // We don't care what gets sent to us, but we need to poll anyway to stay USB compliant
        // Try to send everything we have
        match usb_serial.write(serial_buffer.as_slice()) {
            // Remove whatever was successfully sent from our buffer
            Ok(len) => { serial_buffer.drain(0..len); }
            // Err(WouldBlock) implies buffer is full.
            Err(UsbError::WouldBlock) => (),
            Err(a) => panic!("{a:?}"),
        };
    }
}

/// Figure out what to do when the select button is pressed, based on the current state and what is currently selected
/// 
/// This function deals exclusively with state machine outputs. Changes to the system state are calculated in `state_machine::next_state()``
fn service_button_event(config: &mut Config, update_reason: &UpdateReason) {
    if *update_reason != UpdateReason::SelectButton {
        return;
    }
    // If we press select while we have a configurable item selected we need to toggle it
    match &config.curr_state {
        Mainmenu(View) => config.status = Sampling,
        Mainmenu(Datalog) => config.status = SamplingAndDatalogging,
        Mainmenu(Configure) => (),

        ViewTemperatures(ViewTemp::None) => config.status = Idle, 
        DatalogTemperatures(DlogTempSel::None) => (), // Don't exit datalogging mode yet, confirm screen first

        ConfigOutputs(Serial) => config.serial.enabled ^= true,
        ConfigOutputs(SDCard) => config.sd.enabled ^= true,
        ConfigOutputs(ConOutSel::Next) => (),

        ConfigSDStatus(ConSdStatSel::Next) => (),

        ConfigSDFilename(ConSDName::Next) => (), // don't accidentally capture this case in the catch-all `char_num` pattern below.
        ConfigSDFilename(Filetype) => config.sd.filetype = config.sd.filetype.next(),
        ConfigSDFilename(char_num) => cycle_ascii_char(&mut config.sd.filename[*char_num as usize]), //filename characters 0-9

        ConfigChannelSelect(ConChanSel::Next) => (), // don't accidentally capture this case in the catch-all `ch_num` pattern below.
        ConfigChannelSelect(ch_num) => config.enabled_channels[*ch_num as usize] ^= true, // selected channels 1-8

        ConfigSampleRate(SampleRate) => {cycle_sample_rate(&mut config.samples_per_sec); WRAPS_PER_SAMPLE.store(8/config.samples_per_sec, Ordering::Relaxed);},
        ConfigSampleRate(ConRateSel::Next) => (),

        DatalogConfirmStop(ConfirmStop) => config.status = Idle,
        DatalogConfirmStop(CancelStop) => (),

        DatalogErrorSDFull(ContinueWithoutSD) => config.sd.enabled = false,
        DatalogErrorSDFull(StopDatalogging) => config.status = Idle,
        
        DatalogSDWriting(DlogSdWrtSel::None) => (),

        DatalogSDSafeToRemove(DlogSafeRemovSel::Next) => (),

        DatalogSDUnexpectedRemoval(DlogSdRemovSel::ContinueWithoutSD) => config.sd.enabled = false,
        DatalogSDUnexpectedRemoval(DlogSdRemovSel::StopDatalogging) => config.status = Idle,
    };
}
/// Cycle through alphanumeric ASCII values.
/// a -> b, ... z -> 0, ... 9 -> a
fn cycle_ascii_char(char: &mut u8) {
    *char = match *char {
        b'a'..=b'y' => *char + 1,
        b'z' => b'0',
        b'0'..=b'8' => *char + 1,
        _ => b'a',
    };
}
/// Cycle between 1, 2, 4, 8
fn cycle_sample_rate(rate: &mut u8) {
    *rate = match *rate {
        1 => 2,
        2 => 4,
        4 => 8,
        _ => 1,
    }
}

/// Whether it's time to read the sensors for new values
static SENSOR_READINGS_AVAILABLE: AtomicBool = AtomicBool::new(false);
// Set flag indicating we should check sensor values
// Interrupt fires 105ms after sensors are reset, values should be ready now.
#[interrupt]
fn TIMER_IRQ_0() {
    SENSOR_READINGS_AVAILABLE.store(true, Ordering::Relaxed);
}

/// How many times the 8Hz PWM interrupt must trigger per sample of the sensors, e.g.
/// 
/// `WRAPS_PER_SAMPLE = 8/n` where `n` is the sampling frequency
static WRAPS_PER_SAMPLE: AtomicU8 = AtomicU8::new(1);

/// Whether it's time to reset the sensors to generate new values
static READY_TO_START_NEXT_READING: AtomicBool = AtomicBool::new(false);
// Interrupt fires some multiple of 125ms (based on sample rate). Used to schedule sensor resets
#[interrupt]
fn PWM_IRQ_WRAP() {
    static NUM_WRAPS: AtomicU8 = AtomicU8::new(1);
    
    if NUM_WRAPS.load(Ordering::Relaxed) == WRAPS_PER_SAMPLE.load(Ordering::Relaxed) {
        READY_TO_START_NEXT_READING.store(true, Ordering::Relaxed);
        NUM_WRAPS.store(1, Ordering::Relaxed);
    }
    else { NUM_WRAPS.store(NUM_WRAPS.load(Ordering::Relaxed)+1, Ordering::Relaxed); }
}


/// Whether any buttons have been pressed
static BUTTON_STATE: AtomicButtonState = AtomicButtonState::new(ButtonState::None);
/// Interrupt-accessible container for button pins. 'None' until interrupt is configured by `configure_button_pins`.
static BUTTON_PINS: Mutex<RefCell<Option<pcb_mapping::ButtonPins>>> = Mutex::new(RefCell::new(None));
// Next and Select button interrupts. Set flags for main process. 
// We don't really care if both buttons are pressed at once. Hopefully the main loop should be tight enough not to matter
#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        if let Some(buttons) = BUTTON_PINS.take(cs) {
            if buttons.select.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.store(ButtonState::SelectButton, Ordering::Relaxed);
            } else if buttons.next.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.store(ButtonState::NextButton, Ordering::Relaxed);
            }
        }
    });
}

/// Whether buttons have been pressed
#[atomic_enum]
#[derive(PartialEq)]
enum ButtonState {
    None = 0,
    NextButton,
    SelectButton,
}


/// Collect individual pins and return structs of related pins, configured for use.
fn collect_pins(sio: SIO, io_bank0: IO_BANK0, pads_bank0: PADS_BANK0, resets: &mut RESETS) -> (TempPowerPins, TempSensePins, SdCardPins, DisplayPins) {
    let sio = Sio::new(sio);
    let pins = rp_pico::Pins::new(
        io_bank0,
        pads_bank0,
        sio.gpio_bank0,
        resets,
    );
    let mut temp_power = TempPowerPins::new(
        pins.gpio0.reconfigure(),
        pins.gpio2.reconfigure(),
        pins.gpio4.reconfigure(),
        pins.gpio6.reconfigure(),
        pins.gpio8.reconfigure(),
        pins.gpio10.reconfigure(),
        pins.gpio12.reconfigure(),
        pins.gpio14.reconfigure(),
    );
    temp_power.turn_off();
    let temp_sense = TempSensePins {
        vn1: pins.gpio1.reconfigure(),
        vn2: pins.gpio3.reconfigure(),
        vn3: pins.gpio5.reconfigure(),
        vn4: pins.gpio7.reconfigure(),
        vn5: pins.gpio9.reconfigure(),
        vn6: pins.gpio11.reconfigure(),
        vn7: pins.gpio13.reconfigure(),
        vn8: pins.gpio15.reconfigure(),
    };
    let sd_card_pins = SdCardPins {
        spi: SdCardSPIPins{
            mosi: pins.gpio19.reconfigure(),
            miso: pins.gpio16.reconfigure(),
            sck: pins.gpio18.reconfigure(),
        },
        cs: pins.gpio17.reconfigure(),
        extra: SdCardExtraPins {
            write_protect: pins.gpio22.reconfigure(),
            card_detect: pins.gpio26.reconfigure(),
        }
        
    };
    let display_pins = DisplayPins {
        sda: pins.gpio20.reconfigure(),
        scl: pins.gpio21.reconfigure(),
    };
    configure_button_pins(pins.gpio27.reconfigure(), pins.gpio28.reconfigure());
    (temp_power, temp_sense, sd_card_pins, display_pins)
}

/// Set up button pins for interrupt usage. Button pins are passed to interrupt
fn configure_button_pins(select: pcb_mapping::SelectButton, next: pcb_mapping::NextButton) {
    next.set_schmitt_enabled(true);
    next.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);

    select.set_schmitt_enabled(true);
    select.set_interrupt_enabled(rp_pico::hal::gpio::Interrupt::EdgeLow, true);

    // Move pins into global variable so interrupt can access them
    critical_section::with(|cs| {
        BUTTON_PINS
            .borrow(cs)
            .replace(Some(ButtonPins { select, next }));
    });
    // Enable interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
}

/// Try to get a reading from all sensor channels. If the channel returns `None` then `false`, otherwise `true`.
fn autodetect_sensor_channels(delay: &mut impl embedded_hal::delay::DelayNs, temp_sensors: &mut TempSensors) -> [bool; NUM_SENSOR_CHANNELS] {
    temp_sensors.power.turn_off();
    // May need a delay here
    temp_sensors.power.turn_on();
    temp_sensors.pios.restart_all();

    delay.delay_ms(lmt01::SENSOR_MAX_TIME_FOR_READING_MS+1);
    let connected = temp_sensors.read_temperatures().map(|opt| opt.is_some());

    temp_sensors.pios.pause_all();
    temp_sensors.power.turn_off();

    connected
}

const BITS_PER_SPI_PACKET: u8 = 8;
fn configure_spi(spi0: SPI0, sdcard_pins: SdCardSPIPins, resets: &mut RESETS, clocks: &ClocksManager) -> rp_pico::hal::Spi<spi::Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), BITS_PER_SPI_PACKET> {
    let sdcard_spi = Spi::<_,_,_,BITS_PER_SPI_PACKET>::new(spi0, (sdcard_pins.mosi, sdcard_pins.miso, sdcard_pins.sck));
    
    // Start at 400kHz before negotiation, then 25MHz after.
    sdcard_spi.init(resets, clocks.peripheral_clock.freq(), 400.kHz(), spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0))
}

fn configure_i2c(i2c0: I2C0, display_pins: DisplayPins, resets: &mut RESETS, clocks: &ClocksManager,) -> liquid_crystal::I2C<rp_pico::hal::I2C<I2C0, (DisplaySda, DisplayScl)>> {
    let display_i2c = I2C::i2c0(i2c0, display_pins.sda, display_pins.scl, 100.kHz(), resets, clocks.system_clock.freq());

    liquid_crystal::I2C::new(display_i2c, display::DISPLAY_I2C_ADDRESS)
}

fn configure_clocks(watchdog: WATCHDOG, xosc: XOSC, clocks: CLOCKS, pll_sys: PLL_SYS, pll_usb: PLL_USB, resets: &mut RESETS) -> (Watchdog, ClocksManager) {
    let mut watchdog = rp_pico::hal::Watchdog::new(watchdog);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ, xosc, clocks,
        pll_sys, pll_usb,
        resets, &mut watchdog,
    ).ok()
     .unwrap();

    (watchdog, clocks)
}

fn configure_timers(pwm: PWM, timer: TIMER, resets: &mut RESETS, clocks: &ClocksManager) -> (Timer, pwm::Slice<pwm::Pwm0, FreeRunning>, Alarm0) {
    // PWM used as timer. Used to control how often we ask the LMT01's to produce a value for us
    let slices = rp_pico::hal::pwm::Slices::new(pwm, resets);
    let mut sample_rate_timer = slices.pwm0.into_mode::<FreeRunning>();

    core::assert!(clocks.system_clock.freq().to_MHz() < 128); // Assume freq < 128MHz because prescaler is 8-bit and we multiply clock by 2

    sample_rate_timer.default_config();
    sample_rate_timer.set_div_int( (clocks.system_clock.freq().to_MHz() * 2) as u8 ); // 2us resolution => max ~130ms range
    sample_rate_timer.set_top(62_500); // Want 125ms range
    sample_rate_timer.enable_interrupt();

    // System timer, general use (delays, etc.)
    let mut system_timer = rp_pico::hal::Timer::new(timer, resets, clocks);

    // We use alarm 0 used to tell us when we can retrieve a value from the LMT01's. 
    let mut sensors_ready_timer = system_timer.alarm_0().unwrap();
    sensors_ready_timer.enable_interrupt();

    (system_timer, sample_rate_timer, sensors_ready_timer)
}

fn configure_usb_bus(usbctrl_regs: USBCTRL_REGS, usbctrl_dpram: USBCTRL_DPRAM, resets: &mut RESETS, usb_clock: UsbClock) -> UsbBusAllocator<UsbBus> {
    UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        usbctrl_regs, usbctrl_dpram,
        usb_clock, true, resets,
    ))
}

fn configure_usb(usb_bus: &UsbBusAllocator<UsbBus>) -> (SerialPort<UsbBus>, UsbDevice<UsbBus>) {
    let usb_serial = SerialPort::new(usb_bus);

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company") // TODO
            .product("Serial port") 
            .serial_number("TEST")]) // TODO
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    (usb_serial, usb_dev)
}