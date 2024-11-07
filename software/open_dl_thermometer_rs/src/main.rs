#![no_std]
#![no_main]
#![feature(variant_count)]
use arrayvec::ArrayVec;
use display::IncrementalDisplayWriter;
use embedded_hal::{digital::OutputPin, spi::{ErrorType, SpiBus, SpiDevice}};
use core::{cell::RefCell, sync::atomic::AtomicU8};
use critical_section::Mutex;
use panic_halt as _;

use rp_pico::{
    entry,
    hal::{clocks::{ClocksManager, UsbClock}, fugit::{ExtU32, RateExtU32}, gpio, pwm::{self, FreeRunning}, spi, timer::{Alarm, Alarm0}, usb::UsbBus, Clock, Sio, Spi, Timer, Watchdog, I2C},
    pac::{self, interrupt, CLOCKS, I2C0, IO_BANK0, PADS_BANK0, PLL_SYS, PLL_USB, PWM, RESETS, SIO, SPI0, TIMER, USBCTRL_DPRAM, USBCTRL_REGS, WATCHDOG, XOSC},
};

use atomic_enum::atomic_enum;
use core::sync::atomic::{AtomicBool, Ordering};

mod config;
use config::{Config, Status::*};
mod constants;
use constants::*;
mod pcb_mapping {include!("pcb_v1_mapping.rs");} use pcb_mapping::{ButtonPins, DisplayPins, DisplayScl, DisplaySda, SDCardPins, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardSPIPins, SdCardSck, TempPowerPins, TempSensePins};
mod lmt01; use lmt01::{TempSensors, CHARS_PER_READING};
mod display;
mod pio;
mod state_machine;
use state_machine::{
    ConfigChannelSelectSelectables as ConChanSel,
    ConfigOutputsSelectables::*,
    ConfigSDFilenameSelectables::{self as ConSDName, *},
    ConfigSampleRateSelectables::*,
    DatalogConfirmStopSelectables::*,
    DatalogErrorSDFullSelectables::*,
    MainmenuSelectables::*,
    State::*,
    UpdateReason, ViewTemperaturesSelectables as ViewTemp,
};

// USB
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;
// Used to demonstrate writing formatted strings

/* TODO:
Display driver
SD card driver
Move SD card initialisation somewhere into the loop
Maybe wrap everything up into a nice struct
Stretch goal: Synchronise time with Pico W and NTP
Stretch goal: Autodetect connected sensor channels and set accordingly
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
    pins: SdCardExtraPins,
}
impl ErrorType for SDCardSPIDriver{
    type Error = embedded_hal::spi::ErrorKind; // For now.
}
impl SpiDevice for SDCardSPIDriver {
    fn transaction(&mut self, operations: &mut [embedded_hal::spi::Operation<'_, u8>]) -> Result<(), Self::Error> {
        let _ = self.pins.cs.set_low();
        for op in operations {
            let _ = match op {
                embedded_hal::spi::Operation::Read(buf) =>                  self.spi_bus.read(buf),
                embedded_hal::spi::Operation::Write(buf) =>                 self.spi_bus.write(buf),
                embedded_hal::spi::Operation::Transfer(rd_buf, wr_buf) =>   self.spi_bus.transfer(rd_buf, wr_buf),
                embedded_hal::spi::Operation::TransferInPlace(buf) =>       self.spi_bus.transfer_in_place(buf),
                embedded_hal::spi::Operation::DelayNs(_) => return Err(embedded_hal::spi::ErrorKind::Other), // embedded_sdmmc uses a separate delay object
            };
        }
        let _ = self.spi_bus.flush();
        let _ = self.pins.cs.set_high();
        Ok(())
    }
}


// Heap so we can use Vec, etc.. Try to use ArrayVec where possible though.
use embedded_alloc::LlffHeap as Heap;
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
fn configure_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
}

const SENSOR_MAX_TIME_FOR_READING_MS: u32 = 104;

#[entry]
fn main() -> ! {
    configure_heap();
    // Take ownership of peripherals and pins
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

    // Timers
    let (mut system_timer, mut sample_rate_timer, mut sensors_ready_timer) = configure_timers(registers.PWM, registers.TIMER, &mut registers.RESETS, &clocks);

    // I2C and Display
    let mut i2c = configure_i2c(registers.I2C0, display_pins, &mut registers.RESETS, &clocks);
    let mut display_manager = IncrementalDisplayWriter::new(&config, &mut i2c);
    display_manager.load_custom_chars(&mut system_timer);
    
    // SPI
    let spi_bus = configure_spi(registers.SPI0, sdcard_pins.spi, &mut registers.RESETS, &clocks);
    let sdcard = embedded_sdmmc::SdCard::new(SDCardSPIDriver{spi_bus, pins: sdcard_pins.extra}, system_timer);

    // USB
    let usb_bus = configure_usb_bus(registers.USBCTRL_REGS, registers.USBCTRL_DPRAM, &mut registers.RESETS, clocks.usb_clock);
    let (mut usb_serial, mut usb_device) = configure_usb(&usb_bus);

    // Whether we are going to update state and redraw screen
    let mut update_available: Option<UpdateReason>;

    // Write controls for SD and serial. false when transmissions are complete, for now
    let mut write_to_sd = false;
    let mut write_to_serial = false;

    // Buffers
    let mut sensor_values = [[0u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS]; // Most recent sensor values, to be written to display, serial, or SD card
    let mut serial_buffer = ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS}>::new(); // We will write to serial as we receive, so buffer need only be big enough for one lot of readings. ASCII.
    let mut spi_buffer = ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS*16 /*Arbitrary size*/}>::new(); // We store values until we're some multiple of the SD card block size. ASCII.

    // Autodetect any connected sensors
    config.enabled_channels = autodetect_sensor_channels(&mut system_timer, &mut temp_sensors);

    loop {
        // Clear any updates from the previous loop
        update_available = None;

        // Read sensor values
        if READY_TO_READ_SENSORS.load(Ordering::Relaxed) {
            sensor_values = temp_sensors.read_temperatures().map(lmt01::temp_to_string);
            temp_sensors.pios.pause_all();
            let flattened_values = sensor_values.iter().flatten().copied(); // SD card and serial (right now...) don't care about char grouping, so flatten [[u8; _]; _] into [u8; _]
            if config.sd.enabled{
                spi_buffer.extend(flattened_values.clone());
                if spi_buffer.is_full() { write_to_sd = true; }
            }
            
            if config.serial.enabled {
                serial_buffer = flattened_values.collect();
                write_to_serial = true;
            }

            READY_TO_READ_SENSORS.store(false, Ordering::Relaxed);
            sample_rate_timer.clear_interrupt(); // Clear the interrupt flag for the 125ms timer. Should be done in the PWM interrupt, but this is good enough 
            update_available = Some(UpdateReason::NewSensorValues);
        }

        // Check for button presses
        update_available = match BUTTON_STATE.load(Ordering::Relaxed) {
            ButtonState::NextButton => Some(UpdateReason::NextButton),
            ButtonState::SelectButton => Some(UpdateReason::SelectButton),
            _ => update_available, // no change
        };

        // Prepare sensors for next reading
        if READY_TO_RESET_SENSORS.load(Ordering::Relaxed) {
            temp_sensors.power.turn_off();
            // May need a delay here
            temp_sensors.power.turn_on();
            temp_sensors.pios.restart_all();
            sensors_ready_timer.clear_interrupt(); // Clear the interrupt flag for the 105ms timer. Should be done in the TIMER0 interrupt, but this is good enough 
            let _ = sensors_ready_timer.schedule((SENSOR_MAX_TIME_FOR_READING_MS+1).millis());
            READY_TO_RESET_SENSORS.store(false, Ordering::Relaxed);
        }

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

        // Serial
        if write_to_serial {
            if serial_buffer.is_empty() {
                write_to_serial = false;
            } 
            else if usb_device.poll(&mut [&mut usb_serial]) {
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

        // Service events and update available state, if required
        if let Some(update_reason) = update_available {
            service_button_event(&mut config, &update_reason);
            state_machine::next_state(&mut config, &update_reason);
            display_manager.determine_new_screen(&mut system_timer, &config, Some(&sensor_values));
            if config.status == Idle { sample_rate_timer.disable(); } else { sample_rate_timer.enable(); }
        }

        display_manager.incremental_update(&mut system_timer);
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

        ViewTemperatures(ViewTemp::None) => config.status = Idle,

        ConfigOutputs(Serial) => config.serial.enabled ^= true,
        ConfigOutputs(SDCard) => config.sd.enabled ^= true,

        ConfigSDFilename(ConSDName::Next) => (), // don't accidentally capture this case in the catch-all `char_num` pattern below.
        ConfigSDFilename(Filetype) => config.sd.filetype = config.sd.filetype.next(),
        ConfigSDFilename(char_num) => cycle_ascii_char(&mut config.sd.filename[*char_num as usize]), //filename characters 0-9

        ConfigChannelSelect(ConChanSel::Next) => (), // don't accidentally capture this case in the catch-all `ch_num` pattern below.
        ConfigChannelSelect(ch_num) => config.enabled_channels[*ch_num as usize] ^= true, // selected channels 1-8

        ConfigSampleRate(SampleRate) => cycle_sample_rate(&mut config.samples_per_sec),

        DatalogConfirmStop(ConfirmStop) => config.status = Idle,

        DatalogErrorSDFull(ContinueWithoutSD) => config.sd.enabled = false,
        DatalogErrorSDFull(StopDatalogging) => config.status = Idle,
        _ => (),
    };
}
/// Cycle through alphanumeric ASCII values
/// a -> b, ... z -> 0, ... 9 -> a
fn cycle_ascii_char(char: &mut u8) {
    *char = match *char {
        b'a'..=b'y' => *char + 1,
        b'z' => b'0',
        b'0'..=b'8' => *char + 1,
        _ => b'a',
    };
}
/// Cycle between 1 - 8
fn cycle_sample_rate(rate: &mut u8) {
    *rate = match *rate {
        1 => 2,
        2 => 4,
        4 => 8,
        _ => 1,
    }
}

/// Whether it's time to read the sensors for new values
static READY_TO_READ_SENSORS: AtomicBool = AtomicBool::new(false);
// Set flag indicating we should check sensor values
// Interrupt fires 105ms after sensors are reset, values should be ready now.
#[interrupt]
fn TIMER_IRQ_0() {
    READY_TO_READ_SENSORS.store(true, Ordering::Relaxed);
}

/// How many times the 8Hz PWM interrupt must trigger per sample of the sensors, e.g.
/// 
/// `WRAPS_PER_SAMPLE = 8/n` where `n` is the sampling frequency
static WRAPS_PER_SAMPLE: AtomicU8 = AtomicU8::new(1);

/// Whether it's time to reset the sensors to generate new values
static READY_TO_RESET_SENSORS: AtomicBool = AtomicBool::new(false);
// Interrupt fires every 125ms. Used to schedule sensor resets
#[interrupt]
fn PWM_IRQ_WRAP() {
    static NUM_WRAPS: AtomicU8 = AtomicU8::new(1);
    
    if NUM_WRAPS.load(Ordering::Relaxed) == WRAPS_PER_SAMPLE.load(Ordering::Relaxed) {
        READY_TO_RESET_SENSORS.store(true, Ordering::Relaxed);
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
fn collect_pins(sio: SIO, io_bank0: IO_BANK0, pads_bank0: PADS_BANK0, resets: &mut RESETS) -> (TempPowerPins, TempSensePins, SDCardPins, DisplayPins) {
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
    let sd_card_pins = SDCardPins {
        spi: SdCardSPIPins{
            mosi: pins.gpio19.reconfigure(),
            miso: pins.gpio16.reconfigure(),
            sck: pins.gpio18.reconfigure(),
        },
        extra: SdCardExtraPins {
            cs: pins.gpio17.reconfigure(),
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

    delay.delay_ms(SENSOR_MAX_TIME_FOR_READING_MS+1);
    let connected = temp_sensors.read_temperatures().map(|opt| opt.is_some());

    temp_sensors.pios.pause_all();
    temp_sensors.power.turn_off();

    connected
}

const BITS_PER_SPI_PACKET: u8 = 8;
fn configure_spi(spi0: SPI0, sdcard_pins: SdCardSPIPins, resets: &mut RESETS, clocks: &ClocksManager) -> rp_pico::hal::Spi<spi::Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), BITS_PER_SPI_PACKET> {
    let sdcard_spi = Spi::<_,_,_,BITS_PER_SPI_PACKET>::new(spi0, (sdcard_pins.mosi, sdcard_pins.miso, sdcard_pins.sck));
    
    // Start at 400kHz before negotiation, then 25MHz after.
    let mut sdcard_spi = sdcard_spi.init(resets, clocks.peripheral_clock.freq(), 400.kHz(), spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0));
    let _ = sdcard_spi.write(&[0;10]); // Send at least 74 cycles of the clock to the SD card. TODO: This should be moved in future, as if a card is hot-inserted it will miss this.
    sdcard_spi.set_baudrate(clocks.peripheral_clock.freq(), 25.MHz());
    sdcard_spi
}

fn configure_i2c(i2c0: I2C0, display_pins: DisplayPins, resets: &mut RESETS, clocks: &ClocksManager,) -> liquid_crystal::I2C<rp_pico::hal::I2C<I2C0, (DisplaySda, DisplayScl)>> {
    let display_i2c = I2C::i2c0(i2c0, display_pins.sda, display_pins.scl, 100.kHz(), resets, clocks.system_clock.freq());

    liquid_crystal::I2C::new(display_i2c, display::DISPLAY_I2C_ADDRESS)
}

fn configure_clocks(watchdog: WATCHDOG, xosc: XOSC, clocks: CLOCKS, pll_sys: PLL_SYS, pll_usb: PLL_USB, resets: &mut RESETS) -> (Watchdog, ClocksManager) {
    let mut watchdog = rp_pico::hal::Watchdog::new(watchdog);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        xosc,
        clocks,
        pll_sys,
        pll_usb,
        resets,
        &mut watchdog,
    ).ok()
     .unwrap();

    (watchdog, clocks)
}

fn configure_timers(pwm: PWM, timer: TIMER, resets: &mut RESETS, clocks: &ClocksManager) -> (Timer, pwm::Slice<pwm::Pwm0, FreeRunning>, Alarm0) {
    // PWM used as timer
    let slices = rp_pico::hal::pwm::Slices::new(pwm, resets);
    let mut sample_rate_timer = slices.pwm0.into_mode::<FreeRunning>();
    core::assert!(clocks.system_clock.freq().to_MHz() < 128); // Assume freq < 128MHz because prescaler is 8-bit
    sample_rate_timer.default_config();
    sample_rate_timer.set_div_int( (clocks.system_clock.freq().to_MHz() * 2) as u8 ); // 2us resolution => max ~130ms range
    sample_rate_timer.set_top(62_500); // Want 125ms range
    sample_rate_timer.enable_interrupt();

    // System timer, alarm 0 used to schedule sensor ready alerts
    let mut system_timer = rp_pico::hal::Timer::new(timer, resets, clocks);
    let mut sensors_ready_timer = system_timer.alarm_0().unwrap();
    sensors_ready_timer.enable_interrupt();

    (system_timer, sample_rate_timer, sensors_ready_timer)
}

fn configure_usb_bus(usbctrl_regs: USBCTRL_REGS, usbctrl_dpram: USBCTRL_DPRAM, resets: &mut RESETS, usb_clock: UsbClock) -> UsbBusAllocator<UsbBus> {
    UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        usbctrl_regs,
        usbctrl_dpram,
        usb_clock,
        true,
        resets,
    ))
}

fn configure_usb(usb_bus: &UsbBusAllocator<UsbBus>) -> (SerialPort<UsbBus>, UsbDevice<UsbBus>) {
    let usb_serial = SerialPort::new(usb_bus);

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    (usb_serial, usb_dev)
}