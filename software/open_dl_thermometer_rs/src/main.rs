#![no_std]
#![no_main]
#![feature(variant_count)]
use core::{cell::RefCell, sync::atomic::{AtomicU8,AtomicBool, Ordering}};
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
use critical_section::Mutex;
use sd_card::SdManager;
use serial::MAX_SNAPSHOT_LEN;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use arrayvec::ArrayString;
use atomic_enum::atomic_enum;
use alloc::string::String;

mod config; use config::{Config, Status::*};
mod constants; use constants::*;
mod pcb_mapping {include!("pcb_v1_mapping.rs");} 
use pcb_mapping::{ButtonPins, DisplayPins, DisplayScl, DisplaySda, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardPins, SdCardSPIPins, SdCardSck, TempPowerPins, TempSensePins};
mod lmt01; use lmt01::TempSensors;
mod display; use display::IncrementalDisplayWriter;
mod sd_card;
mod serial;
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
Display driver: Custom chars
SD card driver
Determine when SD card is safe to remove
Maybe wrap everything up into a nice struct
Stretch goal: Synchronise time with Pico W and NTP. Maybe not doable - cyw43 doesn't seem to support EAP WAP?
*/

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
    let Some(mut registers) = pac::Peripherals::take() else { unreachable!() };
    
    // GPIO pin groups
    let (temp_power, temp_sense, sdcard_pins, display_pins) = collect_pins(registers.SIO, registers.IO_BANK0, registers.PADS_BANK0, &mut registers.RESETS);

    // System clocks
    let (_watchdog, clocks) = configure_clocks(registers.WATCHDOG, registers.XOSC, registers.CLOCKS, registers.PLL_SYS, registers.PLL_USB, &mut registers.RESETS);

    // System configuration
    let mut config = Config::new();

    // Timers
    let (mut system_timer, mut sample_rate_timer, mut sensors_ready_timer) = configure_timers(registers.PWM, registers.TIMER, &mut registers.RESETS, &clocks);

    // I2C
    let mut i2c = configure_i2c(registers.I2C0, display_pins, &mut registers.RESETS, &clocks);

    // SPI
    let spi_bus = configure_spi(registers.SPI0, sdcard_pins.spi, &mut registers.RESETS, &clocks);

    // USB
    // WARNING: USB_SERIAL relies on usb_bus never being consumed. usb_bus MUST continue to be in scope for the lifetime of the program.
    let usb_bus = configure_usb_bus(registers.USBCTRL_REGS, registers.USBCTRL_DPRAM, &mut registers.RESETS, clocks.usb_clock);

    // Upgrade the lifetime of the USB bus reference to 'static so we can store USB_SERIAL in a static variable and print anywhere.
    // DO NOT DO THIS unless you understand exactly what this entails.
    // Safety: main never returns, so usb_bus (and thus our reference to it) is effectively static.
    // We are single-threaded so no references to the bus could outlive this thread. We also panic-abort, so no stack unwinding can occur.
    let static_usb_bus_ref: &'static UsbBusAllocator<UsbBus> = unsafe{ core::mem::transmute(&usb_bus) }; 
    let mut usb_device = configure_usb(static_usb_bus_ref);

    // RTC
    let rtc = rp_pico::hal::rtc::RealTimeClock::new(registers.RTC, clocks.rtc_clock, &mut registers.RESETS, rp_pico::hal::rtc::DateTime{ year: 2024, month: 1, day: 1, day_of_week: rp_pico::hal::rtc::DayOfWeek::Monday, hour: 1, minute: 1, second: 1 }).unwrap();

    // SD card manager
    let mut sd_manager = crate::sd_card::SdManager::new(spi_bus, sdcard_pins.cs, system_timer, sdcard_pins.extra, rtc);
    // What the insertion state of the SD card was in the previous loop. Used to compare for insertions/removals.
    let mut sd_prev_insert_state = false;

    // Display
    let mut display_manager = IncrementalDisplayWriter::new(&config, sd_manager.get_card_info(), &mut i2c);
    display_manager.load_custom_chars(&mut system_timer);

    // PIO and temp sensor controller
    let pio_state_machines = lmt01::configure_pios_for_lmt01(registers.PIO0, registers.PIO1, &mut registers.RESETS, &temp_sense);
    let mut temp_sensors = TempSensors::new(temp_power, temp_sense, pio_state_machines);

    // Write controls for SD card. false when transmissions are complete, for now
    let mut write_to_sd = false;

    // Buffers
    let mut buffers = Buffers::new();

    // Autodetect any connected sensors
    config.enabled_channels = temp_sensors.autodetect_sensor_channels(&mut system_timer);

    println!("Initialisation complete.");
    
    loop {
        // Whether we are going to update state and redraw screen
        let mut update_available: Option<UpdateReason> = None;

        // Monitor sensors
        if SENSOR_READINGS_AVAILABLE.get() {
            read_sensors(&mut temp_sensors, &mut buffers, &config, &mut sample_rate_timer, &mut write_to_sd, &mut system_timer);
            update_available = Some(UpdateReason::NewSensorValues);
        }
        
        if READY_TO_START_NEXT_READING.get() {
            start_next_sensor_reading(&mut temp_sensors, &mut sensors_ready_timer);
        }

        // Check for button presses. This does override the `update_available` value from the above sensor readings, but
        // button presses always prompt a screen redraw anyway, so any new sensor values will be displayed regardless. 
        update_available = match BUTTON_STATE.get() {
            ButtonState::NextButton => Some(UpdateReason::NextButton),
            ButtonState::SelectButton => Some(UpdateReason::SelectButton),
            _ => update_available, // no change
        };

        // Check if the card is inserted or removed
        monitor_sdcard_state(sd_prev_insert_state, &mut sd_manager, &mut config, &mut update_available, &clocks.peripheral_clock);
        sd_prev_insert_state = sd_manager.is_card_inserted();

        // Deal with SD card
        if write_to_sd {
            sd_manager.write_bytes(buffers.spi.as_bytes());
            write_to_sd = false;
        }

        // Serial
        buffers.serial = manage_serial_comms(&mut usb_device, buffers.serial);

        // Service events and update available state, if required
        if let Some(update_reason) = update_available {
            service_button_event(&mut config, &update_reason);
            state_machine::next_state(&mut config, &mut sd_manager, &update_reason);
            state_machine::state_outputs(&mut config);
            display_manager.determine_new_screen(&mut system_timer, &config, sd_manager.get_card_info(), buffers.display);
            if config.status == Idle { sample_rate_timer.disable(); } else { sample_rate_timer.enable(); }
        }

        // Screen updates
        display_manager.incremental_update(&mut system_timer);
    }
}

/// Listen for an SD card being inserted or removed and initialise, if required
fn monitor_sdcard_state(sd_was_inserted: bool, sd_manager: &mut crate::SdManager, config: &mut Config, update_available: &mut Option<UpdateReason>, peripheral_clock: &PeripheralClock) {
    let sd_just_removed = sd_was_inserted && !sd_manager.is_card_inserted();
    let sd_just_added = !sd_was_inserted && sd_manager.is_card_inserted();
    if sd_just_removed { // SD card removed

        if !sd_manager.is_safe_to_remove() {
            // SD card removed unexpectedly
            // Move to SD card error screen
            *update_available = Some(UpdateReason::SDRemovedUnexpectedly);
            sd_manager.reset_after_unexpected_removal();
        }
    }
    else if sd_just_added { // SD card inserted
        sd_manager.initialise_card(peripheral_clock);
    }

    // Beginning to datalog
    if config.status == SamplingAndDatalogging && !sd_manager.ready_to_write() {
        let filename = alloc::format!("{}.{}", String::from_utf8_lossy(&config.sd.filename), config.sd.filetype.as_str());
        sd_manager.open_file(&filename); // TODO: Ensure this function makes ready_to_write() return true.
    }
    // Just stopped datalogging
    else if config.status != SamplingAndDatalogging && !sd_manager.is_safe_to_remove() {
        // We don't need to write to the card if we're not datalogging, so make it safe to remove just in case the user removes it.
        sd_manager.prepare_for_removal(); // TODO: Ensure this function the ready_to_remove() return true.
    }
}

/// Stores values while they wait to be sent/used.
struct Buffers {
    /// Most recent sensor values, used by the display. ASCII
    display: display::DisplayValues,
    /// We store values until we're at least as large as the block size of the SD card block size. ASCII.
    spi: ArrayString::<{MAX_SNAPSHOT_LEN*11}>, // 11 is arbitrary. Gets us to 91*11 = 1001 bytes, with a 512 block size it's pretty close to 2 blocks
    /// We will write to serial as we receive, so buffer need only be big enough for one lot of readings. ASCII.
    serial: ArrayString::<MAX_SNAPSHOT_LEN>,
}
impl Buffers {
    pub fn new() -> Self {
        let display = [None; NUM_SENSOR_CHANNELS];
        let spi = ArrayString::new();
        let serial = ArrayString::new();
        Self {display, spi, serial}
    }
}

/// Read values from the LMT01 sensors and update buffers
fn read_sensors(temp_sensors: &mut TempSensors, 
        buffers: &mut Buffers,  
        config: &Config, sample_rate_timer: &mut Slice<Pwm0, FreeRunning>,
        write_to_sd: &mut bool,
        system_timer: &mut Timer) {

    buffers.display = temp_sensors.read_temperatures_and_end_conversion().map(lmt01::temp_to_string);
    
    // Technically this timestamp value should probably be from when the conversion *started*, not when it ended, but only 0.1sec difference
    let serialised_snapshot = serial::serialise_snapshot(system_timer.get_counter(), &buffers.display);
    
    if config.sd.selected_for_use {
        if buffers.spi.try_push_str(&serialised_snapshot).is_err() {
            eprintln!("SPI buffer overfull! Dropping data: {}", serialised_snapshot.as_str());
            *write_to_sd = true;
        }
        if buffers.spi.is_full() { *write_to_sd = true; }
    }

    if config.serial.selected_for_use {
        buffers.serial = serialised_snapshot;
    }

    SENSOR_READINGS_AVAILABLE.set(false);
    sample_rate_timer.clear_interrupt(); // Clear the interrupt flag for the 125ms timer. Should be done in the PWM interrupt, but this is good enough 
}

/// If it's time to get another reading then restart the LMT01 sensors.
fn start_next_sensor_reading(temp_sensors: &mut TempSensors, sensors_ready_timer: &mut Alarm0) {
    temp_sensors.begin_conversion();
    sensors_ready_timer.clear_interrupt(); // Clear the interrupt flag for the 105ms timer. Should be done in the TIMER0 interrupt, but this is good enough 
    let _ = sensors_ready_timer.schedule((lmt01::SENSOR_MAX_TIME_FOR_READING_MS+1).millis());
    READY_TO_START_NEXT_READING.set(false);
}

/// Try to send data over serial.
fn manage_serial_comms(usb_device: &mut UsbDevice<UsbBus>, serial_buffer: ArrayString::<MAX_SNAPSHOT_LEN>) -> ArrayString::<MAX_SNAPSHOT_LEN>{
    // We don't care what gets sent to us, but we need to poll anyway to stay USB compliant
    // Try to send everything we have
    let ready = critical_section::with(|cs| -> bool {
        let Some(ref mut serial) = *serial::USB_SERIAL.borrow_ref_mut(cs) else { unreachable!() };
        usb_device.poll(&mut [serial])
    });
    
    if !serial_buffer.is_empty() && ready {
        use serial::UsbSerialPrintError::*;
        match serial::nonblocking_print(serial_buffer.as_bytes()) {
            // Remove whatever was successfully sent from our buffer
            Err(WouldBlock(len)) => return ArrayString::from(&serial_buffer[len..]).unwrap_or_else(|_| unreachable!() ),
            Err(OtherError(a)) => panic!("{a:?}"),
            Ok(()) => return serial_buffer,
        };
    }
    serial_buffer
}

/// Update system configuration according to button presses.
/// 
/// This function handles Mealy-style changes that depend on specific transitions. Next state transitions and Moore-style 'state-only' outputs handled elsewhere.
fn service_button_event(config: &mut Config, update_reason: &UpdateReason) {
    if *update_reason != UpdateReason::SelectButton {
        return;
    }
    // If we press select while we have a configurable item selected we need to toggle it
    match &config.curr_state {
        Mainmenu(View)          => (),
        Mainmenu(Datalog)       => (),
        Mainmenu(Configure)     => (),

        ViewTemperatures(ViewTemp::None)        => (), 
        DatalogTemperatures(DlogTempSel::None)  => (),

        ConfigOutputs(Serial)                   => config.serial.selected_for_use ^= true,
        ConfigOutputs(SDCard)                   => config.sd.selected_for_use ^= true,
        ConfigOutputs(ConOutSel::Next)          => (),

        ConfigSDStatus(ConSdStatSel::Next)      => (),

        ConfigSDFilename(ConSDName::Next)       => (), // don't accidentally capture this case in the catch-all `char_num` pattern below.
        ConfigSDFilename(Filetype)              => config.sd.filetype = config.sd.filetype.next(),
        ConfigSDFilename(char_num)              => cycle_ascii_char(&mut config.sd.filename[*char_num as usize]), //filename characters 0-9

        ConfigChannelSelect(ConChanSel::Next)   => (), // don't accidentally capture this case in the catch-all `ch_num` pattern below.
        ConfigChannelSelect(ch_num)             => config.enabled_channels[*ch_num as usize] ^= true, // selected channels 1-8

        ConfigSampleRate(SampleRate)            => {cycle_sample_rate(&mut config.samples_per_sec); WRAPS_PER_SAMPLE.set(8/config.samples_per_sec);},
        ConfigSampleRate(ConRateSel::Next)      => (),

        DatalogConfirmStop(ConfirmStop)         => (),
        DatalogConfirmStop(CancelStop)          => (),

        DatalogErrorSDFull(ContinueWithoutSD)   => config.sd.selected_for_use = false,
        DatalogErrorSDFull(StopDatalogging)     => (),
        
        DatalogSDWriting(DlogSdWrtSel::None)    => (),

        DatalogSDSafeToRemove(DlogSafeRemovSel::Next)                   => (),

        DatalogSDUnexpectedRemoval(DlogSdRemovSel::ContinueWithoutSD)   => config.sd.selected_for_use = false,
        DatalogSDUnexpectedRemoval(DlogSdRemovSel::StopDatalogging)     => (),
    };
}
/// Cycle through alphanumeric ASCII values.
/// a -> b, ... z -> 0, ... 9 -> a
fn cycle_ascii_char(char: &mut u8) {
    *char = match *char {
        b'a'..=b'y' => *char + 1,
        b'z' => b'0',
        b'0'..=b'8' => *char + 1,
        b'9' => b' ',
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
    SENSOR_READINGS_AVAILABLE.set(true);
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
    
    if NUM_WRAPS.get() == WRAPS_PER_SAMPLE.get() {
        READY_TO_START_NEXT_READING.set(true);
        NUM_WRAPS.set(1);
    }
    else { NUM_WRAPS.set(NUM_WRAPS.get()+1); }
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
        if let Some(ref mut buttons) = *BUTTON_PINS.borrow_ref_mut(cs) {
            if buttons.select.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.set(ButtonState::SelectButton);
            } else if buttons.next.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.set(ButtonState::NextButton);
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
    let Some(mut sensors_ready_timer) = system_timer.alarm_0() else { unreachable!() };
    sensors_ready_timer.enable_interrupt();

    (system_timer, sample_rate_timer, sensors_ready_timer)
}

fn configure_usb_bus(usbctrl_regs: USBCTRL_REGS, usbctrl_dpram: USBCTRL_DPRAM, resets: &mut RESETS, usb_clock: UsbClock) -> UsbBusAllocator<UsbBus> {
    UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        usbctrl_regs, usbctrl_dpram,
        usb_clock, true, resets,
    ))
}

fn configure_usb(usb_bus: &'static UsbBusAllocator<UsbBus>) -> UsbDevice<'static, UsbBus> {
    // Generate and store USB serial port in static variable so we can debug print anywhere
    let usb_serial = SerialPort::new(usb_bus);
    critical_section::with(|cs| {
        serial::USB_SERIAL.borrow(cs).replace(Some(usb_serial));
    });

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company") // TODO
            .product("Serial port") 
            .serial_number("TEST")]) // TODO
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    usb_dev
}

/// Convenience trait for relaxed atomic reads/writes.
trait RelaxedIO {
    type Inner;
    /// Gets the inner value, relaxed ordering
    fn get(&self) -> Self::Inner;
    /// Sets the inner value, relaxed ordering
    fn set(&self, val: Self::Inner);
}
macro_rules! impl_relaxedio_for {
    ($type: ty, $inner: ty) => {
        impl RelaxedIO for $type {
            type Inner = $inner;
            fn get(&self) -> Self::Inner { self.load(Ordering::Relaxed) }
            fn set(&self, val: Self::Inner) { self.store(val, Ordering::Relaxed); }
        }
    };
}
impl_relaxedio_for!(AtomicU8, u8);
impl_relaxedio_for!(AtomicBool, bool);
impl_relaxedio_for!(AtomicButtonState, ButtonState);