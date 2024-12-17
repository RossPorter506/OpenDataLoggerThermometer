#![no_std]
#![no_main]
#![feature(variant_count)]
use core::{cell::RefCell, panic::PanicInfo, str, sync::atomic::{AtomicBool, AtomicU8, Ordering}};
extern crate alloc;
use alloc::string::String;

use embedded_sdmmc::SdCardError;
use liquid_crystal::DelayNs;
use rp_pico::{
    entry,
    hal::{
        clocks::{ClocksManager, PeripheralClock, RtcClock, SystemClock, UsbClock}, Clock, Watchdog,
        fugit::{ExtU32, RateExtU32}, gpio, 
        pwm::{self, FreeRunning, Pwm0, Slice}, 
        rtc::{DateTime, DayOfWeek, RealTimeClock}, spi, Spi,
        timer::{Alarm, Alarm0}, Timer,
        usb::UsbBus, Sio, I2C
    },
    pac::{self, interrupt, CLOCKS, I2C0, IO_BANK0, PADS_BANK0, PIO0, PIO1, PLL_SYS, PLL_USB, PWM, RESETS, RTC, SIO, SPI0, TIMER, USBCTRL_DPRAM, USBCTRL_REGS, WATCHDOG, XOSC},
};
use critical_section::Mutex;
use sd_card::{SdCardInfo, SdManager, SdCardEvent};
use serial::{nonblocking_read, MAX_SNAPSHOT_LEN};
use static_cell::StaticCell;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use rtt_target::{rtt_init_print, rprintln};
use arrayvec::ArrayString;
use atomic_enum::atomic_enum;

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
    DatalogErrorSDUnexpectedRemovalSelectables as DlogSdRemovSel, 
    DatalogSDSafeToRemoveSelectables as DlogSafeRemovSel, 
    DatalogSDWritingSelectables as DlogSdWrtSel, 
    DatalogSDErrorSelectables as DlogSdErrSel, 
    DatalogTemperaturesSelectables as DlogTempSel, 
    MainmenuSelectables::*, State::*, UpdateReason, 
    ViewTemperaturesSelectables as ViewTemp
};

/* TODO:
Use serial printing only if configured for use, else RTT 
Test: Basically everything
Figure out what errors should merely be printed versus those that should panic
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

#[panic_handler]
fn panic_handler(panic_info: &PanicInfo) -> ! {
    rprintln!("Panic: {:?}", panic_info);
    loop {}
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    configure_heap();
    // Take ownership of peripherals
    let Some(mut registers) = pac::Peripherals::take() else { unreachable!() };
    
    // GPIO pin groups
    let (temp_power, temp_sense, sdcard_pins, display_pins) = collect_pins(registers.SIO, registers.IO_BANK0, registers.PADS_BANK0, &mut registers.RESETS);

    // System clocks
    let (_watchdog, clocks) = configure_clocks(registers.WATCHDOG, registers.XOSC, registers.CLOCKS, registers.PLL_SYS, registers.PLL_USB, &mut registers.RESETS);

    // Timers
    let (mut system_timer, mut sample_rate_timer, mut sensors_ready_timer) = configure_timers(registers.PWM, registers.TIMER, &mut registers.RESETS, &clocks);

    // USB - Also enables USB serial printing
    let mut usb_device = configure_usb(registers.USBCTRL_REGS, registers.USBCTRL_DPRAM, &mut registers.RESETS, clocks.usb_clock);

    // System configuration
    let mut config = Config::new();

    // SD card manager
    let mut sd_manager = configure_sdcard(registers.SPI0, registers.RTC, sdcard_pins, clocks.rtc_clock, system_timer, &mut registers.RESETS, &clocks.peripheral_clock);
    // Write controls for SD card. false when transmissions are complete, for now
    let mut write_to_sd = false;

    // Display
    let mut display_manager = configure_display(registers.I2C0, display_pins, sd_manager.get_card_info(), &config, &clocks.system_clock, &mut system_timer, &mut registers.RESETS);

    // Temp sensors
    let mut temp_sensors = configure_sensors(registers.PIO0, registers.PIO1, temp_sense, temp_power, &mut registers.RESETS);
    
    // Buffers
    let mut buffers = Buffers::new();

    // Autodetect any connected sensors
    config.enabled_channels = temp_sensors.autodetect_sensor_channels(&mut system_timer);

    rprintln!("Initialisation complete.");
    
    loop {
        // Whether we are going to update state and redraw screen
        let mut update_available: Option<UpdateReason> = None;

        // Monitor sensors
        if SENSOR_READINGS_AVAILABLE.get() {
            read_sensors(&mut temp_sensors, &mut buffers, &config, &mut sample_rate_timer, &mut write_to_sd, &mut system_timer);
            update_available = Some(UpdateReason::NewSensorValues);
            SENSOR_READINGS_AVAILABLE.set(false);
        }
        
        if READY_TO_START_NEXT_READING.get() {
            start_next_sensor_reading(&mut temp_sensors, &mut sensors_ready_timer);
            READY_TO_START_NEXT_READING.set(false);
        }

        // Check for button presses. This does override the `update_available` value from the above sensor readings, but
        // button presses always prompt a screen redraw anyway, so any new sensor values will be displayed regardless. 
        update_available = match BUTTON_STATE.get() {
            ButtonState::NextButton => Some(UpdateReason::NextButton),
            ButtonState::SelectButton => Some(UpdateReason::SelectButton),
            _ => update_available, // no change
        };
        BUTTON_STATE.set(ButtonState::None);

        // Check if the card is inserted or removed
        sd_manager = monitor_sdcard_state(sd_manager, &mut config, &mut update_available, &clocks.peripheral_clock);

        // Write to SD card if buffer is full
        if write_to_sd {
            match sd_manager.write_bytes(buffers.spi.as_bytes()) {
                Ok(_) => {
                    buffers.spi.clear();
                    write_to_sd = false;
                },
                Err(e) => {
                    eprintln!("{e:?}");
                    update_available = Some(UpdateReason::SDError(e));
                },
            };
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
fn monitor_sdcard_state(mut sd_manager: SdManager, config: &mut Config, update_available: &mut Option<UpdateReason>, peripheral_clock: &PeripheralClock) -> SdManager {
    let mut sd_status: Result<(), embedded_sdmmc::Error<SdCardError>> = Ok(());

    match sd_manager.get_physical_card_events() {
        SdCardEvent::WasJustInserted => {
            sd_manager.initialise_card(peripheral_clock);
        },
        SdCardEvent::WasJustRemoved => {
            if !sd_manager.is_safe_to_remove() {
                // SD card removed unexpectedly
                // Move to SD card error screen
                *update_available = Some(UpdateReason::SDRemovedUnexpectedly);
                sd_manager = sd_manager.reset_after_unexpected_removal();
            }
        },
        SdCardEvent::NoChange => (),
    }

    // Beginning to datalog
    if config.status == SamplingAndDatalogging && config.sd.selected_for_use && !sd_manager.ready_to_write() {
        let filename = alloc::format!("{}.{}", String::from_utf8_lossy(&config.sd.filename), config.sd.filetype.as_str());
        sd_status = sd_manager.try_open_file(&filename); // TODO: Ensure this function makes ready_to_write() return true.
    }
    // Just stopped datalogging
    else if config.status != SamplingAndDatalogging && config.sd.selected_for_use && !sd_manager.is_safe_to_remove() {
        // We don't need to write to the card if we're not datalogging, so make it safe to remove just in case the user removes it.
        sd_status = sd_manager.prepare_for_removal(); // TODO: Ensure this function the ready_to_remove() return true.
    }

    if let Err(e) = sd_status {
        eprintln!("{e:?}");
        *update_available = Some(UpdateReason::SDError(e));
        config.sd.selected_for_use = false;
    }
    sd_manager
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
    sensors_ready_timer.schedule((lmt01::SENSOR_MAX_TIME_FOR_READING_MS+1).millis()).unwrap_or_else(|_| unreachable!());
    READY_TO_START_NEXT_READING.set(false);
}

/// Try to send data over serial.
fn manage_serial_comms(usb_device: &mut UsbDevice<UsbBus>, serial_buffer: ArrayString::<MAX_SNAPSHOT_LEN>) -> ArrayString::<MAX_SNAPSHOT_LEN>{
    // Has something been sent or received since last time?
    let new_events = critical_section::with(|cs| -> bool {
        let Some(ref mut serial) = *serial::USB_SERIAL.borrow_ref_mut(cs) else { unreachable!() };
        usb_device.poll(&mut [serial])
    });

    if new_events {
        // Read the serial buffer, check for ping requests
        check_for_and_respond_to_pings();

        // Try to send everything we need to send
        if !serial_buffer.is_empty() {
            use serial::UsbSerialPrintError::*;
            match serial::nonblocking_print(serial_buffer.as_bytes()) {
                // Remove whatever was successfully sent from our buffer
                Err(WouldBlock(len)) => return ArrayString::from(&serial_buffer[len..]).unwrap_or_else(|_| unreachable!() ),
                Err(OtherError(a)) => panic!("{a:?}"),
                Ok(()) => return serial_buffer,
            };
        }
    }
    
    serial_buffer
}

/// Reads the serial recieve buffer. If we receive an ASCII ENQ character respond with an ASCII ACK.
fn check_for_and_respond_to_pings() {
    let mut buf = [0u8; 8];
    const ASCII_ENQ_CHAR: u8 = b'\x05';
    const ASCII_ACK_STR: &str = "\x06";
    use serial::UsbSerialReadError::*;
    match nonblocking_read(buf.as_mut_slice()) {
        Err(WouldBlock) => (), // Recieve buffer empty
        Ok(len) => {
            let enq_count = buf.iter().take(len).filter(|&&c| c == ASCII_ENQ_CHAR).count();
            let response = str::repeat(ASCII_ACK_STR, enq_count);
            println!("{response}"); // This is technically a blocking write, but...
        },
        Err(OtherError(e)) => panic!("{e:?}"),
    }
}

/// Update system configuration according to button presses.
/// 
/// This function handles Mealy-style changes that depend on specific transitions. Next state transitions and Moore-style 'state-only' outputs handled elsewhere.
fn service_button_event(config: &mut Config, update_reason: &UpdateReason) {
    if *update_reason != UpdateReason::SelectButton { return }
    
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
        
        DatalogSDError(DlogSdErrSel::ContinueWithoutSD)                 => (),
        DatalogSDError(DlogSdErrSel::StopDatalogging)                   => (),
    };
}
/// Cycle through alphanumeric ASCII values.
/// a -> b -> ... z -> 0 -> ... 9 -> ' ' ->  a
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
                buttons.select.clear_interrupt(gpio::Interrupt::EdgeLow);
            } else if buttons.next.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.set(ButtonState::NextButton);
                buttons.next.clear_interrupt(gpio::Interrupt::EdgeLow);
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
fn configure_button_pins(mut select: pcb_mapping::SelectButton, mut next: pcb_mapping::NextButton) {
    next.set_schmitt_enabled(true);
    next.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    next.clear_interrupt(gpio::Interrupt::EdgeLow);

    select.set_schmitt_enabled(true);
    select.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    select.clear_interrupt(gpio::Interrupt::EdgeLow);

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
fn configure_spi(spi0: SPI0, sdcard_pins: SdCardSPIPins, resets: &mut RESETS, peripheral_clock: &PeripheralClock) -> rp_pico::hal::Spi<spi::Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), BITS_PER_SPI_PACKET> {
    let sdcard_spi = Spi::<_,_,_,BITS_PER_SPI_PACKET>::new(spi0, (sdcard_pins.mosi, sdcard_pins.miso, sdcard_pins.sck));
    
    // Start at 400kHz before negotiation, then 25MHz after.
    sdcard_spi.init(resets, peripheral_clock.freq(), sd_card::SDCARD_INITIAL_FREQ_KHZ.kHz(), spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0))
}

fn configure_i2c(i2c0: I2C0, display_pins: DisplayPins, resets: &mut RESETS, system_clock: &SystemClock,) -> liquid_crystal::I2C<rp_pico::hal::I2C<I2C0, (DisplaySda, DisplayScl)>> {
    const I2C_FREQ_KHZ: u32 = 1000; // Depending on your display you may have to set this lower, to 400 or 100kHz.
    let display_i2c = I2C::i2c0(i2c0, display_pins.sda, display_pins.scl, I2C_FREQ_KHZ.kHz(), resets, system_clock.freq());

    liquid_crystal::I2C::new(display_i2c, display::DISPLAY_I2C_ADDRESS)
}

fn configure_clocks(watchdog: WATCHDOG, xosc: XOSC, clocks: CLOCKS, pll_sys: PLL_SYS, pll_usb: PLL_USB, resets: &mut RESETS) -> (Watchdog, ClocksManager) {
    let mut watchdog = rp_pico::hal::Watchdog::new(watchdog);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ, xosc, clocks,
        pll_sys, pll_usb,
        resets, &mut watchdog,
    ).ok()
     .unwrap(); // TODO

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

fn configure_usb(usbctrl_regs: USBCTRL_REGS, usbctrl_dpram: USBCTRL_DPRAM, resets: &mut RESETS, usb_clock: UsbClock) -> UsbDevice<'static, UsbBus> {
    let usb_bus = UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        usbctrl_regs, usbctrl_dpram,
        usb_clock, true, resets,
    ));
    // In order to emulate the println! macros the USB bus reference must be 'static.
    // Move the USB bus into a StaticCell to get a 'static reference to it
    static USB_BUS: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
    let usb_bus_ref: &'static UsbBusAllocator<UsbBus> = USB_BUS.init(usb_bus); 

    // Generate and store USB serial port in static variable so we can debug print anywhere
    let usb_serial = SerialPort::new(usb_bus_ref);
    critical_section::with(|cs| {
        serial::USB_SERIAL.borrow(cs).replace(Some(usb_serial));
    });

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company") // TODO
            .product("Serial port") 
            .serial_number("TEST")]) // TODO
        .unwrap() // TODO
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    usb_dev
}

fn configure_sensors(pio0: PIO0, pio1: PIO1, temp_sense: TempSensePins, temp_power: TempPowerPins, resets: &mut RESETS) -> TempSensors {
    let pio_state_machines = lmt01::configure_pios_for_lmt01(pio0, pio1, resets, &temp_sense);
    TempSensors::new(temp_power, temp_sense, pio_state_machines)
}

fn configure_display(i2c0: I2C0, display_pins: DisplayPins, card_info: SdCardInfo, config: & Config, system_clock: & SystemClock, delay: & mut impl DelayNs, resets: & mut RESETS) -> IncrementalDisplayWriter<'static> {
    let i2c = configure_i2c(i2c0, display_pins, resets, system_clock);
    static I2C: StaticCell<liquid_crystal::I2C<rp_pico::hal::I2C<I2C0, (DisplaySda, DisplayScl)>>> = StaticCell::new();
    let i2c_static_ref = I2C.init(i2c);
    let mut display_manager = IncrementalDisplayWriter::new(config, card_info, delay, i2c_static_ref);
    display_manager.load_custom_chars(delay);
    display_manager
}

fn configure_sdcard(spi0: SPI0, rtc: RTC, sdcard_pins: SdCardPins, rtc_clock: RtcClock, delay: Timer, resets: &mut RESETS, peripheral_clock: &PeripheralClock) -> SdManager {
    let spi_bus = configure_spi(spi0, sdcard_pins.spi, resets, peripheral_clock);
    let rtc = RealTimeClock::new(rtc, rtc_clock, resets, DateTime{ year: 2024, month: 1, day: 1, day_of_week: DayOfWeek::Monday, hour: 1, minute: 1, second: 1 }).unwrap(); // TODO
    SdManager::new(spi_bus, sdcard_pins.cs, delay, sdcard_pins.extra, rtc)
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