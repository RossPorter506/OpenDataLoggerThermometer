#![no_std]
#![no_main]
#![feature(variant_count)]
use core::{cell::RefCell, panic::PanicInfo, sync::atomic::{AtomicBool, AtomicU8, Ordering}};
extern crate alloc;
use alloc::string::String;

use board::Board;
use embedded_sdmmc::SdCardError;
use rp_pico::{
    entry,
    hal::{
        clocks::PeripheralClock,
        gpio, 
        pwm::{FreeRunning, Pwm0, Slice}, 
        timer::{Alarm, Alarm0}
    },
    pac::interrupt,
};
use critical_section::Mutex;
use sd_card::{SdManager, SdCardEvent};
use serial::MAX_SNAPSHOT_LEN;
use rtt_target::rprintln;
use arrayvec::ArrayString;
use atomic_enum::atomic_enum;

mod config; use config::{Config, Status::*};
mod constants; use constants::*;
mod pcb_mapping {include!("pcb_v1_mapping.rs");} 
mod lmt01;
mod display;
mod sd_card;
mod serial;
mod pio;
mod state_machine;
use state_machine::UpdateReason;
mod board;

/* TODO:
Determine sensible limits for open files/folders
Use serial printing only if configured for use, else RTT 
Test: Sensors, writing to SD card, serial
Figure out what errors should merely be printed versus those that should panic
Maybe wrap everything up into a nice struct
Stretch goal: Synchronise time with Pico W via NTP. Maybe not usable for us - cyw43 doesn't seem to support EAP WAP?
*/

#[panic_handler]
fn panic_handler(panic_info: &PanicInfo) -> ! {
    rprintln!("Panic: {:?}", panic_info);
    loop {}
}

#[entry]
fn main() -> ! {
    // High-level system configuration
    let mut config = Config::new();

    // PCB-specific configuration and interface
    let mut board = Board::configure(&mut config);
    
    let mut buffers = Buffers::new();

    rprintln!("Initialisation complete.");
    
    loop {
        // Whether we are going to update state and redraw screen
        let mut update_available: Option<UpdateReason> = None;

        // Monitor sensors
        if SENSOR_READINGS_AVAILABLE.take() {
            board.read_sensors(&mut buffers, &config);
            update_available = Some(UpdateReason::NewSensorValues);
        }
        
        if READY_TO_START_NEXT_READING.take() {
            board.start_next_sensor_reading();
        }

        // Check for button presses. This does override the `update_available` value from the above sensor readings, but
        // button presses always prompt a screen redraw anyway, so any new sensor values will be displayed regardless. 
        update_available = match BUTTON_STATE.take() {
            ButtonState::NextButton => Some(UpdateReason::NextButton),
            ButtonState::SelectButton => Some(UpdateReason::SelectButton),
            _ => update_available, // no change
        };

        // Check if the card is inserted or removed
        board.sd_manager = monitor_sdcard_state(board.sd_manager, &mut config, &mut update_available, &board.peripheral_clock);

        // Write to SD card if buffer is full
        if let Err(nb::Error::Other(e)) = board.manage_sd_writes(&mut buffers) {
            rprintln!("{:?}", e);
            update_available = Some(UpdateReason::SDError(e));
        }

        // Serial
        buffers.serial = board.manage_serial_comms(buffers.serial);

        // Service events and update available state, if required
        if let Some(update_reason) = update_available {
            config.process_events(&update_reason, board.sd_manager.get_card_info().is_ready());

            board.display_manager.determine_new_screen(&config, board.sd_manager.get_card_info(), buffers.display);
        }

        // Screen updates
        board.display_manager.incremental_update(&mut board.system_timer);
    }
}

/// Listen for an SD card being inserted or removed and initialise, if required
fn monitor_sdcard_state(mut sd_manager: SdManager, config: &mut Config, update_available: &mut Option<UpdateReason>, peripheral_clock: &PeripheralClock) -> SdManager {
    let mut sd_status: Result<(), embedded_sdmmc::Error<SdCardError>> = Ok(());

    match sd_manager.get_physical_card_events() {
        SdCardEvent::WasJustInserted => {
            *update_available = Some(UpdateReason::SDStateChange);
            sd_manager.initialise_card(peripheral_clock);
        },
        SdCardEvent::WasJustRemoved => {
            if !sd_manager.is_safe_to_remove() {
                // SD card removed unexpectedly
                // Move to SD card error screen
                *update_available = Some(UpdateReason::SDRemovedUnexpectedly);
                sd_manager = sd_manager.reset_after_unexpected_removal();
            }
            else {
                *update_available = Some(UpdateReason::SDStateChange);
            }
        },
        SdCardEvent::NoChange => (),
    }

    // Beginning to datalog
    if config.status == SamplingAndDatalogging && config.sd.selected_for_use && !sd_manager.ready_to_write() {
        let filename = alloc::format!("{}.{}", String::from_utf8_lossy(&config.sd.filename).trim(), config.sd.filetype.as_str());
        sd_status = sd_manager.try_open_file(&filename); // TODO: Ensure this function makes ready_to_write() return true.
    }
    // Just stopped datalogging
    else if config.status != SamplingAndDatalogging && config.sd.selected_for_use && !sd_manager.is_safe_to_remove() {
        // We don't need to write to the card if we're not datalogging, so make it safe to remove just in case the user removes it.
        sd_status = sd_manager.prepare_for_removal(); // TODO: Ensure this function the is_safe_to_remove() return true.
        *update_available = Some(UpdateReason::SDSafeToRemove);
    }

    if let Err(e) = sd_status {
        rprintln!("{:?}", e);
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

/// Whether it's time to read the sensors for new values
static SENSOR_READINGS_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Timer used to tell when sensor readings are ready to be read.
static SENSORS_READY_TIMER: Mutex<RefCell< Option<Alarm0> >> = Mutex::new(RefCell::new(None)); 

// Set flag indicating we should check sensor values
// Interrupt fires 105ms after sensors are reset, values should be ready now.
#[interrupt]
fn TIMER_IRQ_0() {
    SENSOR_READINGS_AVAILABLE.set(true);
    critical_section::with(|cs| {
        let Some(ref mut timer) = *SENSORS_READY_TIMER.borrow_ref_mut(cs) else {unreachable!()};
        timer.clear_interrupt();
    });
}

/// The default sample frequency of the system on startup, in Hz.
const DEFAULT_SAMPLE_FREQ_HZ: u8 = 1;

/// Timer used to count out multiples of 125ms periods, used to determine when new readings should be taken.
static SAMPLE_RATE_TIMER: Mutex<RefCell< Option<Slice<Pwm0, FreeRunning>> >> = Mutex::new(RefCell::new(None)); 

/// How many times the 8Hz PWM interrupt must trigger per sample of the sensors, e.g.
/// 
/// `WRAPS_PER_SAMPLE = 8/n` where `n` is the sampling frequency
static WRAPS_PER_SAMPLE: AtomicU8 = AtomicU8::new(8/DEFAULT_SAMPLE_FREQ_HZ);

/// Whether it's time to reset the sensors to generate new values
static READY_TO_START_NEXT_READING: AtomicBool = AtomicBool::new(false);
// Interrupt fires some multiple of 125ms (based on sample rate). Used to schedule sensor resets
#[interrupt]
fn PWM_IRQ_WRAP() {
    static NUM_WRAPS: AtomicU8 = AtomicU8::new(1);
    
    if NUM_WRAPS.get() >= WRAPS_PER_SAMPLE.get() {
        READY_TO_START_NEXT_READING.set(true);
        NUM_WRAPS.set(1);
    }
    else { NUM_WRAPS.set(NUM_WRAPS.get()+1); }

    critical_section::with(|cs| {
        let Some(ref mut timer) = *SAMPLE_RATE_TIMER.borrow_ref_mut(cs) else { unreachable!() };
        timer.clear_interrupt();
    });
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
        let Some(ref mut buttons) = *BUTTON_PINS.borrow_ref_mut(cs) else { unreachable!() };
            if buttons.select.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.set(ButtonState::SelectButton);
                buttons.select.clear_interrupt(gpio::Interrupt::EdgeLow);
            } else if buttons.next.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.set(ButtonState::NextButton);
                buttons.next.clear_interrupt(gpio::Interrupt::EdgeLow);
        }
    });
}

/// Whether buttons have been pressed
#[atomic_enum]
#[derive(PartialEq, Default)]
enum ButtonState {
    #[default]
    None = 0,
    NextButton,
    SelectButton,
}

/// Convenience trait for relaxed atomic reads/writes.
trait RelaxedIO {
    type Inner;
    /// Gets the inner value, relaxed ordering
    fn get(&self) -> Self::Inner;
    /// Gets the inner value and replaces it with the default, relaxed ordering.
    fn take(&self) -> Self::Inner;
    /// Sets the inner value, relaxed ordering
    fn set(&self, val: Self::Inner);
}
macro_rules! impl_relaxedio_for {
    ($type: ty, $inner: ty) => {
        impl RelaxedIO for $type {
            type Inner = $inner;
            fn get(&self) -> Self::Inner { self.load(Ordering::Relaxed) }
            fn set(&self, val: Self::Inner) { self.store(val, Ordering::Relaxed); }
            fn take(&self) -> Self::Inner{ 
                let val = self.load(Ordering::Relaxed); 
                self.store(Default::default(), Ordering::Relaxed);
                val
            }
        }
    };
}
impl_relaxedio_for!(AtomicU8, u8);
impl_relaxedio_for!(AtomicBool, bool);
impl_relaxedio_for!(AtomicButtonState, ButtonState);