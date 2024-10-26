#![no_std]
#![no_main]
#![feature(variant_count)]
use arrayvec::ArrayVec;
use core::cell::RefCell;
use critical_section::Mutex;
use panic_halt as _;

use rp_pico::{
    entry,
    hal::{gpio, Sio},
    pac::{self, interrupt},
};

use atomic_enum::atomic_enum;
use core::sync::atomic::{AtomicBool, Ordering};

mod config;
use config::{Config, Status::*};
mod constants;
use constants::*;
mod pcb_mapping {include!("pcb_v1_mapping.rs");} use pcb_mapping::{ButtonPins, DisplayPins, SDCardPins, TempPowerPins, TempSensePins};
mod lmt01; use lmt01::{TempSensors, CHARS_PER_READING};
mod display;
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

/*
struct SystemPeripherals {
    temp_sensors: TempSensors,
    display: Display,
    sd_card: SDCard,
    usb: USB,
}
impl SystemPeripherals {
}*/

#[entry]
fn main() -> ! {
    // Take ownership of peripherals and pins
    let mut peripherals = pac::Peripherals::take().unwrap();
    let sio = Sio::new(peripherals.SIO);
    let pins = rp_pico::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );
    let (mut temp_sensors, mut sd_card_pins, mut display_pins) = collect_pins(pins);
    
    let mut watchdog = rp_pico::hal::Watchdog::new(peripherals.WATCHDOG);
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    ).ok()
     .unwrap();

    /* Set up USB bus */
    let usb_bus = UsbBusAllocator::new(rp_pico::hal::usb::UsbBus::new(
        peripherals.USBCTRL_REGS,
        peripherals.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // System configuration
    let mut config = Config::new();

    // Whether we are going to update state and redraw screen
    let mut update_available: Option<UpdateReason> = None;

    // Write controls for SD and serial. false when transmissions are complete, for now
    let mut write_to_sd = false;
    let mut write_to_serial = false;

    // Buffers
    let mut sensor_values = [[0u8; CHARS_PER_READING]; NUM_SENSOR_CHANNELS]; // Most recent sensor values, to be written to display, serial, or SD card
    let mut serial_buffer = ArrayVec::<u8, {CHARS_PER_READING*NUM_SENSOR_CHANNELS}>::new(); // We will write to serial as we receive, so buffer need only be big enough for one lot of readings. ASCII.
    let mut spi_buffer = ArrayVec::<u8, {16*CHARS_PER_READING*NUM_SENSOR_CHANNELS} /* Arbitrary */>::new(); // We store values until we're some multiple of the SD card block size. ASCII.

    loop {
        // Check for button presses
        match BUTTON_STATE.load(Ordering::Relaxed) {
            ButtonState::None => (),
            ButtonState::Next => update_available = Some(UpdateReason::NextButton),
            ButtonState::Select => update_available = Some(UpdateReason::SelectButton),
        }

        // Read sensor values
        if READY_TO_READ_SENSORS.load(Ordering::Relaxed) {
            sensor_values = temp_sensors.read_temperatures().map(lmt01::temp_to_string);
            temp_sensors.power.pulse();
            //set_timer(config.samples_per_sec);
            if config.sd.enabled{
                spi_buffer.extend(sensor_values.clone().into_iter().flatten());
                if spi_buffer.is_full() { write_to_sd = true; }
            }
            
            if config.serial.enabled {
                serial_buffer = sensor_values.clone().into_iter().flatten().collect();
                write_to_serial = true;
            }

            READY_TO_READ_SENSORS.store(false, Ordering::Relaxed);
            update_available = Some(UpdateReason::NewSensorValues);
            todo!();
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
            else if usb_dev.poll(&mut [&mut serial]) {
                // We don't care what gets sent to us, but we need to poll anyway to stay USB compliant
                match serial.write(serial_buffer.as_slice()) {
                    Ok(len) => { serial_buffer.drain(0..len); }
                    // On error, just drop unwritten data.
                    // Err(WouldBlock) implies buffer is full.
                    Err(UsbError::WouldBlock) => (),
                    Err(a) => panic!("{a:?}"),
                };
            }
        }

        // Service events and update available state, if required
        if let Some(update_reason) = update_available {
            service_event(&mut config, &update_reason);
            state_machine::next_state(&mut config, &update_reason);
            display::update_display(&config, &sensor_values);
            update_available = None;
        }
    }
}

// Figure out what to do when the select button is pressed, based on the current state and what is currently selected
fn service_event(config: &mut Config, update_reason: &UpdateReason) {
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

        ConfigSDFilename(ConSDName::Next) => (),
        ConfigSDFilename(Filetype) => config.sd.filetype = config.sd.filetype.next(),
        ConfigSDFilename(char_num) => cycle_ascii_char(&mut config.sd.filename[*char_num as usize]), //filename characters 0-9

        ConfigChannelSelect(ConChanSel::Next) => (),
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
    let orig = char.clone();
    *char = match orig {
        b'a'..=b'y' => orig + 1,
        b'z' => b'0',
        b'0'..=b'8' => orig + 1,
        _ => b'a',
    };
}
/// Cycle between 1 - 9
fn cycle_sample_rate(rate: &mut u8) {
    let orig = rate.clone();
    *rate = match orig {
        1..=8 => orig + 1,
        _ => 1,
    }
}

/// Whether it's time to read the sensors for new values
static READY_TO_READ_SENSORS: AtomicBool = AtomicBool::new(false);
// Set flag indicating we should check sensor values
#[interrupt]
fn TIMER_IRQ_0() {
    READY_TO_READ_SENSORS.store(true, Ordering::Relaxed);
}

/// Interrupt-accessible container for button pins. 'None' until interrupt is configured by `configure_button_pins`.
static BUTTON_PINS: Mutex<RefCell<Option<pcb_mapping::ButtonPins>>> = Mutex::new(RefCell::new(None));
// Next and Select button interrupts. Set flags for main process. 
// We don't really care if both buttons are pressed at once. Hopefully the main loop should be tight enough not to matter
#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        if let Some(buttons) = BUTTON_PINS.borrow(cs).take() {
            if buttons.select.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.store(ButtonState::Select, Ordering::Relaxed);
            } else if buttons.next.interrupt_status(gpio::Interrupt::EdgeLow) {
                BUTTON_STATE.store(ButtonState::Next, Ordering::Relaxed);
            }
        }
    });
}

/// Whether buttons have been pressed
#[atomic_enum]
#[derive(PartialEq)]
enum ButtonState {
    None = 0,
    Next,
    Select,
}
/// Whether any buttons have been pressed
static BUTTON_STATE: AtomicButtonState = AtomicButtonState::new(ButtonState::None);

/// Collect individual pins and return structs of related pins, configured for use.
fn collect_pins(pins: rp_pico::Pins) -> (TempSensors, SDCardPins, DisplayPins) {
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
    let temp_sensors = TempSensors::new(temp_power, temp_sense);
    let sd_card_pins = SDCardPins {
        mosi: pins.gpio19.reconfigure(),
        miso: pins.gpio16.reconfigure(),
        sck: pins.gpio18.reconfigure(),
        cs: pins.gpio17.reconfigure(),
        write_protect: pins.gpio22.reconfigure(),
        card_detect: pins.gpio26.reconfigure(),
    };
    let display_pins = DisplayPins {
        sck: pins.gpio20.reconfigure(),
        scl: pins.gpio21.reconfigure(),
    };
    configure_button_pins(pins.gpio27.reconfigure(), pins.gpio28.reconfigure());
    (temp_sensors, sd_card_pins, display_pins)
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
