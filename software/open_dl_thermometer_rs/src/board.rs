// Does top-level configuration of board-specific parameters

use crate::{
    config::Config, 
    display::IncrementalDisplayWriter, eprintln, lmt01::{self, TempSensors}, 
    pcb_mapping::{self, ButtonPins, DisplayPins, DisplayScl, DisplaySda, SdCardExtraPins, SdCardMiso, SdCardMosi, SdCardPins, SdCardSPIPins, SdCardSck, TempPowerPins, TempSensePins}, 
    println, 
    sd_card::{self, SdCardInfo, SdManager}, 
    serial::{self, nonblocking_read, MAX_SNAPSHOT_LEN}, Buffers, RelaxedIO};

use arrayvec::ArrayString;
use embedded_sdmmc::SdCardError;
use liquid_crystal::DelayNs;
use rp_pico::{
    hal::{clocks::{ClocksManager, PeripheralClock, RtcClock, SystemClock, UsbClock}, 
    fugit::{ExtU32, RateExtU32}, gpio, pwm::FreeRunning, 
    rtc::{DateTime, DayOfWeek, RealTimeClock}, spi, timer::Alarm, usb::UsbBus, Clock, Sio, Spi, Timer, Watchdog, I2C}, 
    pac::{self, CLOCKS, I2C0, IO_BANK0, PADS_BANK0, PIO0, PIO1, PLL_SYS, PLL_USB, PWM, RESETS, RTC, SIO, SPI0, TIMER, USBCTRL_DPRAM, USBCTRL_REGS, WATCHDOG, XOSC},};
use rtt_target::rtt_init_print;

pub struct Board {
    pub system_timer: Timer,
    pub usb_device: UsbDevice<'static, UsbBus>,
    pub sd_manager: SdManager,
    write_to_sd: bool,
    pub display_manager: IncrementalDisplayWriter<'static>,
    pub temp_sensors: TempSensors,
    pub peripheral_clock: PeripheralClock,
}
impl Board {
    pub fn configure(config: &mut Config) -> Self {
        rtt_init_print!();
        configure_heap();
        // Take ownership of peripherals
        let Some(mut registers) = pac::Peripherals::take() else { unreachable!() };
        
        // GPIO pin groups
        let (temp_power, temp_sense, sdcard_pins, display_pins) = collect_pins(registers.SIO, registers.IO_BANK0, registers.PADS_BANK0, &mut registers.RESETS);

        // System clocks
        let (_watchdog, clocks) = configure_clocks(registers.WATCHDOG, registers.XOSC, registers.CLOCKS, registers.PLL_SYS, registers.PLL_USB, &mut registers.RESETS);

        // Timers
        let mut system_timer = configure_timers(registers.PWM, registers.TIMER, &mut registers.RESETS, &clocks);

        // USB - Also enables USB serial printing
        let usb_device = configure_usb(registers.USBCTRL_REGS, registers.USBCTRL_DPRAM, &mut registers.RESETS, clocks.usb_clock);

        // SD card manager
        let mut sd_manager = configure_sdcard(registers.SPI0, registers.RTC, sdcard_pins, clocks.rtc_clock, system_timer, &mut registers.RESETS, &clocks.peripheral_clock);
        // Write controls for SD card. false when transmissions are complete, for now
        let write_to_sd = false;

        // Display
        let display_manager = configure_display(registers.I2C0, display_pins, sd_manager.get_card_info(), config, &clocks.system_clock, &mut system_timer, &mut registers.RESETS);

        // Temp sensors
        let mut temp_sensors = configure_sensors(registers.PIO0, registers.PIO1, temp_sense, temp_power, &mut registers.RESETS);

        // Autodetect any connected sensors
        config.enabled_channels = temp_sensors.autodetect_sensor_channels(&mut system_timer);

        Board {system_timer, usb_device, sd_manager, write_to_sd, display_manager, temp_sensors, peripheral_clock: clocks.peripheral_clock}
    }

    /// Read values from the LMT01 sensors and update buffers
    pub fn read_sensors(&mut self, buffers: &mut Buffers, config: &Config) {

        buffers.display = self.temp_sensors.read_temperatures_and_end_conversion().map(lmt01::temp_to_string);

        // TODO: Technically this timestamp value should probably be from when the conversion *started*, not when it ended, but only 0.1sec difference
        let serialised_snapshot = serial::serialise_snapshot(self.system_timer.get_counter(), &buffers.display);

        if config.sd.selected_for_use {
            if buffers.spi.try_push_str(&serialised_snapshot).is_err() {
                eprintln!("SPI buffer overfull! Dropping data: {}", serialised_snapshot.as_str());
                self.write_to_sd = true;
            }
            if buffers.spi.is_full() { self.write_to_sd = true; }
        }

        if config.serial.selected_for_use {
            buffers.serial = serialised_snapshot;
        }

        crate::SENSOR_READINGS_AVAILABLE.set(false);
    }

    /// If it's time to get another reading then restart the LMT01 sensors.
    pub fn start_next_sensor_reading(&mut self) {
        self.temp_sensors.begin_conversion();
        critical_section::with(|cs| {
            let Some(ref mut timer) = *crate::SENSORS_READY_TIMER.borrow_ref_mut(cs) else { unreachable!() };
            timer.schedule((lmt01::SENSOR_MAX_TIME_FOR_READING_MS+1).millis()).unwrap_or_else(|_| unreachable!());
        });
        crate::READY_TO_START_NEXT_READING.set(false);
    }


    /// Try to send data over serial.
    pub fn manage_serial_comms(&mut self, serial_buffer: ArrayString::<MAX_SNAPSHOT_LEN>) -> ArrayString::<MAX_SNAPSHOT_LEN>{
        // Has something been sent or received since last time?
        let new_events = critical_section::with(|cs| -> bool {
            let Some(ref mut serial) = *serial::USB_SERIAL.borrow_ref_mut(cs) else { unreachable!() };
            self.usb_device.poll(&mut [serial])
        });

        if new_events {
            // Read the serial buffer, check for ping requests
            Self::check_for_and_respond_to_pings();

            // Try to send everything we need to send
            if !serial_buffer.is_empty() {
                use serial::UsbSerialPrintError::*;
                return match serial::nonblocking_print(serial_buffer.as_bytes()) {
                    // Remove whatever was successfully sent from our buffer
                    Err(WouldBlock(len)) => ArrayString::from(&serial_buffer[len..]).unwrap_or_else(|_| unreachable!() ),
                    Err(OtherError(a)) => panic!("{a:?}"),
                    Ok(()) => ArrayString::new(),
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

    /// If the SD card buffer is full, write the contents to the card.
    /// 
    /// Returns WouldBlock if the buffer isn't yet full.
    pub fn manage_sd_writes(&mut self, buffers: &mut Buffers) -> nb::Result<(), embedded_sdmmc::Error<SdCardError>> {
        if self.write_to_sd {
            return match self.sd_manager.write_bytes(buffers.spi.as_bytes()) {
                Ok(_) => {
                    buffers.spi.clear();
                    self.write_to_sd = false;
                    Ok(())
                },
                Err(e) => Err(nb::Error::Other(e)),
            };
        }
        Err(nb::Error::WouldBlock)
    }
}

// Heap so we can use Vec, etc.. Try to use ArrayVec where possible though.
use embedded_alloc::LlffHeap as Heap;
use static_cell::StaticCell;
use usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid}};
use usbd_serial::SerialPort;
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
fn configure_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024; // TODO
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
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
        crate::BUTTON_PINS
            .borrow_ref_mut(cs)
            .replace(ButtonPins { select, next });
    });
    // Enable interrupt
    unsafe { pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0); }
}

pub const BITS_PER_SPI_PACKET: u8 = 8;
fn configure_spi(spi0: SPI0, sdcard_pins: SdCardSPIPins, resets: &mut RESETS, peripheral_clock: &PeripheralClock) -> rp_pico::hal::Spi<spi::Enabled, SPI0, (SdCardMosi, SdCardMiso, SdCardSck), BITS_PER_SPI_PACKET> {
    let sdcard_spi = Spi::<_,_,_,BITS_PER_SPI_PACKET>::new(spi0, (sdcard_pins.mosi, sdcard_pins.miso, sdcard_pins.sck));
    
    // Start at 400kHz before negotiation, then 25MHz after.
    sdcard_spi.init(resets, peripheral_clock.freq(), sd_card::SDCARD_INITIAL_FREQ_KHZ.kHz(), spi::FrameFormat::MotorolaSpi(embedded_hal::spi::MODE_0))
}

fn configure_i2c(i2c0: I2C0, display_pins: DisplayPins, resets: &mut RESETS, system_clock: &SystemClock,) -> liquid_crystal::I2C<rp_pico::hal::I2C<I2C0, (DisplaySda, DisplayScl)>> {
    const I2C_FREQ_KHZ: u32 = 1000; // Depending on your display you may have to set this lower, to 400 or 100kHz.
    let display_i2c = I2C::i2c0(i2c0, display_pins.sda, display_pins.scl, I2C_FREQ_KHZ.kHz(), resets, system_clock.freq());

    liquid_crystal::I2C::new(display_i2c, crate::display::DISPLAY_I2C_ADDRESS)
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

fn configure_timers(pwm: PWM, timer: TIMER, resets: &mut RESETS, clocks: &ClocksManager) -> Timer {
    // PWM used as timer. Used to control how often we ask the LMT01's to produce a value for us
    let slices = rp_pico::hal::pwm::Slices::new(pwm, resets);
    let mut sample_rate_timer = slices.pwm0.into_mode::<FreeRunning>();

    core::assert!(clocks.system_clock.freq().to_MHz() == 125); // Assume freq < 128MHz because prescaler is 8-bit and we multiply clock by 2

    sample_rate_timer.disable();
    sample_rate_timer.default_config();
    sample_rate_timer.set_div_int( (clocks.system_clock.freq().to_MHz() * 2) as u8 ); // 125 MHz / 250 => 2us resolution => max ~130ms range
    sample_rate_timer.set_top(62_500 - 1); // Want 125ms range
    sample_rate_timer.clear_interrupt();
    sample_rate_timer.enable_interrupt();
    critical_section::with(|cs| {
        crate::SAMPLE_RATE_TIMER.borrow_ref_mut(cs).replace(sample_rate_timer);
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP); }

    // System timer, general use (delays, etc.)
    let mut system_timer = rp_pico::hal::Timer::new(timer, resets, clocks);

    // We use alarm 0 used to tell us when we can retrieve a value from the LMT01's. 
    let Some(mut sensors_ready_timer) = system_timer.alarm_0() else { unreachable!() };
    sensors_ready_timer.clear_interrupt();
    sensors_ready_timer.enable_interrupt();
    critical_section::with(|cs| {
        crate::SENSORS_READY_TIMER.borrow_ref_mut(cs).replace(sensors_ready_timer);
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0); }

    system_timer
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
            .manufacturer("Ross Porter")
            .product("OpenDatalogger Thermometer") 
            .serial_number("69420")])
        .unwrap_or_else(|_| unreachable!())
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