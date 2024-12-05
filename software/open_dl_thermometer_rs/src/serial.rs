use core::cell::RefCell;

use critical_section::Mutex;
use rp_pico::hal::{timer::Instant, usb::UsbBus};
use to_arraystring::ToArrayString;
use usb_device::UsbError;
use usbd_serial::SerialPort;
use arrayvec::ArrayString;

use crate::{display::DisplayValues, lmt01::CHARS_PER_READING, NUM_SENSOR_CHANNELS};

/// USB serial object used for printing
pub static USB_SERIAL: Mutex<RefCell<Option<SerialPort<'static, UsbBus>>>> = Mutex::new(RefCell::new(None));

/// Standard printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! println {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            let mut stand_in = $crate::serial::Dummy{};
            use core::fmt::Write;
            writeln!(&mut stand_in, $first, $( $($rest)* )*).ok()
        }
    };
}
/// Standard printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! print {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            let mut stand_in = $crate::serial::Dummy{};
            use core::fmt::Write;
            write!(&mut stand_in, $first, $( $($rest)* )*).ok()
        }
    };
}

/// Error printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! eprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::println!("# {}", format_args!($first, $( $($rest)* )*))
    };
}
/// Error printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! eprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("# {}", format_args!($first, $( $($rest)* )*))
    };
}

/// Warning printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! wprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::println!("? {}", format_args!($first, $( $($rest)* )*))
    };
}
/// Warning printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! wprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("? {}", format_args!($first, $( $($rest)* )*))
    };
}

/// Info printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! iprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::println!("/ {}", format_args!($first, $( $($rest)* )*))
    };
}
/// Info printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! iprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("/ {}", format_args!($first, $( $($rest)* )*))
    };
}

/// Debug printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! dprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::println!("- {}", format_args!($first, $( $($rest)* )*))
    };
}
/// Debug printing. Panics if another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
#[macro_export]
macro_rules! dprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("- {}", format_args!($first, $( $($rest)* )*))
    };
}

/// Dummy struct that calls print function
pub struct Dummy{}
impl core::fmt::Write for Dummy {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        blocking_print(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

/// Attempts to send data over serial. Blocks until the entire message is sent. Note that if the buffer fills up this fn will block until a listener attaches.
/// 
/// Panics: If another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
pub fn blocking_print<'a>(str: impl Into<&'a [u8]>) -> Result<(), UsbSerialPrintError>{
    let buf: &[u8] = str.into();
    let mut start = 0;
    while start < buf.len() {
        use UsbSerialPrintError::*;
        match nonblocking_print(&buf[start..]) {
            Ok(()) => return Ok(()),
            // Partial send. Remove whatever was successfully sent from our buffer
            Err(WouldBlock(len)) => start += len,
            Err(OtherError(a)) => return Err(OtherError(a)),
        };
    }
    Ok(())
}

/// Attempts to send data over serial. If the buffer is full or is filled the WouldBlock error contains how many bytes were sent before the buffer was filled.
/// 
/// Panics: If another mutable reference to USB_SERIAL exists, or if `configure_usb()` hasn't been called yet.
pub fn nonblocking_print<'a>(str: impl Into<&'a [u8]>) -> Result<(), UsbSerialPrintError>{
    use UsbSerialPrintError::*;
    let buf: &[u8] = str.into();

    critical_section::with(|cs| {
        let Some(ref mut serial) = *USB_SERIAL.borrow_ref_mut(cs) else { panic!() }; // Only reachable if this fn is called before USB is configured

        match serial.write(buf) {
            Ok(len) if len == buf.len() => Ok(()), 
            Ok(len)                     => Err(WouldBlock(len)),
            // Err(WouldBlock) implies buffer is full.
            Err(UsbError::WouldBlock)   => Err(WouldBlock(0)),
            Err(a)                      => Err(OtherError(a)),
        }
    })
}

pub fn nonblocking_read<'a>(buf: impl Into<&'a mut [u8]>) -> Result<usize, UsbSerialReadError>{
    use UsbSerialReadError::*;

    critical_section::with(|cs| {
        let Some(ref mut serial) = *USB_SERIAL.borrow_ref_mut(cs) else { panic!() }; // Only reachable if this fn is called before USB is configured

        match serial.read(buf.into()) {
            Ok(0)                       => Err(WouldBlock),
            Ok(len)                     => Ok(len), 
            // Err(WouldBlock) implies buffer is full.
            Err(UsbError::WouldBlock)   => Err(WouldBlock),
            Err(a)                      => Err(OtherError(a)),
        }
    })
}

#[derive(Debug)]
pub enum UsbSerialPrintError {
    /// Partial or no send. Contains how many bytes were sent.
    WouldBlock(usize),
    OtherError(UsbError),
}
#[derive(Debug)]
pub enum UsbSerialReadError {
    /// Partial or no send. Contains how many bytes were sent.
    WouldBlock,
    OtherError(UsbError),
}

pub const MAX_TIMESTAMP_LEN: usize = (u64::MAX/1000).ilog10() as usize + 1;
/// {ts},{s1},{s2},{s3},{s4},{s5},{s6},{s7},{s8},;\n
pub const MAX_SNAPSHOT_LEN: usize = (MAX_TIMESTAMP_LEN+1) + NUM_SENSOR_CHANNELS*(CHARS_PER_READING+1) + 2;
/// Serialise timestamp and sensor values into a single string for transmission
pub fn serialise_snapshot(timestamp: Instant, sensor_readings: &DisplayValues) -> ArrayString<MAX_SNAPSHOT_LEN> {
    let mut snapshot = ArrayString::new();
    let timestamp_millis = timestamp.ticks()/1000;
    snapshot.push_str(&timestamp_millis.to_arraystring());
    snapshot.push_str(",");
    for opt_reading in sensor_readings {
        if let Some(reading) = opt_reading {
            snapshot.push_str(reading);
        }
        snapshot.push_str(",");
    }
    snapshot.push_str(";\n");
    snapshot
}