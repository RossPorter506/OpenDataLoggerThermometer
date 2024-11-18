use core::cell::RefCell;

use critical_section::Mutex;
use rp_pico::hal::{timer::Instant, usb::UsbBus};
use to_arraystring::ToArrayString;
use ufmt::uWrite;
use usb_device::UsbError;
use usbd_serial::SerialPort;
use arrayvec::ArrayString;

use crate::{display::DisplayValues, lmt01::CHARS_PER_READING, NUM_SENSOR_CHANNELS};

/// USB serial object used for printing
pub static USB_SERIAL: Mutex<RefCell<Option<SerialPort<'static, UsbBus>>>> = Mutex::new(RefCell::new(None));

/// Standard printing
#[macro_export]
macro_rules! println {
    ($first:tt $(, $( $rest:tt )* )?) => {
        let mut stand_in = $crate::serial::Dummy{};
        ufmt::uwriteln!(&mut stand_in, $first, $( $($rest)* )*).ok()
    };
}
/// Standard printing
#[macro_export]
macro_rules! print {
    ($first:tt $(, $( $rest:tt )* )?) => {
        let mut stand_in = $crate::serial::Dummy{};
        ufmt::uwrite!(&mut stand_in, $first, $( $($rest)* )*).ok()
    };
}

/// Error printing
#[macro_export]
macro_rules! eprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("# "); $crate::println!($first, $( $($rest)* )*)
    };
}
/// Error printing
#[macro_export]
macro_rules! eprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("# "); $crate::print!($first, $( $($rest)* )*)
    };
}

/// Warning printing
#[macro_export]
macro_rules! wprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("? "); $crate::println!($first, $( $($rest)* )*)
    };
}
/// Warning printing
#[macro_export]
macro_rules! wprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("? "); $crate::print!($first, $( $($rest)* )*)
    };
}

/// Info printing
#[macro_export]
macro_rules! iprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("/ "); $crate::println!($first, $( $($rest)* )*)
    };
}
/// Info printing
#[macro_export]
macro_rules! iprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("/ "); $crate::print!($first, $( $($rest)* )*)
    };
}

/// Debug printing
#[macro_export]
macro_rules! dprintln {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("- "); $crate::println!($first, $( $($rest)* )*)
    };
}
/// Debug printing
#[macro_export]
macro_rules! dprint {
    ($first:tt $(, $( $rest:tt )* )?) => {
        $crate::print!("- "); $crate::print!($first, $( $($rest)* )*)
    };
}

/// Dummy struct that calls print function
pub struct Dummy{}
impl uWrite for Dummy {
    type Error = UsbSerialPrintError;
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        blocking_print(s.as_bytes())
    }
}

/// Attempts to send data over serial. Blocks until the entire message is sent.
fn blocking_print<'a>(str: impl Into<&'a [u8]>) -> Result<(), UsbSerialPrintError>{
    let buf: &[u8] = str.into();
    let mut start = 0;
    while start < buf.len() {
        use UsbSerialPrintError::*;
        match nonblocking_print(&buf[start..]) {
            // Remove whatever was successfully sent from our buffer
            Err(WouldBlock(len)) => start += len,
            // Err(WouldBlock) implies buffer is full.
            Err(OtherError(a)) => return Err(OtherError(a)),
            Ok(()) => return Ok(()),
        };
    }
    Ok(())
}

/// Attempts to send data over serial. If the buffer is full or is filled the WouldBlock error contains how many bytes were sent before the buffer was filled.
pub fn nonblocking_print<'a>(str: impl Into<&'a [u8]>) -> Result<(), UsbSerialPrintError>{
    let buf: &[u8] = str.into();
    critical_section::with(|cs| {
        let Some(mut serial) = USB_SERIAL.take(cs) else {return Err(UsbSerialPrintError::OtherError(UsbError::InvalidState))};
        match serial.write(buf) {
            Ok(len) if len == buf.len() => Ok(()), 
            Ok(len)                     => Err(UsbSerialPrintError::WouldBlock(len)),
            // Err(WouldBlock) implies buffer is full.
            Err(UsbError::WouldBlock)   => Err(UsbSerialPrintError::WouldBlock(0)),
            Err(a)                      => Err(UsbSerialPrintError::OtherError(a)),
        }
    })
}

pub enum UsbSerialPrintError {
    /// Partial or no send. Contains how many bytes were sent.
    WouldBlock(usize),
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