use core::cell::RefCell;

use critical_section::Mutex;
use rp_pico::hal::usb::UsbBus;
use ufmt::uWrite;
use usb_device::UsbError;
use usbd_serial::SerialPort;

/// USB serial object used for printing
pub static USB_SERIAL: Mutex<RefCell<Option<SerialPort<'static, UsbBus>>>> = Mutex::new(RefCell::new(None));

#[macro_export]
macro_rules! println {
    ($first:tt $(, $( $rest:tt )* )?) => {
        let mut stand_in = $crate::serial::Dummy{};
        ufmt::uwriteln!(&mut stand_in, $first, $( $($rest)* )*).ok()
    };
}


#[macro_export]
macro_rules! print {
    ($first:tt $(, $( $rest:tt )* )?) => {
        let mut stand_in = $crate::serial::Dummy{};
        ufmt::uwrite!(&mut stand_in, $first, $( $($rest)* )*).ok()
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