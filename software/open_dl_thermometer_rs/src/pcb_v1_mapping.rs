use crate::constants::*;
use crate::gpio;

// LMT01 sensors have a VP and VN terminal.
pub type VN1                = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionPio0, gpio::PullNone>;
pub type VN2                = gpio::Pin<gpio::bank0::Gpio3, gpio::FunctionPio0, gpio::PullNone>;
pub type VN3                = gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionPio0, gpio::PullNone>;
pub type VN4                = gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionPio0, gpio::PullNone>;
pub type VN5                = gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPio1, gpio::PullNone>;
pub type VN6                = gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionPio1, gpio::PullNone>;
pub type VN7                = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionPio1, gpio::PullNone>;
pub type VN8                = gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionPio1, gpio::PullNone>;

pub type VP1Off             = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP2Off             = gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP3Off             = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP4Off             = gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP5Off             = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP6Off             = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP7Off             = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP8Off             = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;
/*
pub type VP1On              = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP2On              = gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP3On              = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP4On              = gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP5On              = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP6On              = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP7On              = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP8On              = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioOutput, gpio::PullNone>;
*/
pub type SdCardMiso         = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardSck          = gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardMosi         = gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardCs           = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionSioOutput, gpio::PullUp>;
pub type SdCardWriteProtect = gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionSioInput, gpio::PullUp>;
pub type SdCardDetect       = gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionSioInput, gpio::PullUp>;

pub type DisplaySda         = gpio::Pin<gpio::bank0::Gpio20, gpio::FunctionI2c, gpio::PullUp>;
pub type DisplayScl         = gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionI2c, gpio::PullUp>;

pub type SelectButton       = gpio::Pin<gpio::bank0::Gpio27, gpio::FunctionSioInput, gpio::PullUp>;
pub type NextButton         = gpio::Pin<gpio::bank0::Gpio28, gpio::FunctionSioInput, gpio::PullUp>;

/// Contains the GPIO pins that power the LMT01 sensors. Note that turning on and off actually toggles between these pins being Output high and Input Hi-Z, as per datasheet.
pub struct TempPowerPins {
    pins_in:  arrayvec::ArrayVec::<gpio::Pin<gpio::DynPinId, gpio::DynFunction, gpio::PullNone>,NUM_SENSOR_CHANNELS>,
    pins_out: arrayvec::ArrayVec::<gpio::Pin<gpio::DynPinId, gpio::DynFunction, gpio::PullNone>,NUM_SENSOR_CHANNELS>,
}
impl TempPowerPins {
    /// Turns the LMT01 sensors on
    pub fn turn_on(&mut self) {
        let mut temp_pins: arrayvec::ArrayVec::<_,NUM_SENSOR_CHANNELS> = arrayvec::ArrayVec::new();
        core::mem::swap(&mut temp_pins, &mut self.pins_in);
        for pin in temp_pins.into_iter() {
            self.pins_out.push(pin.reconfigure());
        }
    }
    /// Turns the LMT01 sensors off
    pub fn turn_off(&mut self) {
        let mut temp_pins: arrayvec::ArrayVec::<_,NUM_SENSOR_CHANNELS> = arrayvec::ArrayVec::new();
        core::mem::swap(&mut temp_pins, &mut self.pins_out);
        for pin in temp_pins.into_iter() {
            self.pins_in.push(pin.reconfigure());
        }
    }
    /// Inverts the state of the LMT01 sensors' power rails
    pub fn invert(&mut self) {
        match self.pins_in.len() {
            NUM_SENSOR_CHANNELS => self.turn_on(),
            _ => self.turn_off(),
        }
    }
    /// Inverts the state of the LMT01 sensors' power rails, and then inverts it again
    pub fn pulse(&mut self) {
        self.invert();
        // Do we need a delay here?
        self.invert();
    }
    #[allow(clippy::too_many_arguments)]
    pub fn new(vp1: VP1Off, vp2: VP2Off, vp3: VP3Off, vp4: VP4Off, vp5: VP5Off, vp6: VP6Off, vp7: VP7Off, vp8: VP8Off) -> Self {
        let pins_in: arrayvec::ArrayVec<gpio::Pin<gpio::DynPinId, gpio::DynFunction, gpio::PullNone>, NUM_SENSOR_CHANNELS> = 
        arrayvec::ArrayVec::<_, NUM_SENSOR_CHANNELS>::from_iter([
            vp1.reconfigure().into_dyn_pin(), 
            vp2.reconfigure().into_dyn_pin(), 
            vp3.reconfigure().into_dyn_pin(), 
            vp4.reconfigure().into_dyn_pin(), 
            vp5.reconfigure().into_dyn_pin(), 
            vp6.reconfigure().into_dyn_pin(), 
            vp7.reconfigure().into_dyn_pin(), 
            vp8.reconfigure().into_dyn_pin()]);
        let pins_out: arrayvec::ArrayVec::<_,NUM_SENSOR_CHANNELS> = arrayvec::ArrayVec::new();
        
        Self {pins_in, pins_out}
    }
}

/// Contains the pins used to read the LMT01 pulse trains
pub struct TempSensePins {
    pub vn1: VN1,
    pub vn2: VN2,
    pub vn3: VN3,
    pub vn4: VN4,
    pub vn5: VN5,
    pub vn6: VN6,
    pub vn7: VN7,
    pub vn8: VN8,
}

pub struct SdCardPins {
    pub spi:            SdCardSPIPins,
    pub cs:             SdCardCs,
    pub extra:          SdCardExtraPins,
}
pub struct SdCardSPIPins { // Pins managed by SPI peripheral
    pub mosi:           SdCardMosi,
    pub miso:           SdCardMiso,
    pub sck:            SdCardSck,
}
pub struct SdCardExtraPins { // Other pins related to SD card
    pub write_protect:  SdCardWriteProtect,
    pub card_detect:    SdCardDetect,
}

pub struct DisplayPins {
    pub sda: DisplaySda,
    pub scl: DisplayScl,
}

pub struct ButtonPins {
    pub select: SelectButton,
    pub next:   NextButton,
}