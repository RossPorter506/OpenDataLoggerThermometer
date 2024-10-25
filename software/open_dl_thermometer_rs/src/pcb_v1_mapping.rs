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

pub type VP1Off                = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP2Off                = gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP3Off                = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP4Off                = gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP5Off                = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP6Off                = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP7Off                = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullNone>;
pub type VP8Off                = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;

pub type VP1On                = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP2On                = gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP3On                = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP4On                = gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP5On                = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP6On                = gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP7On                = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioOutput, gpio::PullNone>;
pub type VP8On                = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioOutput, gpio::PullNone>;

pub type SdCardMiso         = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardCs           = gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardSck          = gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardMosi         = gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionSpi, gpio::PullUp>;
pub type SdCardWriteProtect = gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionI2c, gpio::PullUp>;
pub type SdCardDetect       = gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionI2c, gpio::PullUp>;

pub type DisplaySck         = gpio::Pin<gpio::bank0::Gpio20, gpio::FunctionI2c, gpio::PullNone>;
pub type DisplayScl         = gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionI2c, gpio::PullNone>;

pub type SelectButton       = gpio::Pin<gpio::bank0::Gpio27, gpio::FunctionSioInput, gpio::PullUp>;
pub type NextButton         = gpio::Pin<gpio::bank0::Gpio28, gpio::FunctionSioInput, gpio::PullUp>;

/// Contains GPIO pins that power LMT01 sensors. Note that turning on and off actually toggles between Output high and Input Hi-Z, as per datasheet.
pub struct TempPowerOff {
    pub vp1: VP1Off,
    pub vp2: VP2Off,
    pub vp3: VP3Off,
    pub vp4: VP4Off,
    pub vp5: VP5Off,
    pub vp6: VP6Off,
    pub vp7: VP7Off,
    pub vp8: VP8Off,
}
impl TempPowerOff {
    fn turn_on(self) -> TempPowerOn {
        TempPowerOn{ vp1: self.vp1.reconfigure(), vp2: self.vp2.reconfigure(), vp3: self.vp3.reconfigure(), vp4: self.vp4.reconfigure(), vp5: self.vp5.reconfigure(), vp6: self.vp6.reconfigure(), vp7: self.vp7.reconfigure(), vp8: self.vp8.reconfigure() }
    }
}
/// Contains GPIO pins that power LMT01 sensors. Note that turning on and off actually toggles between Output high and Input Hi-Z, as per datasheet.
pub struct TempPowerOn {
    pub vp1: VP1On,
    pub vp2: VP2On,
    pub vp3: VP3On,
    pub vp4: VP4On,
    pub vp5: VP5On,
    pub vp6: VP6On,
    pub vp7: VP7On,
    pub vp8: VP8On,
}
impl TempPowerOn {
    fn turn_off(self) -> TempPowerOff {
        TempPowerOff{ vp1: self.vp1.reconfigure(), vp2: self.vp2.reconfigure(), vp3: self.vp3.reconfigure(), vp4: self.vp4.reconfigure(), vp5: self.vp5.reconfigure(), vp6: self.vp6.reconfigure(), vp7: self.vp7.reconfigure(), vp8: self.vp8.reconfigure() }
    }
}


pub struct TempSense {
    pub vn1: VN1,
    pub vn2: VN2,
    pub vn3: VN3,
    pub vn4: VN4,
    pub vn5: VN5,
    pub vn6: VN6,
    pub vn7: VN7,
    pub vn8: VN8,
}

pub struct SDCardPins {
    pub mosi:           SdCardMosi,
    pub miso:           SdCardMiso,
    pub sck:            SdCardSck,
    pub cs:             SdCardCs,
    pub write_protect:  SdCardWriteProtect,
    pub card_detect:    SdCardDetect,
}

pub struct DisplayPins {
    pub sck: DisplaySck,
    pub scl: DisplayScl,
}

pub struct ButtonPins {
    pub select: SelectButton,
    pub next:   NextButton,
}