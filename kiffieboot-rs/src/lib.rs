//! Runtime support for Kiffieboot bootloader
//!
//! This crate can be used to start the bootloader or to provide an USB Device
//! Firmware Upgrade (DFU) class interface.
//!
//! # Example (USB DFU interface)
//!
//! ```no_run
//! # use pic32_hal::usb::UsbBus;
//! # use pic32_hal::pac;
//! # use pic32_hal::coretimer::Delay;
//! # use kiffieboot::{DfuRuntimeClass, Kiffieboot};
//! # use usb_device::prelude::*;
//! # let pac = pac::Peripherals::take().unwrap();
//! # use pic32_hal::time::U32Ext;
//! # use pic32_hal::clock::Osc;
//! # use embedded_hal::delay::DelayNs;
//! let sysclock = 48_000_000_u32.hz();
//! let clock = Osc::new(pac.OSC, sysclock);
//! let mut timer = Delay::new(sysclock);
//!
//! let usb_bus = UsbBus::new(pac.USB);
//! let mut dfu_runtime = DfuRuntimeClass::new(&usb_bus, Kiffieboot::default());
//! let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
//!     .strings(&[StringDescriptors::new(LangID::EN)
//!         .manufacturer("Some company")
//!         .product("Some USB device")])
//!     .unwrap()
//!     .build();
//!
//! loop {
//!     usb_dev.poll(&mut [&mut dfu_runtime]);
//!     dfu_runtime.tick(1);
//!     timer.delay_ms(1);
//! }
//! ```
//!
//! # Example (directly jump into the bootloader)
//!
//! ```no_run
//! # use kiffieboot::Kiffieboot;
//!  Kiffieboot::start_bootloader();
//! ```
//!
//! # Example (using dfu-util from a Linux shell)
//!
//! ```sh
//! dfu-util -R -D my_application.bin
//! ```

#![no_std]

use mips_mcu::interrupt;

#[cfg(feature = "usbd-dfu-rt")]
use usbd_dfu_rt::DfuRuntimeOps;

#[cfg(feature = "usbd-dfu-rt")]
pub use usbd_dfu_rt::DfuRuntimeClass;

#[cfg(feature = "pic32mx2x0")]
use pic32mx2xx::pic32mx2xxfxxxb::{CFG, RCON};

#[cfg(feature = "pic32mx2x4")]
use pic32mx2xx::pic32mx2x4fxxxd::{CFG, CRU as RCON}; // the XLP devices have the RCON register included in the CRU

#[cfg(feature = "pic32mx470")]
use pic32mx470::pic32mx47xfxxxl::{CFG, RCON};

const BOOT_MAGIC: u32 = 0x746f6f62;

#[derive(Default)]
pub struct Kiffieboot {}

impl Kiffieboot {
    /// Reset and start the bootloader
    pub fn start_bootloader() -> ! {
        let boot_magic = 0x8000_0000 as *mut u32; // start of SRAM
        let rcon = RCON::ptr();
        let cfg = CFG::ptr();
        unsafe {
            interrupt::disable();
            *boot_magic = BOOT_MAGIC; // store magic value to start of SRAM

            // initiate software reset
            (*cfg).syskey.write(|w| w.syskey().bits(0));
            (*cfg).syskey.write(|w| w.syskey().bits(0xaa996655));
            (*cfg).syskey.write(|w| w.syskey().bits(0x556699aa));
            (*rcon).rswrstset.write(|w| w.swrst().bit(true));
            let _ = (*rcon).rswrst.read().bits();
        }
        // wait for reset
        #[allow(clippy::empty_loop)]
        loop {}
    }
}

#[cfg(feature = "usbd-dfu-rt")]
impl DfuRuntimeOps for Kiffieboot {
    const WILL_DETACH: bool = false;
    const CAN_UPLOAD: bool = false;
    const MAX_TRANSFER_SIZE: u16 = 1024;

    fn detach(&mut self) {
        Self::start_bootloader();
    }
}
