# Kiffieboot 3k (kb3k)

A DFU bootloader for PIC32MX microcontroller (MCU) that fits into the 3KiB boot
flash memory. It has a minimalist implementation of the [USB Device Class for
Device Firmware Upgrade (DFU)](https://usb.org/sites/default/files/DFU_1.1.pdf)

The bootloader can be accessed with [dfu-util](https://dfu-util.sourceforge.net/).
`dfu-util` is available for many Linux distribution and also for Windows (not
tested under Windows). I can be started as follows:

```sh
dfu-util -R -D image.bin
```

where `image.bin` is flash image in binary format to be stored in the program
flash memory starting at virtual address 0x9d00_0000.

The bootloader has been tested on PIC32MX2xx and PIC32MX274 devices but should
run on similar devices having a full speed USB peripheral as well

## Images of the bootloader

There is a build script `build_all.sh` that creates various variants of binaries
for different MCUs and different system clock frequencies. Precompiled binaries
are stored in the `bin` subdirectory. To build the binaries, the [ChipKIT PIC32
compiler](https://github.com/chipKIT32/chipKIT-compiler-builds/releases) is used
(I believe that Platformio uses the same compiler for PIC32). However, other GCC
variants, including the official toolchain from Microchip, can be used as well
when performing minor changes in the code.

### Stand alone image

Stand alone images (filenames ending with `.hex`) are in Intel Hex format and
can be directly flashed into the boot flash. Please check if the configuration
register bits contained in the image are suitable for the application.

### Embedded blob

The embedded blobs (filenames ending with `.bin` and `.xxd`) include a
3056-byte blob of the bootloader without configuration words. These files are
meant to be included into the build process or an application image that
includes both the bootloader as well as the application.

In Rust that can look like this

```rust
#[link_section = ".bootloader"]
#[used]
pub static BOOT_LOADER: [u8; 3056] = *include_bytes!("kb3k-dfu-mx2xx-48mhz.bin");
```

Note that a the section `.bootloader` must be defined in the linker script so
that the bootloader blob will be put into the boot flash at 0xbfc00000.

## Starting the bootloader

### Starting upon hardware reset

The bootloader starts automatically if the first word of the program flash is
0xffffffff. It also starts when the reset is triggered by the MCLR pin of the
MCU.

### Starting by the application

The application can start the bootloader by writing the magic word 0x746f6f62
into the SRAM and then resetting the MCU.

A Rust crate to start the bootloader and to provide an USB Device Firmware
Upgrade (DFU) interface is in directory `kiffieboot-rs` of this repository.

When not using this crate, the bootloader can be started in Rust like so:

```rust
use mips_mcu::interrupt;
use pic32_hal::pac::{CFG, RCON};

const BOOT_MAGIC: u32 = 0x746f6f62;

/// Reset and start the Kiffieboot bootloader
pub fn start_bootloader() -> ! {
    let boot_magic = 0x8000_0000 as *mut u32; // start of SRAM
    let rcon = RCON::ptr();
    let cfg = CFG::ptr();
    unsafe {
        interrupt::disable();
        *boot_magic = BOOT_MAGIC; // store magic value into start of SRAM

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
```

Equivalent C code looks like this:

```c
void kb_enter_bootloader(void) {
    mips_di();
    uint32_t *boot_magic = (uint32_t *)0xa0000000; /* start of SRAM */
    *boot_magic = 0x746f6f62;                      /* store magic value at start of SRAM */

    /* do a software reset */
    SYSKEY = 0;
    SYSKEY = 0xaa996655;
    SYSKEY = 0x556699aa;
    RSWRSTSET = _RSWRST_SWRST_MASK;
    RSWRST;
    while(true) { }; /* wait until reset happens */
}
```
