# Runtime support for Kiffieboot kb3k bootloader including USB interface

This crate can be used to start the bootloader or to provide an USB Device
Firmware Upgrade (DFU) class interface. The DFU support is based on the
`usbd-dfu-rt` crate.

## Example (USB DFU interface)

```rust
let sysclock = 48_000_000_u32.hz();
let clock = Osc::new(pac.OSC, sysclock);
let mut timer = Delay::new(sysclock);
let usb_bus = UsbBus::new(pac.USB);
let mut dfu_runtime = DfuRuntimeClass::new(&usb_bus, Kiffieboot::default());
let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
    .manufacturer("Some company")
    .product("Some USB device")
    .build();
loop {
    usb_dev.poll(&mut [&mut dfu_runtime]);
    dfu_runtime.tick(1);
    timer.delay_ms(1);
}
```

## Example (directly jump into the bootloader)

```rust
 Kiffieboot::start_bootloader();
```

## Example (using dfu-util from a Linux shell)

```sh
dfu-util -R -D my_application.bin
```
