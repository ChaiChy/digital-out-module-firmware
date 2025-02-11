//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::fmt::Write;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{OutputPin, PinState};
use panic_probe as _;

use heapless::String;
use usb_device::{class_prelude::UsbBusAllocator, device::UsbDeviceBuilder, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// Provide an alias for our BSP so we can switch targets quickly. BSP = Board Support Package
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::gpio::{Function, PinId, PullType},
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
};

// checks for a specific bit in val, set gpio of corresponding bit to the value of that bit.
fn set_pin_state<GPIO: PinId, F: Function, P: PullType>(val: u16, bit: u8, pin: &mut bsp::hal::gpio::Pin<GPIO, F, P>) -> Result<(), <bsp::hal::gpio::Pin<GPIO, F, P> as embedded_hal::digital::ErrorType>::Error> where bsp::hal::gpio::Pin<GPIO, F, P>: OutputPin{
    if (val >> bit) & 1 == 1 {
        pin.set_high()
    } else {
        pin.set_low()
    }
}
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // initialize usb_peripheral
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // set up a serial port to listen from
    let mut serial = SerialPort::new(&usb_bus);

    let mut do_module_usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1, 0x2))
        .strings(&[StringDescriptors::default()
            .manufacturer("Slavery AG")
            .product("Temu Lite")
            .serial_number("Test")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //let mut led_pin = pins.led.into_push_pull_output();


    // gpio instances for switching digital outs
    let mut do_1 = pins.gpio0.into_push_pull_output();
    let mut do_2 = pins.gpio1.into_push_pull_output();
    let mut do_3 = pins.gpio2.into_push_pull_output();
    let mut do_4 = pins.gpio3.into_push_pull_output();
    let mut do_5 = pins.gpio4.into_push_pull_output();
    let mut do_6 = pins.gpio5.into_push_pull_output();
    let mut do_7 = pins.gpio6.into_push_pull_output();
    let mut do_8 = pins.gpio7.into_push_pull_output();
    let mut do_9 = pins.gpio8.into_push_pull_output();
    let mut do_10 = pins.gpio9.into_push_pull_output();


    // operation starts here: poll serial port to check for changes and update digital outputs
    loop {
       
        // Check serial port/usb bus for new data
        if do_module_usb_device.poll(&mut [&mut serial]) {
            let mut buffer = [0u8; 2];
            match serial.read(&mut buffer) {
                Err(_e) => {

                    // report to console that something ain't cookin
                    // let mut text: String<64> = String::new();
                    // writeln!(&mut text, "Something ain't right\r\n").unwrap();
                    
                    // let _ = serial.write(text.as_bytes());

                    //let _ = delay.delay_ms(1000_u32);
                }

                Ok(val) => {
                    //led_pin.set_high().unwrap();

                    let received_value = u16::from_le_bytes([buffer[0], buffer[1]]);

                    // set do_0 to high or low based on val
                    set_pin_state(received_value, 0, &mut do_1).unwrap();
                    set_pin_state(received_value, 1, &mut do_2).unwrap();
                    set_pin_state(received_value, 2, &mut do_3).unwrap();
                    set_pin_state(received_value, 3, &mut do_4).unwrap();
                    set_pin_state(received_value, 4, &mut do_5).unwrap();
                    set_pin_state(received_value, 5, &mut do_6).unwrap();
                    set_pin_state(received_value, 6, &mut do_7).unwrap();
                    set_pin_state(received_value, 7, &mut do_8).unwrap();
                    set_pin_state(received_value, 8, &mut do_9).unwrap();
                    set_pin_state(received_value, 9, &mut do_10).unwrap();

                    let mut text: String<64> = String::new();
                    writeln!(&mut text, "received val: {}\r\n", received_value).unwrap();
                    
                    let _ = serial.write(text.as_bytes());
                    let _ = delay.delay_ms(1000_u32);
                }
            }

        }
    }
}

// End of file
