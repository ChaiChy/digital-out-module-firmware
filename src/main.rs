#![no_std]
#![no_main]

use core::fmt::Write;

use bsp::entry;
use defmt_rtt as _;
use embedded_hal::digital::{OutputPin, PinState, StatefulOutputPin};
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

fn read_pin_state<GPIO: PinId, F: Function, P: PullType>(pin: &mut bsp::hal::gpio::Pin<GPIO, F, P>)-> Result<bool, <bsp::hal::gpio::Pin<GPIO, F, P> as embedded_hal::digital::ErrorType>::Error> where bsp::hal::gpio::Pin<GPIO, F, P>: OutputPin + StatefulOutputPin{
    pin.is_set_high()
}

#[entry]
fn main() -> ! {

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

    // serial_id of device
    let serial_id = "2";
    
    let mut do_module_usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xf00d, 0xbabe))
        .strings(&[StringDescriptors::default()
            .manufacturer("Rocket Factory Augsburg")
            .product("HIL Testbench [DIGITAL OUT]")
            .serial_number(serial_id)])
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

    //let do_pins = [&mut do_1, &mut do_2, &mut do_3, &mut do_4, &mut do_5, &mut do_6, &mut do_7, &mut do_8, &mut do_9, &mut do_10];
    let do_pins: [&mut dyn OutputPin<Error = _>; 1] = [&mut do_1];
    do_pins.iter_mut().enumerate().for_each(|(index, pin)| {
        pin.set_high().unwrap();
    });

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

                    let addressed_serial_id = (received_value >> 11) & 0x1F;
                    let rw_bit = (received_value >> 10) & 0x01;
                    
                    if addressed_serial_id != serial_id.parse::<u16>().unwrap() {
                        
                        // payload is a write command
                        if rw_bit == 1 {
                            let new_do_state = received_value & 0x3FF;
                            
                            for (idx, pin) in do_pins.iter_mut().enumerate() {
                                set_pin_state(new_do_state, idx, pin).unwrap();
                            }

                            let mut text: String<64> = String::new();
                            writeln!(&mut text, "new digital out state: {}\r\n", new_do_state).unwrap();
                        }
                        // payload is a read command
                        else if  rw_bit == 0 {
                            let mut pin_state = 0;
                            for (idx, pin) in do_pins.iter_mut().enumerate() {
                                let bit = read_pin_state(pin).unwrap() as u16;
                                pin_state |= bit << idx;
                            }
                        }

                    }
                    let mut text: String<64> = String::new();
                    writeln!(&mut text, "received val: {}\r\n", received_value).unwrap();
                    
                }
            }

        }
    }
}

// End of file
