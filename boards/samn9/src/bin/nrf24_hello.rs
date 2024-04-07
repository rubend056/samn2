#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::{Mode, SpiDevice};
use samn_common::{nrf24::{DataRate, Device, NRF24L01}, radio::*};
use heapless::Vec;
use panic_serial as _;
use samn_common::radio::*;

panic_serial::impl_panic_handler!(
    // This is the type of the UART port to use for printing the message:
    arduino_hal::usart::Usart<
      arduino_hal::pac::USART0,
      arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
      arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
    >
);

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);

    let (mut spi, _) = arduino_hal::Spi::new(
        dp.SPI0,
        pins.b5.into_output(),
        pins.b3.into_output(),
        pins.b4.into_pull_up_input(),
        pins.b2.into_output(),
        spi::Settings {
            mode: Mode {
                polarity: embedded_hal::spi::Polarity::IdleLow,
                phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
            },
            clock: spi::SerialClockRate::OscfOver2,
            ..Default::default()
        },
    );

    let mut spi =
        embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());

    let mut nrf24 = NRF24L01::new(pins.g2_ce.into_output(), spi).unwrap();
    nrf24.configure().unwrap();

    let mut led = pins.led.into_output();

    {
        let mut t = 0;
        loop {
            let mut out: Vec<u8, 32> = Default::default();
            ufmt::uwrite!(&mut out, "Hello :) {} ", t).unwrap();
            // Because nrf24 has 
            if nrf24.transmit_(&Payload::new(&out)).unwrap().unwrap() {
                // Got ack
                led.toggle();
                delay_ms(50);
                led.toggle();
            } else {
                // Didn't get ack
                led.toggle();
                delay_ms(50);
                led.toggle();
                delay_ms(100);
                led.toggle();
                delay_ms(50);
                led.toggle();
            }
            t += 1;
            delay_ms(1000);
        }
    }
}
