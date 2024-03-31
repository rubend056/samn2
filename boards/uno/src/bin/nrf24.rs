#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::{Mode, SpiDevice};
use embedded_nrf24l01::{DataRate, Device, NRF24L01};
use panic_serial as _;
use samn_common::radio;
// use samn2::radio::Radio;

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
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
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
        embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.d7.into_output(), Delay::new());

    let mut nrf24 = NRF24L01::new(pins.d6.into_output(), spi).unwrap();
    radio::nrf24::init(&mut nrf24);
    ufmt::uwriteln!(&mut serial, "setting up nrf\r").unwrap_infallible();

    {
        nrf24.rx().unwrap();
        ufmt::uwriteln!(&mut serial, "receiving\r").unwrap_infallible();

        loop {
            if let Ok(buf) = nrf24.receive() {
                let b = core::str::from_utf8(buf.as_ref()).unwrap();
                ufmt::uwriteln!(&mut serial, "len {} got: {}\r", buf.len(), b).unwrap_infallible();
            }
            delay_ms(1);
        }
    }
}
