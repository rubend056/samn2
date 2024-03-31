/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};

use samn_common::{cc1101::{ Cc1101, RadioMode}, radio::*};
use embedded_hal::spi::Mode;
use heapless::{String, Vec};
use panic_serial as _;

panic_serial::impl_panic_handler!(
    // This is the type of the UART port to use for printing the message:
    arduino_hal::usart::Usart<
      arduino_hal::pac::USART0,
      arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
      arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
    >
);

const RX: bool = true;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);
    ufmt::uwriteln!(&mut serial, "Hello\r",).unwrap();

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

    let mut cc1101 = Cc1101::new(spi).unwrap();
    delay_ms(10);
    cc1101.reset().unwrap();
    delay_ms(10);
    cc1101.configure();
    cc1101.flush_rx().unwrap();
    cc1101.flush_tx().unwrap();

    ufmt::uwriteln!(&mut serial, "cc1101 configured\r").unwrap_infallible();
    let (chip_part_number, chip_version) = cc1101.get_hw_info().unwrap();
    ufmt::uwriteln!(
        &mut serial,
        "cc1101 part# {} version {}\r",
        chip_part_number,
        chip_version
    )
    .unwrap_infallible();

    let mut crc_pin = pins.d6;

    loop {
        // Go back to receive mode
        cc1101.flush_rx().unwrap();
        cc1101.to_rx().unwrap();

        // Get a payload
        if let Ok(payload) = nb::block!(cc1101.receive_(&mut crc_pin)) {
            // ufmt::uwriteln!(&mut serial, "got pkt\r").unwrap_infallible();
            // delay_ms(5);

            // Send out an ack asap
            {
                // Have to do this or CCA won't let me switch to TX
                cc1101.to_idle().unwrap();
                cc1101.transmit_(&Payload::new(&[55])).unwrap();
            }

            // ufmt::uwrite!(&mut serial,"state: {}, ",cc1101.get_marc_state().unwrap()).unwrap_infallible();
            // let mut packet = [0u8;32];
            // packet[0] = 1;
            // packet[1] = 55;
            // cc1101.0.write_fifo(&packet).unwrap();
            // ufmt::uwrite!(&mut serial,"state: {}, ",cc1101.get_marc_state().unwrap()).unwrap_infallible();
            // ufmt::uwrite!(&mut serial,"to tx, ",).unwrap_infallible();
            // cc1101.0.write_strobe(cc1101::Command::STX).unwrap();
            // delay_ms(100);
            // ufmt::uwrite!(&mut serial,"state: {}, ",cc1101.get_marc_state().unwrap()).unwrap_infallible();
            // loop {}

            // Print out payload
            match core::str::from_utf8(payload.data()) {
                Ok(b) => ufmt::uwriteln!(&mut serial, "data: '{}'\r", b).unwrap_infallible(),
                Err(_) => ufmt::uwriteln!(&mut serial, "err?\r").unwrap_infallible(),
            };
        }
    }
}
