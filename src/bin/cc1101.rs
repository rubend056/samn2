/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};

use cc1101::{Cc1101, Config, DEVIATN, FSCTRL1, MCSM1, MDMCFG0, MDMCFG1, MDMCFG2, MDMCFG3, MDMCFG4, PKTCTRL1};
use embedded_hal::spi::SpiDevice;
use panic_serial as _;

panic_serial::impl_panic_handler!(
    // This is the type of the UART port to use for printing the message:
    arduino_hal::usart::Usart<
      arduino_hal::pac::USART0,
      arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
      arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
    >
);

const RX: bool = false;

use samn_common::radio::cc1101::config_1;


#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);
    ufmt::uwriteln!(
        &mut serial,
        "Hello\r",
    ).unwrap();

    // Create SPI interface.
    let (spi, _) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        spi::Settings::default(),
    );
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.d7.into_output(), Delay::new());

    let mut cc1101 = Cc1101::new(spi).unwrap();
    let (chip_part_number, chip_version) = cc1101.get_hw_info().unwrap();
    ufmt::uwriteln!(
        &mut serial,
        "cc1101 part# {} version {}\r",
        chip_part_number,
        chip_version
    )
    .unwrap_infallible();

    cc1101.0.write_strobe(cc1101::Command::SRES).unwrap();
    delay_ms(1);
    
    config_1(&mut cc1101);

    ufmt::uwriteln!(&mut serial, "cc1101 configured\r").unwrap_infallible();

    if RX {
        cc1101.set_radio_mode(cc1101::RadioMode::Receive).unwrap();

        loop {
            let mut buf = [0; 12];
            let len = cc1101.receive(&mut buf).unwrap();

            let b = core::str::from_utf8(&buf[0..len as usize]).unwrap();
            ufmt::uwriteln!(&mut serial, "len {} got: {}\r", len, b).unwrap_infallible();
        }
    } else {
        let mut t = 0;
        loop {
            cc1101.set_radio_mode(cc1101::RadioMode::Transmit).unwrap();
            cc1101.transmit(b"Hello :)").unwrap();
            ufmt::uwriteln!(&mut serial, "Sent out {}\r", t).unwrap_infallible();
            t+=1;
            delay_ms(1000);
        }
    }
}



