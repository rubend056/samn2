#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::Mode;
use embedded_nrf24l01::{Configuration, CrcMode, DataRate, NRF24L01};
use panic_halt as _;

const TX: bool = true;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

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

    let mut nrf24 = NRF24L01::new(pins.d6.into_output(), pins.d7.into_output(), spi).unwrap();

    ufmt::uwriteln!(&mut serial, "setting up nrf\r").unwrap_infallible();

    nrf24.set_frequency(8).unwrap();
    nrf24.set_auto_retransmit(0, 0).unwrap();
    nrf24.set_rf(&DataRate::R250Kbps, 3).unwrap();
    nrf24
        .set_pipes_rx_enable(&[true, false, false, false, false, false])
        .unwrap();
    nrf24.set_auto_ack(&[false; 6]).unwrap();
    nrf24.set_crc(CrcMode::Disabled).unwrap();
    nrf24.set_rx_addr(0, &b"fnord"[..]).unwrap();
    nrf24.set_tx_addr(&b"fnord"[..]).unwrap();

    let mut t = 0;
    if TX {
        let mut nrf24 = nrf24.tx().unwrap();
        ufmt::uwriteln!(&mut serial, "transmitting\r").unwrap_infallible();
        loop {
            if nrf24.send(b"Hello").is_ok() {
                ufmt::uwriteln!(&mut serial, "sent {}!\r", t).unwrap_infallible();
                t += 1;
            } else {
                ufmt::uwriteln!(&mut serial, "not sent\r").unwrap_infallible();
            }
            delay_ms(1000);
        }
    } else {
        let mut nrf24 = nrf24.rx().unwrap();
        ufmt::uwriteln!(&mut serial, "receiving\r").unwrap_infallible();
        loop {
            // let connected  = nrf24.device().is_connected().unwrap();
            // ufmt::uwriteln!(&mut serial, "nrf connected: {}\r", connected).unwrap_infallible();
            if let Some(_) = nrf24.can_read().unwrap()  {
                if let Ok(buf) = nrf24.read() {
                    let b = core::str::from_utf8(buf.as_ref()).unwrap();
                    ufmt::uwriteln!(&mut serial, "len {} got: {}\r", buf.len(), b)
                        .unwrap_infallible();
                }
            }

            delay_ms(10)
        }
    }

    // loop {
    //     let connected  = nrf24.device().is_connected().unwrap();
    //     ufmt::uwriteln!(&mut serial, "nrf connected: {}\r", connected).unwrap_infallible();
    //     // ufmt::uwriteln!(&mut serial, "test").unwrap_infallible();
    //     arduino_hal::delay_ms(1000);
    // }
}
