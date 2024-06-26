#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::{Mode, SpiDevice};
use embedded_nrf24l01::{Device, NRF24L01};
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

const ALWAYS_RX: bool = false;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);

    let (mut spi, _) = arduino_hal::Spi::new(
        dp.SPI,
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

    let mut nrf24 = NRF24L01::new(pins.ce.into_output(), spi).unwrap();
    radio::nrf24::init(&mut nrf24);
    ufmt::uwriteln!(&mut serial, "setting up nrf\r").unwrap_infallible();

    // After this config, registers should look like this:
    // 00 -> 0e 0e // CRC on (2bytes),Power up
    // 01 -> 0e 01 // Auto ack on pipe 0
    // 02 -> 0e 01 // Pipe 0 Rx enabled
    // 03 -> 0e 03 // Rx address width 5bytes
    // 04 -> 0e ff // Most retransmissions, most delay
    // 05 -> 0e 08 // Channel 8
    // 06 -> 0e 0e // DR 2Mbps, max power
    // 07 -> 0e 0e // No interrupts, RX empty, TX empty
    // 08 -> 0e 00 // No lost packets or retrasmitted
    // 09 -> 0e 00 // No carrier detect
    // 0a -> 0e 66 // Rx_addr_0
    // 0b -> 0e c2 // Rx_addr_1
    // 0c -> 0e c3 // Rx_addr_2
    // 0d -> 0e c4 // Rx_addr_3
    // 0e -> 0e c5 // Rx_addr_4
    // 0f -> 0e c6 // Rx_addr_5
    // 10 -> 0e 66 // Tx_addr
    // 11 -> 0e 00 // Rx_pw_0 payload width
    // 12 -> 0e 00 // Rx_pw_1 payload width
    // 13 -> 0e 00 // Rx_pw_2 payload width
    // 14 -> 0e 00 // Rx_pw_3 payload width
    // 15 -> 0e 00 // Rx_pw_4 payload width
    // 16 -> 0e 00 // Rx_pw_5 payload width
    // 17 -> 0e 11 // Rx/Tx fifo empty
    // 1c -> 0e 3f // Rx dynamic payloads ON all pipes
    // 1d -> 0e 05 // Dyn payloads, Dyn ack

    
    let mut led = pins.led.into_output();

    if ALWAYS_RX {
        nrf24.rx().unwrap();
        ufmt::uwriteln!(&mut serial, "receiving\r").unwrap_infallible();

        loop {
            if let Some(_) = nrf24.can_read().unwrap() {
                if let Ok(buf) = nrf24.read() {
                    let b = core::str::from_utf8(buf.as_ref()).unwrap();
                    ufmt::uwriteln!(&mut serial, "len {} got: {}\r", buf.len(), b)
                        .unwrap_infallible();
                    led.set_high();
                    delay_ms(100);
                    led.set_low();
                }
            }
            delay_ms(10);
        }
    }

    ufmt::uwriteln!(&mut serial, "t: transmit; r: receive; p: print registers\r")
        .unwrap_infallible();
    
    let mut t = 0;
    loop {
        delay_ms(10);
        if let Ok(v) = serial.read() {
            if v == 116 {
                // t

                if nrf24.send(b"Hello").unwrap() {
                    ufmt::uwriteln!(&mut serial, "sent {}!\r", t).unwrap_infallible();
                    t += 1;
                } else {
                    ufmt::uwriteln!(&mut serial, "can't send\r").unwrap_infallible();
                }
            } else if v == 114 {
                //r
                nrf24.rx().unwrap();
                ufmt::uwriteln!(&mut serial, "receiving, press r to go back\r").unwrap_infallible();

                loop {
                    if serial.read() == Ok(114) {
                        ufmt::uwriteln!(&mut serial, "stopped receiving\r").unwrap_infallible();
                        break;
                    }

                    if let Ok(buf) = nrf24.receive() {
                        let b = core::str::from_utf8(buf.as_ref()).unwrap();
                        ufmt::uwriteln!(&mut serial, "len {} got: {}\r", buf.len(), b)
                            .unwrap_infallible();
                    }
                    delay_ms(100);
                }
                nrf24.ce_disable();
            } else if v == 112 {
                let mut a = |i| {
                    let w = [i; 1];
                    let mut r = [0x00; 2];
                    nrf24.spi.transfer(&mut r, &w).unwrap();

                    ufmt::uwriteln!(&mut serial, "{:02x} -> {:02x} {:02x}\r", w[0], r[0], r[1])
                        .unwrap_infallible();
                };

                for i in 0..=0x17 {
                    a(i);
                }
                a(0x1C);
                a(0x1D);
            } else if v != 10 {
                ufmt::uwriteln!(&mut serial, "no command matches {}\r", v).unwrap_infallible();
            }
        }
    }
}
