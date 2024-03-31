/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};

use samn_common::{radio::{Payload, Radio}, cc1101::{Cc1101}};
use embedded_hal::spi::Mode;
use heapless::Vec;
use panic_serial as _;

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

    let mut cc1101 = Cc1101::new(spi).unwrap();
    delay_ms(10);
    cc1101.reset().unwrap();
    delay_ms(10);
    cc1101.configure();

    let (chip_part_number, chip_version) = cc1101.get_hw_info().unwrap();
    ufmt::uwriteln!(
        &mut serial,
        "cc1101 part# {} version {}\r",
        chip_part_number,
        chip_version
    )
    .unwrap_infallible();

    let mut led = pins.led.into_output();
    let mut crc_pin = pins.g2_ce;

    let mut t = 0;
    loop {
        {
            // Format a message
            let mut data: Vec<u8, 31> = Default::default();
            ufmt::uwrite!(&mut data, "Hello :) {}", t).unwrap();
            ufmt::uwriteln!(
                &mut serial,
                "Sending {}\r",
                t
            ).unwrap_infallible();
            
            // Transmit the payload
            cc1101.transmit_(&Payload::new(&data)).unwrap();

            // ufmt::uwrite!(&mut serial,"writing, ",).unwrap_infallible();
            // let mut packet = [0u8;32];
            // packet[0] = 1;
            // packet[1] = 87;
            // cc1101.0.write_fifo(&packet).unwrap();
            // ufmt::uwrite!(&mut serial,"state: {}, ",cc1101.get_marc_state().unwrap()).unwrap_infallible();
            // ufmt::uwrite!(&mut serial,"to tx, ",).unwrap_infallible();
            // cc1101.0.write_strobe(cc1101::Command::STX).unwrap();
            // delay_ms(100);
            // ufmt::uwrite!(&mut serial,"state: {}, ",cc1101.get_marc_state().unwrap()).unwrap_infallible();
            // loop {}
            // cc1101.set_radio_mode(cc1101::RadioMode::Transmit).unwrap();
            // ufmt::uwrite!(&mut serial,"to idle, ",).unwrap_infallible();
            // cc1101.await_machine_state(cc1101::MachineState::IDLE).unwrap();
            // ufmt::uwriteln!(&mut serial,"flushing\r",).unwrap_infallible();
            // cc1101.0.write_strobe(cc1101::Command::SFTX).unwrap();
        }

        // Try to get an ack back
        let mut got_ack = false;
        
        {
            // ufmt::uwriteln!(&mut serial,"going to receive\r",).unwrap_infallible();
            // Go to receive mode
            cc1101.flush_rx().unwrap();
            cc1101.to_rx().unwrap();

            // Wait ~200ms for an ack
            let mut i = 0;
            while i < 200 {
                if let Ok(payload) = cc1101.receive_(&mut crc_pin) {
                    
                    if payload.data()[0] == 55 {
                        got_ack = true;
                        break; // Only break if we received the ack
                    }
                }
                i += 1;
                arduino_hal::delay_ms(1);
            }
            // Switch back to Idle
            cc1101.to_idle().unwrap();
        }

        // Notify user of ack/no-ack
        led.toggle();
        arduino_hal::delay_ms(50);
        led.toggle();
        if !got_ack {
            arduino_hal::delay_ms(100);
            led.toggle();
            arduino_hal::delay_ms(50);
            led.toggle();
        }

        t += 1;
        delay_ms(1000);
    }
}
