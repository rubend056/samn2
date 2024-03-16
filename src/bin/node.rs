#![no_std]
#![no_main]

use core::cmp::{max, min};

use arduino_hal::{adc, delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::{Mode, SpiDevice};
use embedded_nrf24l01::{Configuration, NRF24L01};
use heapless::Vec;
use panic_serial as _;
use samn2::radio::Radio;
use samn_common::node::{
    Command, MessageData::{self}, NodeInfo, Sensor
};

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
    let node_info = NodeInfo {
        id: 43,
        board_version: 9,
    };

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);

    let (spi, _) = arduino_hal::Spi::new(
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

    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let bat_pin = pins.a0.into_analog_input(&mut adc);

    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.d7.into_output(), Delay::new());

    let mut radio = NRF24L01::new(pins.d6.into_output(), spi).unwrap();
    radio.init();

    loop {
        {
            // Read sensor data
            let mut data = Vec::<Sensor, 4>::new();

            data.push(Sensor::Battery {
                // turning adc reading into correct AA battery percentage
                level: ((max(min(bat_pin.analog_read(&mut adc), 600), 300) - 300) / 3)
                    .try_into()
                    .unwrap(),
            })
            .unwrap();

            // Send sensor data
            let data: Vec<u8, 32> = postcard::to_vec(&MessageData::SensorData(data)).unwrap();
            radio = radio.send(&data);
        }

        // Listen for commands
        {
            let mut bytes = Vec::new();
            radio = radio.receive(&mut bytes);
            while bytes.len() > 0 {
                // If commands heard, answer them
                {
                    // Answer command
                    let message: MessageData = postcard::from_bytes(&bytes).unwrap();
                    if let MessageData::Command(command) = message {
                        let data = match command {
                            Command::Info => {
                                postcard::to_vec::<_, 32>(&node_info).unwrap()
                            }
                        };
                        radio = radio.send(&data);
                    }
                }
                // Listen again
                bytes.clear();
                radio = radio.receive(&mut bytes);
            }
        }

        delay_ms(2000); // Should be sleep instead with watchdog
    }
}
