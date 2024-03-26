#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cmp::{max, min};

use arduino_hal::{adc, delay_ms, delay_us, hal::wdt, prelude::*, spi, Delay};
use embedded_hal::{
    digital::OutputPin,
    spi::{Mode, SpiDevice},
};
use avr_device::interrupt;
use embedded_nrf24l01::{Device, NRF24L01};
use heapless::Vec;
use panic_serial as _;
// use samn2::radio::Radio;
use samn_common::{
    node::{
        Actuator, ActuatorValue, Command, MessageData, Response, Sensor, ACTUATORS_MAX, SENSORS_MAX,
    },
    radio::nrf24,
};

panic_serial::impl_panic_handler!(
    // This is the type of the UART port to use for printing the message:
    arduino_hal::usart::Usart<
      arduino_hal::pac::USART0,
      arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
      arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
    >
);

/// Compares only the Enum types, not the values
fn variant_eq<T>(a: &T, b: &T) -> bool {
    core::mem::discriminant(a) == core::mem::discriminant(b)
}

const BOARD_VERSION: u8 = 9;
const NODE_ID: u16 = 14;

#[avr_device::interrupt(atmega328pb)]
fn WDT() {
    // We do nothing, this is just so interrupt vector is set 
    // and device doesn't self reset, just wakes up
    return;
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Serial
    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);

    // Radio
    let (spi, _) = arduino_hal::Spi::new(
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
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());
    let mut radio = NRF24L01::new(pins.g2_ce.into_output(), spi).unwrap();
    nrf24::init(&mut radio);

    // Watchdog timer
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
    // dp.AC

    // Battery
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let bat_pin = pins.battery.into_analog_input(&mut adc);

    // Led
    let mut led = pins.led.into_output();

    // Actuators
    let mut actuators = Vec::<Actuator, ACTUATORS_MAX>::new();
    actuators
        .push(Actuator {
            id: 0,
            value: ActuatorValue::Light(false),
        })
        .unwrap();

    let mut reporting_interval = 10_000;
    let mut actuator_update_flag = true;

    loop {
        {
            // Read and report sensor data
            let mut data = Vec::<Sensor, SENSORS_MAX>::new();

            data.push(Sensor::Battery(
                // turning adc reading into correct AA battery percentage

                // ((max(min(bat_pin.analog_read(&mut adc), 600), 300) - 300) / 2)
                (bat_pin.analog_read(&mut adc)/4)
                    .try_into()
                    .unwrap(),
            ))
            .unwrap();

            // Send sensor data
            let data: Vec<u8, 32> =
                postcard::to_vec(&MessageData::SensorData { id: NODE_ID, data }).unwrap();

            // Indicate we are sending
            led.toggle();
            delay_ms(100);
            led.toggle();

            radio.send(&data).unwrap();
        }

        // Listen for commands for >=76ms, reset counter if command received
        {
            let mut i = 0u8;
            radio.rx().unwrap();
            while i < u8::MAX {
                if let Ok(bytes) = radio.receive() {
                    // If commands heard, answer them
                    {
                        // Answer command
                        let message: MessageData = postcard::from_bytes(&bytes).unwrap();
                        match message {
                            MessageData::Command { id, command } => {
                                let data = postcard::to_vec::<_, 32>(&MessageData::Response {
                                    id: NODE_ID,
                                    id_c: id,
                                    response: match command {
                                        Command::Info => Response::Info {
                                            board_version: BOARD_VERSION,
                                            actuators: actuators.clone(),
                                        },
                                        Command::ReportingInterval(interval) => {
                                            // reporting_interval = interval;
                                            Response::Ok
                                        }
                                        Command::SetActuator(actuator_in) => {
                                            if let Some(actuator) =
                                                actuators.iter_mut().find(|v| **v == actuator_in)
                                            {
                                                if variant_eq(&actuator.value, &actuator_in.value) {
                                                    actuator.value = actuator_in.value;
                                                    actuator_update_flag = true;
                                                    Response::Ok
                                                } else {
                                                    Response::ErrActuatorValueTypeDoesntMatch
                                                }
                                            } else {
                                                Response::ErrActuatorNotFound
                                            }
                                        }
                                        _ => Default::default(),
                                    },
                                })
                                .unwrap();
                                radio.ce_disable(); // Stop rx
                                radio.send(&data).unwrap(); // Send sets tx, sends
                                radio.rx().unwrap(); // Back to rx
                                i = 0;
                            }
                            _ => {}
                        }
                    }
                }
                i += 1;
                delay_us(300);
            }

            radio.ce_disable();
        }

        {
            // Update actuators
            if actuator_update_flag {
                {
                    let ActuatorValue::Light(v) = actuators[0].value;
                    if v {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }

                actuator_update_flag = false;
            }
        }

        
        // watchdog.start(timeout);
        
        delay_ms(reporting_interval); // Should be sleep instead with watchdog
    }
}
