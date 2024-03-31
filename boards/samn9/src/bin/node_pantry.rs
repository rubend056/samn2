#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::{
    cell::Cell,
    cmp::{max, min},
};

use arduino_hal::{adc, delay_ms, delay_us, hal::wdt, prelude::*, spi, Delay};
use avr_device::interrupt::{self, Mutex};
use embedded_hal::{
    digital::OutputPin,
    spi::{Mode, SpiDevice},
};
use embedded_nrf24l01::{Device, NRF24L01};
use heapless::{String, Vec};
use panic_serial as _;
// use samn2::radio::Radio;
use samn_common::{
    node::{
        Actuator, Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response,
        Sensor, LIMBS_MAX,
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

/// This is able to count up to 136 years.
static SECONDS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[avr_device::interrupt(atmega328pb)]
fn WDT() {
    // We do nothing, this is just so interrupt vector is set
    // and device doesn't self reset, just wakes up
    // return;

    // Actually we'll use this as our timer
    avr_device::interrupt::free(|cs| {
        let seconds = SECONDS.borrow(cs);
        seconds.set(seconds.get() + 1);
    })
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    const NODE_ID: u16 = 22;
    let node_info: NodeInfo = NodeInfo {
        name: "Pantry".into(),
        board: Board::SamnV9,
    };

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
    let spi =
        embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());
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

    let mut limbs = Limbs::new();
    limbs
        .push(Limb(
            0,
            LimbType::Sensor {
                report_interval: 60,
                data: None,
            },
        ))
        .unwrap();
    limbs
        .push(Limb(1, LimbType::Actuator(Actuator::Light(false))))
        .unwrap();

    // Sensor Id -> last_report_time
    let mut last_heartbeat: u32 = 0;
    let mut sensor_last_report = heapless::LinearMap::<_, _, LIMBS_MAX>::new();
    // Should we update the actuators
    let mut actuator_update_flag = true;

    loop {
        {
            let now = interrupt::free(|cs| SECONDS.borrow(cs).get());
            // Here we check if sensors need to report anything
            // Depending on the report interval of each sensor
            let mut limbs_reporting = Limbs::new();

            for limb in limbs.iter_mut() {
                if let LimbType::Sensor {
                    report_interval,
                    data,
                } = &mut limb.1
                {
                    // Check if it's our time to report, using report_interval
                    if sensor_last_report
                        .get(&limb.0)
                        .map(|last| now >= *last + (*report_interval as u32))
                        .unwrap_or(true)
                    {
                        // Set limb ID 0 as Battery
                        if limb.0 == 0 {
                            *data = Some(Sensor::Battery(
                                // turning adc reading into correct AA battery percentage
                                ((max(min(bat_pin.analog_read(&mut adc), 600), 300) - 300) / 2)
                                    // (bat_pin.analog_read(&mut adc)/4)
                                    .try_into()
                                    .unwrap(),
                            ));
                        }
                        limbs_reporting.push(limb.clone()).unwrap();
                        sensor_last_report.insert(limb.0, now).unwrap();
                    }
                }
            }

            // Only send if there's any sensor to report
            // Or we have a hearbeat due
            if limbs_reporting.len() > 0 || now >= last_heartbeat + 10 {
                let message = Message {
                    id: NODE_ID,
                    data: MessageData::Response {
                        id: None,
                        response: if limbs_reporting.len() > 0 {
                            Response::Limbs(limbs_reporting)
                        } else {
                            Response::Heartbeat(now)
                        },
                    },
                };
                let data: Vec<u8, 32> = postcard::to_vec(&message).unwrap();

                // Indicate we are sending
                led.toggle();
                delay_ms(100);
                led.toggle();

                radio.send(&data).unwrap();
                last_heartbeat = now;
            }
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
                        let message: Message = postcard::from_bytes(&bytes).unwrap();
                        match message.data {
                            MessageData::Command { id, command } => {
                                let data = postcard::to_vec::<_, 32>(&Message {
                                    id: NODE_ID,
                                    data: MessageData::Response {
                                        id: Some(id),
                                        response: match command {
                                            Command::Info => Response::Info(node_info.clone()),
                                            Command::Limbs => Response::Limbs(limbs.clone()),
                                            Command::SetLimb(limb_in) => {
                                                if let Some(limb) = limbs
                                                    .iter_mut()
                                                    .find(|limb| limb.0 == limb_in.0)
                                                {
                                                    if variant_eq(&limb.1, &limb_in.1) {
                                                        match (&mut limb.1, limb_in.1) {
                                                            (
                                                                LimbType::Actuator(actuator),
                                                                LimbType::Actuator(actuator_in),
                                                            ) => {
                                                                if variant_eq(
                                                                    actuator,
                                                                    &actuator_in,
                                                                ) {
                                                                    *actuator = actuator_in;
                                                                    actuator_update_flag = true;
                                                                    Response::Ok
                                                                } else {
                                                                    Response::ErrLimbTypeDoesntMatch
                                                                }
                                                            }

                                                            (
                                                                LimbType::Sensor {
                                                                    report_interval,
                                                                    data: _,
                                                                },
                                                                LimbType::Sensor {
                                                                    report_interval:
                                                                        report_interval_in,
                                                                    data: _,
                                                                },
                                                            ) => {
                                                                *report_interval =
                                                                    report_interval_in;
                                                                Response::Ok
                                                            }
                                                            _ => Response::ErrLimbTypeDoesntMatch,
                                                        }
                                                    } else {
                                                        Response::ErrLimbTypeDoesntMatch
                                                    }
                                                } else {
                                                    Response::ErrLimbNotFound
                                                }
                                            }
                                            _ => Default::default(),
                                        },
                                    },
                                })
                                .unwrap();
                                radio.ce_disable(); // Stop rx
                                radio.send(&data).unwrap(); // Send sets tx, sends
                                radio.rx().unwrap(); // Back to rx
                                i = 0; // Reset timeout counter
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
                    for limb in limbs.iter() {
                        if let LimbType::Actuator(actuator) = &limb.1 {
                            match actuator {
                                Actuator::Light(value) => {
                                    if limb.0 == 1 {
                                        if *value {
                                            led.set_high();
                                        } else {
                                            led.set_low();
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }

                actuator_update_flag = false;
            }
        }

        // Start watchdog, this also resets it :)
        watchdog.start(wdt::Timeout::Ms1000).unwrap();
        {
            // A little stealing so we can set some low level registers
            let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
            // Enable watchdog timer as an interrupt
            dp.WDT.wdtcsr.modify(|_, w| w.wdie().set_bit());
            // Enable sleep, set mode to power-down
            dp.CPU.smcr.modify(|_, w| w.se().set_bit().sm().pdown());
            unsafe {
                // Enable interrupts
                avr_device::interrupt::enable();
                // Sleep
                avr_device::asm::sleep();
            }
        }

        // delay_ms(reporting_interval); // Should be sleep instead with watchdog
    }
}
