#![no_std]
#![no_main]

use arduino_hal::{delay_ms, spi, Delay};
use samn9::mypanic::maybe_init_serial_panic;
use samn_common::{cc1101::Cc1101, radio::*};

use samn9::*;

static IS_A: bool = true;

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	// If we don't do this the watchdog will reset the device after
	// 16ms allowing only the next few lines of code to execute :(
	// That's why we were seeing a neverending blinking light
	// This took me 2 days to figure out
	acknowlege_and_disable_watchdog();

	// Led
	let mut led = pins.led.into_output();
	led.toggle();
	delay_ms(LED_ON_MS);
	led.toggle();

	maybe_init_serial_panic();

	// let mut node_addr = 0x8484u16;

	let mut radio;
	let mut g2 = pins.g2_ce;
	{
		let (spi, _) = arduino_hal::Spi::new(
			dp.SPI0,
			pins.b5.into_output(),
			pins.b3.into_output(),
			pins.b4.into_pull_up_input(),
			pins.b2.into_output(),
			spi::Settings {
				mode: embedded_hal::spi::MODE_0,
				clock: spi::SerialClockRate::OscfOver2,
				data_order: spi::DataOrder::MostSignificantFirst,
			},
		);
		let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());
		radio = Cc1101::new(spi).unwrap();
		radio.init(&mut arduino_hal::Delay::new()).unwrap();
		radio.set_rx_filter(&[0x84]).unwrap();
	}

	if IS_A {
		// Look for a valid Node Address
		loop {
			// let now = now();
			// if now >= last_message + SEARCH_NETWORK_INTERVAL {
			// Blink twice
			led.toggle();
			delay_ms(LED_ON_MS);
			led.toggle();

			delay_ms(LED_OFF_MS);

			// led.toggle();
			// delay_ms(LED_ON_MS);
			// led.toggle();
			// delay_ms(LED_OFF_MS);

			// Send looking for network
			Radio::transmit(&mut radio, &Payload::new_with_addr(b"ping", 0x8484, 0x84)).unwrap();

			if let Some(payload) = check_for_payloads_for_a_bit(&mut radio, &mut g2) {
				if payload.len() == 4 && payload.data() == *b"pong" {
					led.toggle();
					delay_ms(LED_OFF_MS);
					led.toggle();
				}
			}

			delay_ms(2000);
		}
	} else {
		loop {
			if let Some(payload) = check_for_payloads_for_a_bit(&mut radio, &mut g2) {
				if payload.len() == 4 && payload.data() == *b"ping" {
					Radio::transmit(&mut radio, &Payload::new_with_addr(b"pong", 0x8484, 0x84)).unwrap();

					led.toggle();
					delay_ms(LED_OFF_MS);
					led.toggle();
				}
			}
		}
	}
}
