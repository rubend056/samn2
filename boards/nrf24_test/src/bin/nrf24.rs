/*!
 * Testing nrf24 with just nrf24 crate
 */
#![no_std]
#![no_main]

use core::convert::Infallible;

use arduino_hal::{delay_ms, Delay};
use embedded_hal::{
	digital::OutputPin,
	spi::{self, SpiDevice},
};
use embedded_hal_bus::spi::DeviceError;
use nrf24::NRF24L01;
use samn9_test::{acknowlege_and_disable_watchdog, mypanic::maybe_init_serial_panic};

/// Our error implementation
///
/// We just implement the known type of Infallible
enum Error {
	Nrf24(nrf24::Error<DeviceError<Infallible, Infallible>, Infallible>),
}
impl From<nrf24::Error<DeviceError<Infallible, Infallible>, Infallible>> for Error {
	fn from(value: nrf24::Error<DeviceError<Infallible, Infallible>, Infallible>) -> Self {
		Self::Nrf24(value)
	}
}
impl Error {
	pub fn discriminant(&self) -> u8 {
		match self {
			Error::Nrf24(err) => err.discriminant(),
		}
	}
}

const PRIM_RX: bool = true;

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	acknowlege_and_disable_watchdog();

	// Digital pin 13 is also connected to an onboard LED marked "L"
	let mut led = pins.led.into_output();
	led.set_high();
	delay_ms(50);
	led.set_low();

	maybe_init_serial_panic();

	let (spi, _) = arduino_hal::Spi::new(
		dp.SPI0,
		pins.b5.into_output(),
		pins.b3.into_output(),
		pins.b4.into_pull_up_input(),
		pins.b2.into_output(),
		arduino_hal::spi::Settings {
			mode: embedded_hal::spi::MODE_0,
			clock: arduino_hal::spi::SerialClockRate::OscfOver2,
			..Default::default()
		},
	);
	let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());

	let mut nrf24 = nrf24::NRF24L01::new(pins.g2_ce.into_output(), spi);

	fn inner_loop<
		A: SpiDevice<Error = DeviceError<Infallible, Infallible>>,
		B: OutputPin<Error = Infallible>,
		C: OutputPin<Error = Infallible>,
	>(
		nrf24: &mut NRF24L01<A, B>,
		led: &mut C,
	) -> Result<(), Error> {
		// We initialize the radio
		nrf24.initialize(&mut Delay::new())?;
		nrf24.configure()?;
		let a_addr = b"nasal";
		let b_addr = b"bohem";
		let p = [true, true, false, false, false, false];
		nrf24.set_auto_ack_pipes(&p)?;
		nrf24.set_rx_enabled_pipes(&p)?;
		nrf24.set_dynamic_payload_pipes(&p)?;
		
		const DELAY: u16 = 100;

		if PRIM_RX {
			nrf24.rx()?;
			nrf24.set_rx_addr(0, a_addr)?;
			nrf24.set_rx_addr(1, b_addr)?;

			loop {
				if let Some((pipe_n, payload)) = nrf24.receive_maybe()? {
					let mut ok = false;
					let bytes = b"test";
					if payload[..bytes.len()] == *bytes {
						ok = true;
					}
					let blinks: u8 = if ok { 1 } else { 2 };

					// let blinks: u8 = payload_l;

					for _ in 0..blinks {
						led.set_high().ok();
						delay_ms(DELAY / 4);
						led.set_low().ok();
						delay_ms(DELAY / 2);
					}
				}
				delay_ms(20);
			}
		} else {
			nrf24.tx()?;
			let mut a_else_b:bool = false;
			loop {
				// Pick an address to send to
				a_else_b = !a_else_b;
				let tx_addr = if a_else_b {a_addr} else {b_addr};
				nrf24.set_tx_addr(tx_addr)?;
				nrf24.set_rx_addr(0, tx_addr)?;

				// Then send a message
				nrf24.transmission_start(b"test", nrf24::PayloadType::Payload, &mut Delay::new())?;

				// Wait for transmission
				let mut ok = false;
				let mut max_wait = false;
				for i in 0..=u8::MAX {
					match nrf24.transmission_ended() {
						nb::Result::Ok(v) => {
							ok = v;
							break;
						}
						nb::Result::Err(nb::Error::Other(err)) => {
							return Err(err.into());
						}
						nb::Result::Err(nb::Error::WouldBlock) => {}
					}
					delay_ms(1);
					if i == u8::MAX {
						max_wait = true;
					}
				}

				// Blink once for sent, twice for not sent
				let blinks: u8 = if max_wait {
					3
				} else {
					if ok {
						1
					} else {
						2
					}
				};
				for _ in 0..blinks {
					led.set_high().ok();
					delay_ms(DELAY);
					led.set_low().ok();
					delay_ms(DELAY);
				}
				delay_ms(DELAY * 10);

				// if ok {
				// 	led.set_high().ok();
				// 	delay_ms(DELAY);
				// 	led.set_low().ok();
				// }
				// delay_ms(DELAY);
			}
		}
	}

	// Outer loop, catches errors and blinks the led to show them
	loop {
		if let Err(err) = inner_loop(&mut nrf24, &mut led) {
			const DELAY: u16 = 500;
			// Blink as many times as the discriminant
			for _ in 0..err.discriminant() {
				led.set_high();
				delay_ms(DELAY);
				led.set_low();
				delay_ms(DELAY);
			}
			delay_ms(DELAY * 5);
		}
	}
}
