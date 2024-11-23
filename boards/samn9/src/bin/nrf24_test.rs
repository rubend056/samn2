/*!
 * Testing nrf24 with samn-common implemenations
 */
#![no_std]
#![no_main]

use core::convert::Infallible;

use arduino_hal::{delay_ms, Delay};
use embedded_hal::{
	digital::{InputPin, OutputPin},
	spi::{self, SpiDevice},
};
use embedded_hal_bus::spi::DeviceError;
use samn9::{acknowlege_and_disable_watchdog, mypanic::maybe_init_serial_panic};
use samn_common::{
	errors::Discriminant,
	nrf24::{self, NRF24L01},
	radio::{
		helper::{self, send_payload},
		Payload, Radio,
	},
};

type Nrf24error = nrf24::Error<DeviceError<Infallible, Infallible>, Infallible>;
/// Our error implementation
///
/// We just implement the known type of Infallible
enum Error {
	HelperError(helper::Error<Nrf24error>),
}
impl From<Nrf24error> for Error {
	fn from(value: Nrf24error) -> Self {
		Self::HelperError(helper::Error::RadioError(value))
	}
}
impl From<helper::Error<Nrf24error>> for Error {
	fn from(value: helper::Error<Nrf24error>) -> Self {
		Self::HelperError(value)
	}
}
impl Error {
	pub fn discriminant(&self) -> u8 {
		match self {
			Error::HelperError(err) => err.discriminant(),
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

	let mut irq = pins.g0_irq.into_pull_up_input();

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
		I: InputPin<Error = Infallible>,
	>(
		nrf24: &mut NRF24L01<A, B>,
		led: &mut C,
		irq: &mut I,
	) -> Result<(), Error> {
		// We initialize the radio
		nrf24.init(&mut Delay::new())?;
		let pipes = [b'a', b'_'];

		const DELAY: u16 = 100;

		if PRIM_RX {
			nrf24.rx()?;
			nrf24.set_rx_filter(&pipes)?;

			loop {
				if let Ok(payload) = nrf24.receive(irq, None) {
					let ok = payload.data() == *b"test";
					let blinks: u8 = if ok { 1 } else { 2 };

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
			let mut a_else_b: bool = false;
			loop {
				// Pick an address to send to
				a_else_b = !a_else_b;
				let tx_pipe = if a_else_b { pipes[0] } else { pipes[1] };

				let send_result = send_payload(
					nrf24,
					&Payload::new_with_addr(b"test", 0x55ae, tx_pipe),
					&mut Delay::new(),
				)?;

				// Blink once for sent, twice for not sent
				let blinks: u8 = if send_result { 1 } else { 2 };

				for _ in 0..blinks {
					led.set_high().ok();
					delay_ms(DELAY);
					led.set_low().ok();
					delay_ms(DELAY);
				}
				delay_ms(DELAY * 10);
			}
		}
	}

	// Outer loop, catches errors and blinks the led to show them
	loop {
		if let Err(err) = inner_loop(&mut nrf24, &mut led, &mut irq) {
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
