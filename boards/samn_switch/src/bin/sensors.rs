/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::{self, RefCell};

use arduino_hal::{
	adc, delay_ms,
	hal::{port::PC1, spi},
	i2c::Direction,
	port::{mode::Analog, Pin},
	Delay,
};
use avr_device::interrupt::{self, Mutex};
use embedded_hal::{i2c::I2c, spi::Mode};
use samn_common::{
	node::Message,
	nrf24::NRF24L01,
	radio::{addr_to_nrf24_hq_pipe, addr_to_rx_pipe, Payload, Radio},
};
use samn_switch::{acknowlege_and_disable_watchdog, mypanic::maybe_init_serial_panic, LED_OFF_MS};

// Shared state for ADC readings and control logic
static I_READINGS_AVERAGE: Mutex<RefCell<u16>> = Mutex::new(RefCell::new(0));
static I_READINGS: Mutex<RefCell<[u16; 16]>> = Mutex::new(RefCell::new([0; 16]));
static READING_INDEX: Mutex<RefCell<u8>> = Mutex::new(RefCell::new(0));

// Shared static for the ADC and pin
static ADC: Mutex<RefCell<Option<adc::Adc>>> = Mutex::new(RefCell::new(None));
static ISENSE_PIN: Mutex<RefCell<Option<Pin<Analog, PC1>>>> = Mutex::new(RefCell::new(None));

// Helper function to calculate the average of the last 16 ADC readings
fn average_adc_readings() -> u16 {
	interrupt::free(|cs| {
		let readings = I_READINGS_AVERAGE.borrow(cs).borrow();
		*readings
	})
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
	interrupt::free(|cs| {
		// Borrow the ADC and pin
		let mut adc_ref = ADC.borrow(cs).borrow_mut();
		let mut pin_ref = ISENSE_PIN.borrow(cs).borrow_mut();

		if let (Some(adc), Some(isense_pin)) = (adc_ref.as_mut(), pin_ref.as_mut()) {
			// Perform an ADC reading
			match adc.read_nonblocking(isense_pin) {
				Ok(adc_value) => {
					// Start the next reading
					adc.read_nonblocking(isense_pin).ok();

					// Access the ADC readings and current index
					let mut readings = I_READINGS.borrow(cs).borrow_mut();
					let mut index = READING_INDEX.borrow(cs).borrow_mut();

					// Store the reading in the circular buffer
					readings[*index as usize] = adc_value;
					*index = (*index + 1) % 16; // Move to the next index, wrapping around to 0 after 15
					if *index == 0 {
						// Store the average of the readings in the longer buffer
						let mut total_average = I_READINGS_AVERAGE.borrow(cs).borrow_mut();

						let sum: u16 = readings.iter().map(|&r| (r as i16 - 511).abs() as u16).sum();
						let average = (sum / 16) as u32 * 100;
						*total_average = ((*total_average as u32 * 61 + average) / 62) as u16;
					}
				}
				_ => {}
			}
		}
	});
}

// With a clock of 8MHz a prescaler of 64 and a count of 125
// This will run every `1 / (8_000_000 / 64 / 125)` = `1ms`
const PRESCALER: u32 = 64;
const TIMER_COUNTS: u32 = 125;
fn timer_init(tc0: arduino_hal::pac::TC0) {
	// Configure the timer for the above interval (in CTC mode)
	// and enable its interrupt.
	tc0.tccr0a.write(|w| w.wgm0().ctc());
	tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
	tc0.tccr0b.write(|w| match PRESCALER {
		8 => w.cs0().prescale_8(),
		64 => w.cs0().prescale_64(),
		256 => w.cs0().prescale_256(),
		1024 => w.cs0().prescale_1024(),
		_ => panic!(),
	});
	tc0.timsk0.write(|w| w.ocie0a().set_bit());
}

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	acknowlege_and_disable_watchdog();

	// I2C Init
	let mut i2c = arduino_hal::i2c::I2c0::new(
		dp.TWI0,
		pins.sda.into_pull_up_input(),
		pins.scl.into_pull_up_input(),
		50_000,
	);

	let mut node_addr = 0x9797u16;
	let node_id = 5u32;
	maybe_init_serial_panic();

	// Led
	let mut led = pins.led.into_output();
	led.toggle();
	delay_ms(LED_OFF_MS);
	led.toggle();

	// Triac
	let mut triac = pins.triac.into_output();
	triac.set_low();
	triac.set_high();

	// Radio
	let mut radio;
	let mut irq = pins.irq.into_pull_up_input();
	{
		let (spi, _) = arduino_hal::Spi::new(
			dp.SPI0,
			pins.sck.into_output(),
			pins.mosi.into_output(),
			pins.miso.into_pull_up_input(),
			pins.ss.into_output(),
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
		radio = NRF24L01::new(pins.ce.into_output(), spi).unwrap();
		radio.configure().unwrap();
		radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();
	}

	// Isense / Vsense
	let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
	let isense_pin = pins.isense.into_analog_input(&mut adc);
	let mut _vsense = pins.vsense.into_pull_up_input();
	// Move ADC and pin to their static locations
	interrupt::free(|cs| {
		*ADC.borrow(cs).borrow_mut() = Some(adc);
		*ISENSE_PIN.borrow(cs).borrow_mut() = Some(isense_pin);
	});
	timer_init(dp.TC0);
	unsafe { avr_device::interrupt::enable() };

	fn temperature_humidity<E: core::fmt::Debug, I2C: I2c<Error = E>>(i2c: &mut I2C) -> (i16, u8) {
		const address: u8 = 0x70;
		const wakeup: [u8; 2] = [0x35, 0x17];
		i2c.write(address, &wakeup).unwrap();
		delay_ms(5);
		const normal_t_first: [u8; 2] = [0x78, 0x66];
		i2c.write(address, &normal_t_first).unwrap();

		// Wait until device is ready to read
		delay_ms(20);
		// let mut i = 0u8;
		// while i2c.ping_device(address, Direction::Read).unwrap() == false && i < u8::MAX {
		// 	delay_ms(1);
		// 	i += 1;
		// }

		let mut bytes = [0u8; 6];
		i2c.read(address, &mut bytes).unwrap();
		let temp = ((bytes[0] as u16) << 8) | (bytes[1] as u16);
		let hum = ((bytes[3] as u16) << 8) | (bytes[4] as u16);
		const sleep: [u8; 2] = [0xB0, 0x98];
		i2c.write(address, &sleep).unwrap();

		((temp / 15 * 4) as i16 - 4500, (hum / 665) as u8)
	};
	// Spits out current in milliamps (max of 20_000 mA, or 20A)
	fn average_current() -> u16 {
		let reading = average_adc_readings();
		// same as * 20_000 / 51_200
		if reading < 2_600
		// To prevent an overflow & conserve precision
		{
			reading * 25 / 64
		} else {
			reading / 64 * 25
		}
	}

	loop {
		delay_ms(1_000);

		let (temp, hum) = temperature_humidity(&mut i2c);

		let mut message = heapless::String::<20>::new();
		ufmt::uwrite!(&mut message, "{}C {}% {}mA", temp, hum, average_current()).unwrap();
		while message.len() < message.capacity() {
			message.push(' ').unwrap();
		} // Fill it up

		radio
			.transmit(&Payload::new_with_addr(
				&postcard::to_vec::<Message, 32>(&Message::DebugMessage(
					node_id,
					message.into_bytes().into_array().unwrap(),
				))
				.unwrap(),
				node_addr,
				addr_to_nrf24_hq_pipe(node_addr),
			))
			.unwrap();
	}
}
