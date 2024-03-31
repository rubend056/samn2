/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::delay_ms;
use dht11::Dht11;
use panic_serial as _;
use ufmt::uwriteln;

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

	// Open drain pin compatibible with HAL
	let pin = pins.b0.into_opendrain();

	// SysTick-based HAL `Delay` on Cortex-M
	let mut delay = arduino_hal::Delay::new();

	let mut dht11 = Dht11::new(pin);

	delay_ms(2000);

	loop {
		let meas = dht11.perform_measurement(&mut delay).unwrap();
		uwriteln!(&mut serial, "Temp: {} Hum: {}", meas.temperature, meas.humidity).unwrap();
		delay_ms(2000);
	}
}
