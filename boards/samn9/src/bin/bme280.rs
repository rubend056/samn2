/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::{delay_ms, i2c::I2c0};
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

	let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
	// this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
	let mut serial = share_serial_port_with_panic(serial);

	let mut i2c = I2c0::new(
		dp.TWI0,
		pins.c4.into_pull_up_input(),
		pins.c5.into_pull_up_input(),
		50_000,
	);
    
    // i2c.i2cdetect(&mut serial, arduino_hal::i2c::Direction::Read).unwrap();
    // loop {}

    const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
        config: bme280_multibus::Config::RESET
            .set_standby_time(bme280_multibus::Standby::Millis125)
            .set_filter(bme280_multibus::Filter::X8),
        ctrl_meas: bme280_multibus::CtrlMeas::RESET
            .set_osrs_t(bme280_multibus::Oversampling::X8)
            .set_osrs_p(bme280_multibus::Oversampling::X8)
            .set_mode(bme280_multibus::Mode::Normal),
        ctrl_hum: bme280_multibus::Oversampling::X8,
    };
	let mut bme280 =
		bme280_multibus::Bme280::from_i2c0(i2c, bme280_multibus::Address::SdoGnd).unwrap();
	bme280.reset().unwrap();
	delay_ms(20);
    bme280.settings(&SETTINGS).unwrap();
    delay_ms(1000);

	loop {
		let meas = bme280.sample().unwrap();
		uwriteln!(
			&mut serial,
			"Temp: {}, Hum {}, Press: {}\r",
			meas.temperature,
            meas.humidity,
            meas.pressure
		)
		.unwrap();
		delay_ms(2000);
	}
}
