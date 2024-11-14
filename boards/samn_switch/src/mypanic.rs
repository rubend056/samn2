// Either we use mypanic, or we use panic_serial

// use arduino_hal::prelude::_unwrap_infallible_UnwrapInfallible;
// extern  crate panic_serial;
// panic_serial::impl_panic_handler!(
//     // This is the type of the UART port to use for printing the message:
//     arduino_hal::usart::Usart<
//       arduino_hal::pac::USART0,
//       arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
//       arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
//     >
// );
// /// DO NOT intialize Serial after this
// pub fn maybe_init_serial_panic(){
//     let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
// 	let pins = arduino_hal::pins!(dp);
// 	let serial = arduino_hal::default_serial!(dp, pins, 57600);
// 	// this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
// 	let mut serial = share_serial_port_with_panic(serial);
//     ufmt::uwriteln!(&mut serial, "Panic Initialized").unwrap_infallible();
// }

use crate::acknowlege_and_disable_watchdog;
use arduino_hal::delay_ms;
// use heapless::String;
// use ufmt::uwrite;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

// static mut RADIO: Option<
// 	samn_common::nrf24::NRF24L01<
// 		core::convert::Infallible,
// 		Pin<Output, PD6>,
// 		embedded_hal_bus::spi::ExclusiveDevice<
// 			arduino_hal::Spi<arduino_hal::hal::Atmega, arduino_hal::pac::SPI0, PB5, PB3, PB4, PB2>,
// 			Pin<Output, PD7>,
// 			arduino_hal::hal::delay::Delay<arduino_hal::clock::MHz8>,
// 		>,
// 	>,
// > = None;
// struct CharArrayWriter<'a> {
// 	buf: &'a mut [char],
// 	pos: usize,
// }

// impl<'a> ufmt::uWrite for CharArrayWriter<'a> {
// 	type Error = core::fmt::Error;

// 	fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
// 			for c in s.chars() {
// 					if self.pos >= self.buf.len() {
// 							return Err(core::fmt::Error);
// 					}
// 					self.buf[self.pos] = c;
// 					self.pos += 1;
// 			}
// 			Ok(())
// 	}
// }

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
	// Disable interrupts
	avr_device::interrupt::disable();
	acknowlege_and_disable_watchdog();

	// We're testing debugging stuff
	if let Some(loc) =_info.location(){
		// if loc.file().len() > 0 {
		// 	loop {}
		// }
		// let mut s = String::<20>::new();
		// uwrite!()
		// uwrite!(&mut s, "{}", loc.file()); 
		// if loc.file().len() > 10 {
		// 	loop{atomic::compiler_fence(Ordering::SeqCst);}
		// }
		
	}
	// if _info.message().as_str().map(|x|x.len()>10).unwrap_or(true){
	// 	loop{atomic::compiler_fence(Ordering::SeqCst);}
	// }

	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };

	let pins = arduino_hal::pins!(dp);
	let mut led = pins.led.into_output();

	loop {
		atomic::compiler_fence(Ordering::SeqCst);
		led.toggle();
		delay_ms(500);
		led.toggle();
		delay_ms(500);
	}
}
pub fn maybe_init_serial_panic() {}
