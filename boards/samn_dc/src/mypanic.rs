
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

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use arduino_hal::delay_ms;
use crate::acknowlege_and_disable_watchdog;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
	// Disable interrupts
	avr_device::interrupt::disable();
	acknowlege_and_disable_watchdog();

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
pub fn maybe_init_serial_panic(){}