/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

use avr_device::interrupt;
use panic_halt as _;
use uno::millis::{millis, millis_init};

const RAMSTART: u16 = 0x0100; // Start of SRAM
const RAMEND: u16 = 0x08FF; // End of SRAM
#[inline(never)]
fn free_stack() -> u16 {
	let spl: u8;
	let sph: u8;
	unsafe {
		core::arch::asm!(
			"in {0}, __SP_L__", // Read SP Low byte into {0}
			"in {1}, __SP_H__", // Read SP High byte into {1}
			out(reg)(spl),      // Output operand {0}
			out(reg)(sph),      // Output operand {1}
		);
	}
	(RAMEND - RAMSTART) - (RAMEND - (((sph as u16) << 8) | (spl as u16)))
}

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);

	let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
  ufmt::uwriteln!(&mut serial, "free_stack: {}\r", free_stack()).unwrap();


	// Digital pin 13 is also connected to an onboard LED marked "L"
	let mut led = pins.d13.into_output();
	led.set_high();

  millis_init(dp.TC0);
  unsafe {interrupt::enable();}

	loop {
    ufmt::uwriteln!(&mut serial, "{}ms\r", millis()).unwrap();
    
		led.toggle();
		arduino_hal::delay_ms(100);
		led.toggle();
		arduino_hal::delay_ms(100);
		led.toggle();
		arduino_hal::delay_ms(100);
		led.toggle();
		arduino_hal::delay_ms(700);
	}
}
