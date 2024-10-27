#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::sync::atomic::{AtomicBool, Ordering};
/// This is able to count up to 136 years.
// static BUTTON_PRESSED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
    BUTTON_PRESSED.store(true, Ordering::SeqCst);
}


#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	// If we don't do this the watchdog will reset the device after
	// 16ms allowing only the next few lines of code to execute :(
	// That's why we were seeing a neverending blinking light
	// This took me 2 days to figure out
	// Only do this after Peripherals::take, or take will Panic
	acknowlege_and_disable_watchdog();

  loop {
    
  }
}