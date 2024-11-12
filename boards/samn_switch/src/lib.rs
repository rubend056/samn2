#![no_std]
#![feature(abi_avr_interrupt)]

use core::{cell::Cell, fmt::Debug, sync::atomic::AtomicBool};

use arduino_hal::{delay_us, hal::wdt};
use avr_device::interrupt::{self, Mutex};

pub mod mypanic;

use samn_common::{node::Message, radio::*};

pub const WATCHDOG_TIMEOUT: wdt::Timeout = wdt::Timeout::Ms8000;
pub const WDT_SECONDS_INCREASE: u32 = 8;
pub const SEARCH_NETWORK_INTERVAL: u32 = 8;
pub const HEARTBEAT_INTERVAL: u16 = 60;
pub const LED_ON_MS: u16 = 50;
pub const LED_OFF_MS: u16 = 50;

/// Compares only the Enum types, not the values
pub fn variant_eq<T>(a: &T, b: &T) -> bool {
	core::mem::discriminant(a) == core::mem::discriminant(b)
}

/// This is able to count up to 136 years.
static SECONDS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
pub static WDT_TRIGGERED: AtomicBool = AtomicBool::new(false);

#[avr_device::interrupt(atmega328pb)]
fn WDT() {
	avr_device::interrupt::free(|cs| {
		let seconds = SECONDS.borrow(cs);
		seconds.set(seconds.get() + WDT_SECONDS_INCREASE);
		WDT_TRIGGERED.store(true, core::sync::atomic::Ordering::Relaxed);
	})
}

/// Only do this after Peripherals::take, or take will Panic, because this function steals.
pub fn acknowlege_and_disable_watchdog() {
	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
	// Clear watchdog flag
	dp.CPU.mcusr.modify(|_, s| s.wdrf().clear_bit());
	// Write logical one to WDCE and WDE
	dp.WDT.wdtcsr.modify(|_, w| w.wdce().set_bit().wde().set_bit());
	// Turn off watchdog
	dp.WDT.wdtcsr.write(|w| unsafe { w.bits(0) });
}

pub fn now() -> u32 {
	interrupt::free(|cs| SECONDS.borrow(cs).get())
}

/// Enable watchdog interrupt and go idle
pub fn en_wdi_and_idle() {
	// A little stealing so we can set some low level registers
	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
	// Enable watchdog timer as an interrupt
	dp.WDT.wdtcsr.modify(|_, w| w.wdie().set_bit());
	// Enable sleep, set mode to idle
	dp.CPU.smcr.modify(|_, w| w.se().set_bit().sm().idle());
	unsafe {
		// Enable interrupts
		avr_device::interrupt::enable();
		// Sleep
		avr_device::asm::sleep();
	}
}

pub fn check_for_messages_for_a_bit<E: Debug, R: Radio<E>, P: embedded_hal::digital::InputPin>(
	radio: &mut R,
	irq: &mut P,
) -> Option<Message> {
	radio.to_rx().unwrap();

	// >= 150 ms wait
	for _ in 0..u8::MAX {
		if let Ok(message) = radio
			.receive(irq, None)
			.map(|payload| postcard::from_bytes::<Message>(payload.data()).unwrap())
		{
			// radio.to_idle().unwrap();
			return Some(message);
		}
		delay_us(500);
	}
	// radio.to_idle().unwrap();
	None
}
