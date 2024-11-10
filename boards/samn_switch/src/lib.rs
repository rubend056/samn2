#![no_std]
#![feature(abi_avr_interrupt)]

use core::{
	cell::Cell,
	cmp::{max, min},
	fmt::Debug,
};

use arduino_hal::{delay_ms, delay_us, hal::wdt};
use avr_device::interrupt::{self, Mutex};

pub mod mypanic;

use samn_common::{
	node::{
		Actuator, Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response, Sensor, LIMBS_MAX,
	},
	radio::*,
};

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

#[avr_device::interrupt(atmega328pb)]
fn WDT() {
	// We do nothing, this is just so interrupt vector is set
	// and device doesn't self reset, just wakes up
	// return;

	// Actually we'll use this as our timer
	avr_device::interrupt::free(|cs| {
		let seconds = SECONDS.borrow(cs);
		seconds.set(seconds.get() + WDT_SECONDS_INCREASE);
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

/// Enable watchdog interrupt and enter power down mode
pub fn en_wdi_and_pd() {
	// A little stealing so we can set some low level registers
	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
	// Enable watchdog timer as an interrupt
	dp.WDT.wdtcsr.modify(|_, w| w.wdie().set_bit());
	// Enable sleep, set mode to power-down
	dp.CPU.smcr.modify(|_, w| w.se().set_bit().sm().pdown());
	unsafe {
		// Enable interrupts
		avr_device::interrupt::enable();
		// Sleep
		avr_device::asm::sleep();
	}
}
/// Enable watchdog interrupt and enter power down mode
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

// #[avr_device::interrupt(atmega328pb)]
// fn INT1() { // This corresponds to the irq pin on nrf24
// 	          // We do nothing, this is just so interrupt vector is set
// }
// pub fn setup_int1() {
//     // Configure INT1 for falling edge. 0x03 would be rising edge.
//     dp.EXINT.eicra.modify(|_, w| w.isc1().bits(0x02));
//     // Enable the INT1 interrupt source.
//     dp.EXINT.eimsk.modify(|_, w| w.int1().set_bit());
// }
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

pub fn check_for_payloads_for_a_bit<E: Debug, R: Radio<E>, P: embedded_hal::digital::InputPin>(
	radio: &mut R,
	irq: &mut P,
) -> Option<Payload> {
	radio.to_rx().unwrap();

	// >= 150 ms wait
	for _ in 0..u8::MAX {
		if let Ok(payload) = radio.receive(irq, None) {
			// radio.to_idle().unwrap();
			return Some(payload);
		}
		delay_us(500);
	}
	// radio.to_idle().unwrap();
	None
}
