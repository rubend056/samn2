#![no_std]
#![no_main]

use panic_halt as _;
use avr_device::atmega328p::PORTB;

#[arduino_hal::entry]
fn main() -> ! {
    // PORTB = PORTB;
    loop {
        continue;
    }
}