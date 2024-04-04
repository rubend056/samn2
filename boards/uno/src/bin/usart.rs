/*!
 * Demonstration of writing to and reading from the serial console.
 */
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell;

use arduino_hal::{delay_ms, prelude::*};
use heapless::Vec;
use panic_serial as _;

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

    millis_init(dp.TC0);
    unsafe { avr_device::interrupt::enable() };

    ufmt::uwriteln!(&mut serial, "Hello from Arduino!\r").unwrap_infallible();
    let mut last = millis();
    let mut b = Vec::<u8,120>::new();
    loop {
        // Read a byte from the serial connection
        if let Ok(byte) = serial.read() {
            b.push(byte).unwrap();
            last = millis();
        }
        if millis() - last > 1000 {
            for b in &b {
                ufmt::uwriteln!(&mut serial, "Got {}!\r", b).unwrap_infallible();
            }
            b.clear();
        }
        // ufmt::uwriteln!(&mut serial, "millis {}!\r", millis()).unwrap_infallible();
        // delay_ms(100)
    }
}







const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = 125;
const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000; // Overflows in 19 hours

static MILLIS_COUNTER: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));
// static mut MILLIS_STARTED: bool = false;

pub fn millis_init(tc0: arduino_hal::pac::TC0) {
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

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
    // unsafe {MILLIS_STARTED = true};
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

pub fn millis() -> u32 {
    // if unsafe { !MILLIS_STARTED } {panic!("Using millis without init");}
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}
