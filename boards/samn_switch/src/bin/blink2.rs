/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut led = pins.led.into_output();
    led.set_low();
    let mut triac = pins.triac.into_output();
    triac.set_low();

    loop {
        arduino_hal::delay_ms(10_000);
        led.toggle();
        triac.toggle();
    }
}
