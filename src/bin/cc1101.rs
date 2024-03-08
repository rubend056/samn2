/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]

use arduino_hal::{spi, Delay};

use cc1101::Cc1101;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Create SPI interface.
    let (mut spi, _) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        spi::Settings::default(),
    );
    let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.d7.into_output(), Delay::new());

    let mut cc1101 = Cc1101::new(spi).unwrap();
    cc1101.get_hw_info().unwrap();

    loop {
        continue;
    }
}
