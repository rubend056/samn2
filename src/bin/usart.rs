/*!
 * Demonstration of writing to and reading from the serial console.
 */
#![no_std]
#![no_main]

use arduino_hal::prelude::*;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    ufmt::uwriteln!(&mut serial, "HI !\r").unwrap_infallible();
    let mut bytes = [0u8;10];
    let mut len = 0;
    loop {
        // Read a byte from the serial connection
        if len == 10 {
            for b in bytes[0..len].iter() {
                ufmt::uwriteln!(&mut serial, "Got {}!\r", b).unwrap_infallible();
            }
            len = 0;
        }
        match serial.read(){
            Ok(b) => {bytes[len] = b;len+=1;}
            Err(_) => if len > 0 {
                for b in bytes[0..len].iter() {
                    ufmt::uwriteln!(&mut serial, "Got {}!\r", b).unwrap_infallible();
                }
                len = 0;
            }
        }
    }
}
