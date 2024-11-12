/*!
 * Blink the builtin LED - the "Hello World" of embedded programming.
 */
#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

// use panic_halt as _;

use samn_switch::mypanic::maybe_init_serial_panic;

const RAMSTART: u16 = 0x0100; // Start of SRAM
const RAMEND: u16 = 0x08FF;   // End of SRAM
#[inline(never)]
fn get_stack_pointer() -> u16 {
    let spl: u8;
    let sph: u8;
    unsafe {
        core::arch::asm!(
            "in {0}, __SP_L__",   // Read SP Low byte into {0}
            "in {1}, __SP_H__",   // Read SP High byte into {1}
            out(reg) (spl),       // Output operand {0}
            out(reg) (sph),       // Output operand {1}
        );
    }
    ((sph as u16) << 8) | (spl as u16)
}



#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    maybe_init_serial_panic();

    // Digital pin 13 is also connected to an onboard LED marked "L"
    let mut led = pins.led.into_output();
    led.set_high();


    loop {
        led.toggle();
        arduino_hal::delay_ms(100);
        led.toggle();
        arduino_hal::delay_ms(100);
        led.toggle();
        arduino_hal::delay_ms(100);
        led.toggle();
        arduino_hal::delay_ms(300);

        panic!("hello");
    }
}
