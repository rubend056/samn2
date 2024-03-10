#![no_std]
#![no_main]

use arduino_hal::{delay_ms, prelude::*, spi, Delay};
use embedded_hal::spi::{Mode,SpiDevice};
use panic_serial as _;

panic_serial::impl_panic_handler!(
    // This is the type of the UART port to use for printing the message:
    arduino_hal::usart::Usart<
      arduino_hal::pac::USART0,
      arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
      arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
    >
);

const TX: bool = true;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
    let mut serial = share_serial_port_with_panic(serial);

    let (mut spi, _) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        spi::Settings {
            mode: embedded_hal::spi::MODE_0,
            clock: spi::SerialClockRate::OscfOver2,
            data_order: spi::DataOrder::MostSignificantFirst
        },
    );

    let mut spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.d7.into_output(), Delay::new());

    let mut ce = pins.d6.into_output();
    ce.set_low();
    delay_ms(1);
    
    let mut a = |i| {
        let w = [i;1];
        let mut r = [0x00;2];
        spi.transfer(&mut r, &w).unwrap();

        ufmt::uwriteln!(&mut serial, "{} -> {} {}\r", w[0],r[0],r[1]).unwrap_infallible();
    };
    
    for i in 0..17u8 {
        a(i);
    }
    a(0x1D);
    
    loop {continue;}
}
