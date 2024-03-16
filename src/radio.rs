use crate::{millis::millis};
use core::fmt::Debug;
use arduino_hal::delay_us;

use cc1101::Cc1101;
use embedded_hal::{digital::OutputPin, spi::SpiDevice};
use embedded_nrf24l01::{StandbyMode, NRF24L01};
use heapless::Vec;
use samn_common::radio::{cc1101::config_1};

/// Implements common functionality for wether nodes have
/// nrf24 or cc1101 radios.
/// 
/// Sending can be done imperatively and blocking.
/// Receiving can also be done blockingly.
pub trait Radio {
  /// Sets up the radio with common settings
  fn init(&mut self);
  /// Send a message
  /// On the nrf24 this will be done with ACK,
  /// On the cc1101 this will be done blidndly.
  fn send(self, bytes: &[u8]) -> Self;
  /// Receive a message;
  /// We put the radio in receive mode and listen for a bit (50ms).
  fn receive(self, bytes: &mut Vec::<u8, 32>) -> Self;
}

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug> Radio
    for StandbyMode<NRF24L01<E, CE, SPI>>
{
    fn init(&mut self) {
        samn_common::radio::nrf24::init (self);
    }
    fn send(self, bytes: &[u8]) -> Self {
        let mut tx = self.tx().unwrap();
        if tx.can_send().unwrap() {
            tx.send(bytes).unwrap();
            nb::block!(tx.poll_send()).unwrap();
        }
        tx.standby().unwrap()
    }
    fn receive(self, bytes: &mut Vec<u8, 32>) -> Self {
        let mut rx = self.rx().unwrap();

        let t0 = millis();
        loop {
            if let Some(_) = rx.can_read().unwrap() {
                if let Ok(buf) = rx.read() {
                    bytes.extend(buf);
                    break;
                }
            }
            if millis() - t0 > 50 {break;}
            delay_us(200);
        }

        rx.standby()
    }
}



impl<T: SpiDevice> Radio for Cc1101<T> {
    fn init(&mut self) {
        config_1(self);
    }
    fn send(mut self, bytes: &[u8]) -> Self {
        self.transmit(bytes).unwrap();
        self
    }
    fn receive(mut self, bytes: &mut Vec<u8, 32>) -> Self {
        self.set_radio_mode(cc1101::RadioMode::Receive).unwrap();

        let t0 = millis();
        loop {
            if self.rx_bytes_available_once().unwrap() > 0 {
                bytes.resize_default(bytes.capacity()).unwrap();
                let len = Cc1101::receive(&mut self, bytes).unwrap();
                bytes.truncate(len as usize);
                break;
            }

            if millis() - t0 > 50 {
                break;
            }
            delay_us(200);
        }

        self
    }
}