#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::{
	cell::Cell,
	cmp::{max, min},
	fmt::Debug,
};

use arduino_hal::{delay_ms, delay_us, hal::wdt, spi, Delay};
use avr_device::interrupt::{self, Mutex};
use embedded_hal::spi::Mode;
use samn_common::nrf24::NRF24L01;
// use panic_serial as _;
// extern crate panic_serial;
// extern crate panic_halt;

use samn_common::{
	node::{
		Actuator, Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response, Sensor, LIMBS_MAX,
	},
	radio::*,
};

const WATCHDOG_TIMEOUT: wdt::Timeout = wdt::Timeout::Ms8000;
const WDT_SECONDS_INCREASE: u32 = 8;
const SEARCH_NETWORK_INTERVAL: u32 = 8;
const HEARTBEAT_INTERVAL: u16 = 60;

// panic_serial::impl_panic_handler!(
//     // This is the type of the UART port to use for printing the message:
//     arduino_hal::usart::Usart<
//       arduino_hal::pac::USART0,
//       arduino_hal::port::Pin<arduino_hal::port::mode::Input, arduino_hal::hal::port::PD0>,
//       arduino_hal::port::Pin<arduino_hal::port::mode::Output, arduino_hal::hal::port::PD1>
//     >
// );

/// Compares only the Enum types, not the values
fn variant_eq<T>(a: &T, b: &T) -> bool {
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
#[avr_device::interrupt(atmega328pb)]
fn INT1() { // This corresponds to the irq pin on nrf24
	          // We do nothing, this is just so interrupt vector is set
}

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

fn acknowlege_and_disable_watchdog() {
	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
	// Clear watchdog flag
	dp.CPU.mcusr.modify(|_, s| s.wdrf().clear_bit());
	// Write logical one to WDCE and WDE
	dp.WDT.wdtcsr.modify(|_, w| w.wdce().set_bit().wde().set_bit());
	// Turn off watchdog
	dp.WDT.wdtcsr.write(|w| unsafe { w.bits(0) });
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
	// Disable interrupts
	avr_device::interrupt::disable();
	acknowlege_and_disable_watchdog();

	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };
	let pins = arduino_hal::pins!(dp);
	let mut led = pins.led.into_output();

	loop {
		atomic::compiler_fence(Ordering::SeqCst);
		led.toggle();
		delay_ms(500);
		led.toggle();
		delay_ms(500);
	}
}

fn now() -> u32 {
	interrupt::free(|cs| SECONDS.borrow(cs).get())
}

fn en_wdi_and_pd() {
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
fn check_for_messages_for_a_bit<E: Debug, R: Radio<E>, P: embedded_hal::digital::InputPin>(
	radio: &mut R,
	irq: &mut P,
) -> Option<Message> {
	radio.to_rx().unwrap();
	let mut i = 0u8;

	// >= 150 ms wait
	while i < u8::MAX {
		if let Ok(message) = radio
			.receive(irq, None)
			.map(|payload| postcard::from_bytes::<Message>(payload.data()).unwrap())
		{
			// radio.to_idle().unwrap();
			return Some(message);
		}
		i += 1;
		delay_us(500);
	}
	// radio.to_idle().unwrap();
	None
}

const DELAY: u16 = 50;

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	// If we don't do this the watchdog will reset the device after
	// 16ms allowing only the next few lines of code to execute :(
	// That's why we were seeing a neverending blinking light
	// This took me 2 days to figure out
	acknowlege_and_disable_watchdog();

	// Led
	let mut led = pins.led.into_output();
	led.toggle();
	delay_ms(DELAY);
	led.toggle();

	let mut node_addr = 0x9797u16;
	let node_info: NodeInfo = NodeInfo {
		board: Board::SamnV9,
		heartbeat_interval: HEARTBEAT_INTERVAL,
	};

	// Serial
	// let serial = arduino_hal::default_serial!(dp, pins, 57600);
	// // this gives ownership of the serial port to panic-serial. We receive a mutable reference to it though, so we can keep using it.
	// let mut serial = share_serial_port_with_panic(serial);

	// panic!("B watchdog");

	// Watchdog timer
	let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
	// Start watchdog :) , operations shouldn't take longer than 2sec.
	watchdog.start(WATCHDOG_TIMEOUT).unwrap();

	// panic!("Nooo");

	// Radio
	let mut radio;
	let mut irq = pins.g0_irq;
	{
		let (spi, _) = arduino_hal::Spi::new(
			dp.SPI0,
			pins.b5.into_output(),
			pins.b3.into_output(),
			pins.b4.into_pull_up_input(),
			pins.b2.into_output(),
			spi::Settings {
				mode: Mode {
					polarity: embedded_hal::spi::Polarity::IdleLow,
					phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
				},
				clock: spi::SerialClockRate::OscfOver2,
				..Default::default()
			},
		);
		let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());
		radio = NRF24L01::new(pins.g2_ce.into_output(), spi).unwrap();
		radio.configure().unwrap();
		radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();

		// // Setting up interrupt for nrf24
		// // Configure INT1 for falling edge. 0x03 would be rising edge.
		// dp.EXINT.eicra.modify(|_, w| w.isc1().bits(0x02));
		// // Enable the INT1 interrupt source.
		// dp.EXINT.eimsk.modify(|_, w| w.int1().set_bit());
	}

	let mut last_message: u32 = 0;
	{
		// Look for a valid Node Address
		let node_id;
		{
			// Get id from EEPROM
			let eeprom = arduino_hal::Eeprom::new(dp.EEPROM);
			let mut bytes = [0u8; 4];
			eeprom.read(0, &mut bytes).unwrap();
			node_id = u32::from_le_bytes(bytes);
		}
		loop {
			let now = now();
			if now >= last_message + SEARCH_NETWORK_INTERVAL {
				// Blink twice
				led.toggle();
				delay_ms(DELAY);
				led.toggle();

				delay_ms(DELAY);

				led.toggle();
				delay_ms(DELAY);
				led.toggle();

				delay_ms(DELAY);

				// Send looking for network
				radio
					.transmit(&Payload::new_with_addr(
						&postcard::to_vec::<Message, 32>(&Message::SearchingNetwork(node_id)).unwrap(),
						node_addr,
						addr_to_nrf24_hq_pipe(node_addr),
					))
					.unwrap();
				last_message = now;

				if let Some(Message::Network(node_id_in, node_addr_in)) = check_for_messages_for_a_bit(&mut radio, &mut irq) {
					if node_id_in == node_id {
						node_addr = node_addr_in;
						radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();
						led.toggle();
						delay_ms(DELAY);
						led.toggle();
						break; // We got a valid address, get out of this loop
					}
				}
			}

			radio.to_idle().unwrap();
			en_wdi_and_pd();
		}
	}

	let mut limbs: Limbs = [
		Some(Limb(
			0,
			LimbType::Sensor {
				report_interval: 60 * 5,
				data: None,
			},
		)),
		Some(Limb(1, LimbType::Actuator(Actuator::Light(false)))),
		Some(Limb(
			2,
			LimbType::Sensor {
				report_interval: 60 * 5,
				data: None,
			},
		)),
	];

	// Battery
	let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
	let bat_pin = pins.battery.into_analog_input(&mut adc);
	// turning adc reading into correct AA battery percentage
	let mut battery_percentage = || -> u8 {
		((max(min(bat_pin.analog_read(&mut adc), 500), 300) - 300) / 2)
			.try_into()
			.unwrap()
	};

	// Bme280
	let mut bme280;
	{
		let i2c = arduino_hal::i2c::I2c0::new(
			dp.TWI0,
			pins.c4.into_pull_up_input(),
			pins.c5.into_pull_up_input(),
			50_000,
		);
		const SETTINGS: bme280::Settings = bme280::Settings {
			config: bme280::Config::RESET
				.set_standby_time(bme280::Standby::Millis125)
				.set_filter(bme280::Filter::X8),
			ctrl_meas: bme280::CtrlMeas::RESET
				.set_osrs_t(bme280::Oversampling::X8)
				.set_osrs_p(bme280::Oversampling::X8)
				.set_mode(bme280::Mode::Normal),
			ctrl_hum: bme280::Oversampling::X8,
		};
		bme280 = bme280::Bme280::from_i2c0(i2c, bme280::Address::SdoGnd).unwrap();
		bme280.reset().ok();
		delay_ms(20);
		bme280.settings(&SETTINGS).ok();
		delay_ms(1000);
	}
	let mut temperature_humidity = || -> (i16, u8) {
		let measurement = bme280.sample().unwrap();
		(
			measurement.temperature.try_into().unwrap(),
			measurement.humidity.try_into().unwrap(),
		)
	};

	// Sensor Id -> last_report_time
	let mut sensor_last_report = [0u32; LIMBS_MAX];
	let mut update_sensors = |now: &u32, limbs: &mut Limbs| -> bool {
		let mut updated = false;
		// Here we check if sensors need to report anything
		// Depending on the report interval of each sensor
		for limb in limbs.iter_mut() {
			if let Some(Limb(limb_id, LimbType::Sensor { report_interval, data })) = limb {
				// Check if it's our time to report, using report_interval
				if *now >= sensor_last_report[*limb_id as usize] + (*report_interval as u32) {
					match *limb_id {
						0 => *data = Some(Sensor::Battery(battery_percentage())),
						2 => *data = Some(Sensor::TempHum(temperature_humidity())),
						_ => {}
					}
					sensor_last_report[*limb_id as usize] = *now;
					updated = true;
				}
			}
		}
		updated
	};

	// let mut update_actuators = |limbs:&Limbs, led: &mut Pin<Output, PD4>| {
	// };

	loop {
		// Sensor / heartbeat / commands / acuators loop
		{
			let now = now();
			let sensor_updated = update_sensors(&now, &mut limbs);

			// Only send if there's any sensor to report
			// Or we have a hearbeat due
			if sensor_updated || now >= last_message + node_info.heartbeat_interval as u32 {
				let message = Message::Message(MessageData::Response {
					id: None,
					response: if sensor_updated {
						Response::Limbs(limbs.clone())
					} else {
						Response::Heartbeat(now)
					},
				});
				let mut data = [0u8; 32];
				let data = postcard::to_slice(&message, &mut data).unwrap();

				// Indicate we are sending
				led.toggle();
				radio
					.transmit(&Payload::new_with_addr(
						&data,
						node_addr,
						addr_to_nrf24_hq_pipe(node_addr),
					))
					.unwrap();
				led.toggle();
				last_message = now;

				// Listen for commands for >=76ms, reset counter if command received
				while let Some(Message::Message(MessageData::Command { id, command })) =
					check_for_messages_for_a_bit(&mut radio, &mut irq)
				{
					// Answer command
					let message = Message::Message(MessageData::Response {
						id: Some(id),
						response: match command {
							Command::Info => Response::Info(node_info.clone()),
							Command::Limbs => Response::Limbs(limbs.clone()),
							Command::SetLimb(limb_in) => {
								if let Some(limb) = limbs
									.iter_mut()
									.filter_map(|l| if let Some(l) = l { Some(l) } else { None })
									.find(|limb| limb.0 == limb_in.0)
								{
									if variant_eq(&limb.1, &limb_in.1) {
										match (&mut limb.1, limb_in.1) {
											(LimbType::Actuator(actuator), LimbType::Actuator(actuator_in)) => {
												if variant_eq(actuator, &actuator_in) {
													*actuator = actuator_in;
													// Updating actuators
													for limb in limbs.iter() {
														if let Some(Limb(limb_id, LimbType::Actuator(actuator))) = limb {
															match (*limb_id, actuator) {
																(1, Actuator::Light(value)) => {
																	if *value {
																		led.set_high();
																	} else {
																		led.set_low();
																	}
																}
																_ => {}
															}
														}
													}

													// Send limbs again
													Response::Limbs(limbs.clone())
												// Response::Ok
												} else {
													Response::ErrLimbTypeDoesntMatch
												}
											}

											(
												LimbType::Sensor {
													report_interval,
													data: _,
												},
												LimbType::Sensor {
													report_interval: report_interval_in,
													data: _,
												},
											) => {
												*report_interval = report_interval_in;

												// Send limbs again
												Response::Limbs(limbs.clone())
												// Response::Ok
											}
											_ => Response::ErrLimbTypeDoesntMatch,
										}
									} else {
										Response::ErrLimbTypeDoesntMatch
									}
								} else {
									Response::ErrLimbNotFound
								}
							}
						},
					});

					let mut data = [0u8; 32];
					let data = postcard::to_slice(&message, &mut data).unwrap();

					led.toggle();
					radio
						.transmit(&Payload::new_with_addr(
							&data,
							node_addr,
							addr_to_nrf24_hq_pipe(node_addr),
						))
						.unwrap(); // Send sets tx, sends
					led.toggle();
				}
			}
		}

		radio.to_idle().unwrap();
		en_wdi_and_pd();
	}
}
