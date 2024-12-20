#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cmp::{max, min};

use arduino_hal::Delay;
use arduino_hal::{delay_ms, hal::wdt, spi};
use helper::*;
use samn9::mypanic::maybe_init_serial_panic;
use samn_common::cc1101::Cc1101;

use samn_common::node::Actuator;
use samn_common::{
	node::{Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response, Sensor, LIMBS_MAX},
	radio::*,
};

use samn9::*;

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
	delay_ms(LED_ON_MS);
	led.toggle();

	let mut node_addr = 0x9797u16;
	let node_info: NodeInfo = NodeInfo {
		board: Board::SamnV9,
		heartbeat_interval: HEARTBEAT_INTERVAL,
	};

	maybe_init_serial_panic();

	// Watchdog timer
	let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
	watchdog.start(WATCHDOG_TIMEOUT).unwrap();

	// Radio
	let mut radio;
	let mut g2 = pins.g2_ce;
	{
		let (spi, _) = arduino_hal::Spi::new(
			dp.SPI0,
			pins.b5.into_output(),
			pins.b3.into_output(),
			pins.b4.into_pull_up_input(),
			pins.b2.into_output(),
			spi::Settings {
				mode: embedded_hal::spi::MODE_0,
				clock: spi::SerialClockRate::OscfOver2,
				data_order: spi::DataOrder::MostSignificantFirst,
			},
		);
		let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spi, pins.csn.into_output(), Delay::new());
		radio = Cc1101::new(spi).unwrap();
		radio.init(&mut Delay::new()).unwrap();
		radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();
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
				delay_ms(LED_ON_MS);
				led.toggle();

				delay_ms(LED_OFF_MS);

				led.toggle();
				delay_ms(LED_ON_MS);
				led.toggle();

				delay_ms(LED_OFF_MS);

				send_looking_for_network(&mut radio, node_id, node_addr);
				last_message = now;
				delay_ms(3);

				if let Some(Message::Network(node_id_in, node_addr_in)) =
					check_for_messages_for_a_bit(&mut radio, &mut g2, &mut Delay::new())
				{
					if node_id_in == node_id {
						node_addr = node_addr_in;
						radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();
						led.toggle();
						delay_ms(LED_OFF_MS);
						led.toggle();
						break; // We got a valid address, get out of this loop
					}
				}
			}

			radio.to_idle().unwrap();
			en_wdi_and_pd();
		}
	}

	// Battery
	let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
	let bat_pin = pins.battery.into_analog_input(&mut adc);
	// turning adc reading into correct AA battery percentage
	let mut battery_percentage = || -> u8 { ((max(min(bat_pin.analog_read(&mut adc), 500), 300) - 300) / 2) as u8 };

	let mut limbs: Limbs = [
		Some(Limb(
			0,
			LimbType::Sensor {
				report_interval: 60 * 5,
				data: Some(Sensor::Battery(battery_percentage())),
			},
		)),
		Some(Limb(1, LimbType::Actuator(Actuator::Light(false)))),
		None,
	];
	let mut light_pin = pins.b0.into_output();
	light_pin.set_low();

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
						_ => {}
					}
					sensor_last_report[*limb_id as usize] = *now;
					updated = true;
				}
			}
		}
		updated
	};

	loop {
		// Sensor / heartbeat / commands / acuators loop

		let now = now();
		let sensor_updated = update_sensors(&now, &mut limbs);

		// Only send if there's any sensor to report
		// Or we have a hearbeat due
		if sensor_updated || now >= last_message + node_info.heartbeat_interval as u32 {
			// Wake up the radio
			radio.wake_up_wait().unwrap();
			// Rewrite the patable because its lost during sleep
			radio.write_patable().unwrap();

			let message = Message::Message(MessageData::Response {
				id: None,
				response: if sensor_updated {
					Response::Limbs(limbs.clone())
				} else {
					Response::Heartbeat(now)
				},
			});

			// Indicate we are sending
			led.toggle();
			send_message(
				&mut radio,
				message,
				node_addr,
			);
			led.toggle();
			last_message = now;

			// Listen for commands for >=76ms, reset counter if command received
			while let Some(Message::Message(MessageData::Command { id, command })) =
				check_for_messages_for_a_bit(&mut radio, &mut g2, &mut Delay::new())
			{
				// Answer command
				let response = match command {
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
																light_pin.set_high();
															} else {
																light_pin.set_low();
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
					Command::ToggleLimb(limb_id) => {
						if let Some(limb) = limbs
							.iter_mut()
							.filter_map(|l| if let Some(l) = l { Some(l) } else { None })
							.find(|limb| limb.0 == limb_id)
						{
							if let Limb(_, LimbType::Actuator(Actuator::Light(value))) = limb {
								*value = !*value;
								if *value {
									light_pin.set_high();
								} else {
									light_pin.set_low();
								}
								Response::Limbs(limbs.clone())
							} else {
								Response::ErrLimbTypeDoesntMatch
							}
						} else {
							Response::ErrLimbNotFound
						}
					}
				};

				// Answer command
				// Toggle leds while doing it
				led.toggle();
				send_message(
					&mut radio,
					Message::Message(MessageData::Response { id: Some(id), response }),
					node_addr,
				);
				led.toggle();
			}

			radio.to_idle().unwrap();
			radio.power_down().unwrap();
		}

		en_wdi_and_pd();
	}
}
