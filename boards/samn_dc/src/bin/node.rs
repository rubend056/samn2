#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

/**
 * So samn_dc nodes has 1 push button & 1 mosfet it can use, along with of course the nrf24 radio.
 *
 * Because this is the first node where we keep the radio ON constantly we don't really care about power consumption.
 * For this one we should permanently keep micrcontroller ON and just keep reading IRQ periodically, every 100ms,
 * that would be simplest.
 *
 * Actually I went with putting microcontroller in powerdown while interrupts wake it up. Then it checks if button
 * has been pressed or there's something on the radio and reacts accordingly depending on if its currently finding
 * a network or doing the main loop of sending limbs/heartbeat and reacting to commands or not.
 *
 * Only bad thing about this is if you press the button fast enough, nah.. it would eventually reach the end of the
 * main loop, because the inrerrupt is only a few instructions long, and enable watchdog interrupt which means the
 * watchdog would work fine... just overthingking on my part...
 *
 * Let's test it out :)
 *
 */
use core::cmp::{max, min};

use arduino_hal::{delay_ms, hal::wdt, spi, Delay};
use embedded_hal::{digital::OutputPin, spi::Mode};
use samn_common::nrf24::NRF24L01;
use samn_dc::mypanic::maybe_init_serial_panic;

use samn_common::{
	node::{
		Actuator, Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response, Sensor, LIMBS_MAX,
	},
	radio::*,
};

use samn_dc::*;

use core::sync::atomic::{AtomicBool, Ordering};
static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);

// Executes on IRQ / Button Press changes
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
	delay_ms(10);

	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };

	// Check if button or irq is active.
	let pin = dp.PORTD.pind.read();
	// let _irq_active = port.pd2().bit_is_clear();
	// We don't do anything with IRQ because that'll get checked later on the main loop
	// We just have to make sure that our button is the one that triggered this interrupt
	// for us to set BUTTTON_PRESSED
	let button_active = pin.pd5().bit_is_clear();

	if button_active {
		BUTTON_PRESSED.store(true, Ordering::SeqCst);
	}
}
fn pressed(flag: &AtomicBool) -> bool {
	avr_device::interrupt::free(|_cs| {
		if flag.load(Ordering::SeqCst) {
			flag.store(false, Ordering::SeqCst);
			true
		} else {
			false
		}
	})
}

#[arduino_hal::entry]
fn main() -> ! {
	let dp = arduino_hal::Peripherals::take().unwrap();
	let pins = arduino_hal::pins!(dp);
	// If we don't do this the watchdog will reset the device after
	// 16ms allowing only the next few lines of code to execute :(
	// That's why we were seeing a neverending blinking light
	// This took me 2 days to figure out
	// Only do this after Peripherals::take, or take will Panic
	acknowlege_and_disable_watchdog();

	// MOSFET
	let mut mosfet = pins.mosfet.into_output();
	mosfet.set_low();

	// Switch
	let mut _button = pins.button.into_pull_up_input(); // make sure switch has has a pull-up attached

	// Led
	let mut led = pins.led.into_output();
	led.toggle();
	delay_ms(LED_OFF_MS);
	led.toggle();

	let mut node_addr = 0x9797u16;
	let node_info: NodeInfo = NodeInfo {
		board: Board::SamnDC,
		heartbeat_interval: HEARTBEAT_INTERVAL,
	};

	// Serial
	maybe_init_serial_panic();

	// Watchdog timer
	let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
	// Start watchdog :) , operations shouldn't take longer than 2sec.
	watchdog.start(WATCHDOG_TIMEOUT).unwrap();

	// Enable PCINT2
	dp.EXINT.pcicr.write(|w| unsafe { w.bits(0b100) });
	// For both  button / pd5 / PCINT21  AND  irq / pd2 / PCINT18
	dp.EXINT.pcmsk2.write(|w| w.bits(0b100100));

	// Radio
	let mut radio;
	let mut irq = pins.irq.into_pull_up_input();
	{
		let (spi, _) = arduino_hal::Spi::new(
			dp.SPI0,
			pins.sck.into_output(),
			pins.mosi.into_output(),
			pins.miso.into_pull_up_input(),
			pins.ss.into_output(),
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
		radio = NRF24L01::new(pins.ce.into_output(), spi).unwrap();
		radio.configure().unwrap();
		radio.set_rx_filter(&[addr_to_rx_pipe(node_addr)]).unwrap();
	}

	// ----- Now that everything is initialized, we try joining the network
	// ----- by sending out a SearchingNetwork message with our node_id so HQ
	// ----- can respond with an address.

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

		if node_id == u32::MIN || node_id == u32::MAX {
			// The user hasn't set a node id for us in the EEPROM
			led.set_low();
			loop {
				led.toggle();
				delay_ms(200);
				led.toggle();

				delay_ms(200);

				led.toggle();
				delay_ms(200);
				led.toggle();

				delay_ms(800);
			}
		}

		loop {
			// Allowing for button pressing while searching for network
			if pressed(&BUTTON_PRESSED) {
				mosfet.toggle();
			}

			let now = now();
			if now >= last_message + SEARCH_NETWORK_INTERVAL {
				// Blink twice
				led.toggle();
				delay_ms(LED_OFF_MS);
				led.toggle();

				delay_ms(LED_OFF_MS);

				led.toggle();
				delay_ms(LED_OFF_MS);
				led.toggle();

				delay_ms(LED_OFF_MS);

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

	// ----- We now have an address and are part of a network, we now setup the limbs
	// ----- and loop continously to operate the node.

	let mut limbs: Limbs = [
		Some(Limb(1, LimbType::Actuator(Actuator::Light(mosfet.is_set_high())))),
		None,
		None,
	];

	loop {
		// Sensor / heartbeat / commands / acuators loop

		let now = now();
		let button_pressed = pressed(&BUTTON_PRESSED);

		if button_pressed {
			// Toggle limb & update mosfet
			if let Some(Limb(_, LimbType::Actuator(Actuator::Light(v)))) = &mut limbs[0] {
				*v = !*v;
				if *v {
					mosfet.set_high();
				} else {
					mosfet.set_low();
				}
			}
		}

		// Only send if there's any sensor to report
		// Or we have a hearbeat due
		// Or an actuator value has changed
		if button_pressed || now >= last_message + node_info.heartbeat_interval as u32 {
			// radio.power_up().unwrap();

			let message = Message::Message(MessageData::Response {
				id: None,
				response: if button_pressed {
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
		}

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
																mosfet.set_high();
															} else {
																mosfet.set_low();
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

		en_wdi_and_pd();
	}
}
