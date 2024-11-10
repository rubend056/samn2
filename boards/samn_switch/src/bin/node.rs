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
use arduino_hal::{delay_ms, hal::wdt, spi, Delay};
use avr_device::interrupt::{self, Mutex};
use embedded_hal::{digital::OutputPin, spi::Mode};
use samn_common::nrf24::NRF24L01;
use samn_switch::mypanic::maybe_init_serial_panic;

use samn_common::{
	node::{
		Actuator, Board, Command, Limb, LimbType, Limbs, Message, MessageData, NodeInfo, Response, Sensor, LIMBS_MAX,
	},
	radio::*,
};

use samn_switch::*;

// ***************** SWITCH TOGGLING
use core::{
	cell::RefCell,
	sync::atomic::{AtomicBool, Ordering},
};
static SWITCH_TOGGLED: AtomicBool = AtomicBool::new(false);
static SWITCH_STATE: AtomicBool = AtomicBool::new(false);

// Executes on IRQ / Button Press changes
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
	// delay_ms(10);

	let dp = unsafe { avr_device::atmega328pb::Peripherals::steal() };

	// Check if button or irq is active.
	let pin = dp.PORTD.pind.read();
	// let _irq_active = port.pd2().bit_is_clear();
	// We don't do anything with IRQ because that'll get checked later on the main loop
	// We just have to make sure that our button is the one that triggered this interrupt
	// for us to set BUTTTON_PRESSED
	let button_active = pin.pd5().bit_is_clear();

	if button_active != SWITCH_STATE.load(Ordering::SeqCst) {
		SWITCH_TOGGLED.store(true, Ordering::SeqCst);
		SWITCH_STATE.store(button_active, Ordering::SeqCst);
	}
}
fn switch_toggled() -> bool {
	avr_device::interrupt::free(|_cs| {
		if SWITCH_TOGGLED.load(Ordering::SeqCst) {
			SWITCH_TOGGLED.store(false, Ordering::SeqCst);
			true
		} else {
			false
		}
	})
}

// ****************** ISENSE
// Shared state for ADC readings and control logic
static I_READINGS_AVERAGE: Mutex<RefCell<u16>> = Mutex::new(RefCell::new(0));
static I_READINGS: Mutex<RefCell<[u16; 16]>> = Mutex::new(RefCell::new([0; 16]));
static READING_INDEX: Mutex<RefCell<u8>> = Mutex::new(RefCell::new(0));

// type A = ;
// Shared static for the ADC and pin
static ADC: Mutex<RefCell<Option<arduino_hal::adc::Adc>>> = Mutex::new(RefCell::new(None));
static ISENSE_PIN: Mutex<
	RefCell<Option<arduino_hal::port::Pin<arduino_hal::port::mode::Analog, arduino_hal::hal::port::PC1>>>,
> = Mutex::new(RefCell::new(None));

// Helper function to calculate the average of the last 16 ADC readings
fn average_adc_readings() -> u16 {
	interrupt::free(|cs| {
		let readings = I_READINGS_AVERAGE.borrow(cs).borrow();
		*readings
	})
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
	interrupt::free(|cs| {
		// Borrow the ADC and pin
		let mut adc_ref = ADC.borrow(cs).borrow_mut();
		let mut pin_ref = ISENSE_PIN.borrow(cs).borrow_mut();

		if let (Some(adc), Some(isense_pin)) = (adc_ref.as_mut(), pin_ref.as_mut()) {
			// Perform an ADC reading
			match adc.read_nonblocking(isense_pin) {
				Ok(adc_value) => {
					// Start the next reading
					adc.read_nonblocking(isense_pin).ok();

					// Access the ADC readings and current index
					let mut readings = I_READINGS.borrow(cs).borrow_mut();
					let mut index = READING_INDEX.borrow(cs).borrow_mut();

					// Store the reading in the circular buffer
					readings[*index as usize] = adc_value;
					*index = (*index + 1) % 16; // Move to the next index, wrapping around to 0 after 15
					if *index == 0 {
						// Store the average of the readings in the longer buffer
						let mut total_average = I_READINGS_AVERAGE.borrow(cs).borrow_mut();

						let sum: u16 = readings.iter().map(|&r| (r as i16 - 511).abs() as u16).sum();
						let average = (sum / 16) as u32 * 100;
						*total_average = ((*total_average as u32 * 61 + average) / 62) as u16;
					}
				}
				_ => {}
			}
		}
	});
}

// With a clock of 8MHz a prescaler of 64 and a count of 125
// This will run every `1 / (8_000_000 / 64 / 125)` = `1ms`
const PRESCALER: u32 = 64;
const TIMER_COUNTS: u32 = 125;
fn timer_init(tc0: arduino_hal::pac::TC0) {
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
}

fn temperature_humidity<E: core::fmt::Debug, I2C: embedded_hal::i2c::I2c<Error = E>>(i2c: &mut I2C) -> (i16, u8) {
	const ADDRESS: u8 = 0x70;
	const WAKEUP: [u8; 2] = [0x35, 0x17];
	i2c.write(ADDRESS, &WAKEUP).unwrap();
	delay_ms(5);
	const NORMAL_T_FIRST: [u8; 2] = [0x78, 0x66];
	i2c.write(ADDRESS, &NORMAL_T_FIRST).unwrap();

	// Wait until device is ready to read
	delay_ms(20);

	let mut bytes = [0u8; 6];
	i2c.read(ADDRESS, &mut bytes).unwrap();
	let temp = ((bytes[0] as u16) << 8) | (bytes[1] as u16);
	let hum = ((bytes[3] as u16) << 8) | (bytes[4] as u16);
	const SLEEP: [u8; 2] = [0xB0, 0x98];
	i2c.write(ADDRESS, &SLEEP).unwrap();

	((temp / 15 * 4) as i16 - 4500, (hum / 665) as u8)
}
// Spits out current in milliamps (max of 20_000 mA, or 20A)
fn average_current() -> u16 {
	let reading = average_adc_readings();
	// same as * 20_000 / 51_200
	if reading < 2_600
	// To prevent an overflow & conserve precision
	{
		reading * 25 / 64
	} else {
		reading / 64 * 25
	}
}

// ************** MAIN

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

	// TRIAC
	let mut triac = pins.triac.into_output();
	triac.set_low();

	// I2C Init (for temp_hum sensor)
	let mut i2c = arduino_hal::i2c::I2c0::new(
		dp.TWI0,
		pins.sda.into_pull_up_input(),
		pins.scl.into_pull_up_input(),
		50_000,
	);

	// Switch
	let mut _switch = pins.switch.into_pull_up_input(); // make sure switch has has a pull-up attached

	// Led
	let mut led = pins.led.into_output();
	led.toggle();
	delay_ms(LED_OFF_MS);
	led.toggle();

	let mut node_addr = 0x9797u16;
	let node_info: NodeInfo = NodeInfo {
		board: Board::SamnSwitch,
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
	// For both  switch / pd5 / PCINT21  AND  irq / pd2 / PCINT18
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

	// Isense / Vsense
	{
		let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
		let isense_pin = pins.isense.into_analog_input(&mut adc);
		let mut _vsense = pins.vsense.into_pull_up_input();
		// Move ADC and pin to their static locations
		interrupt::free(|cs| {
			*ADC.borrow(cs).borrow_mut() = Some(adc);
			*ISENSE_PIN.borrow(cs).borrow_mut() = Some(isense_pin);
		});
		timer_init(dp.TC0);
	}

	// Enable interrupts
	unsafe { avr_device::interrupt::enable() };

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
			// Allowing for switch toggling while searching for network
			if switch_toggled() {
				triac.toggle();
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
			en_wdi_and_idle();
		}
	}

	// ----- We now have an address and are part of a network, we now setup the limbs
	// ----- and loop continously to operate the node.

	let mut limbs: Limbs = [
		Some(Limb(0, LimbType::Actuator(Actuator::Light(triac.is_set_high())))),
		Some(Limb(
			1,
			LimbType::Sensor {
				report_interval: 60 * 5,
				data: None,
			},
		)),
		Some(Limb(
			2,
			LimbType::Sensor {
				report_interval: 60 * 5,
				data: None,
			},
		)),
	];

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
						1 => *data = Some(Sensor::TempHum(temperature_humidity(&mut i2c))),
						2 => *data = Some(Sensor::Current(average_current())),
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
		let toggled = switch_toggled();
		let sensor_updated = update_sensors(&now, &mut limbs);

		if toggled {
			// Toggle limb & update mosfet
			if let Some(Limb(_, LimbType::Actuator(Actuator::Light(v)))) = &mut limbs[0] {
				*v = !*v;
				if *v {
					triac.set_high();
				} else {
					triac.set_low();
				}
			}
		}

		// Only send if there's any sensor to report
		// Or we have a hearbeat due
		// Or an actuator value has changed
		if sensor_updated || toggled || now >= last_message + node_info.heartbeat_interval as u32 {
			// radio.power_up().unwrap();

			let message = Message::Message(MessageData::Response {
				id: None,
				response: if toggled || sensor_updated {
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
													(0, Actuator::Light(value)) => {
														if *value {
															triac.set_high();
														} else {
															triac.set_low();
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
			};

			let mut data_initial = [0u8; 32];
			let data;
			{
				let message: Message = Message::Message(MessageData::Response { id: Some(id), response });
				data = postcard::to_slice(&message, &mut data_initial).unwrap();
			};

			// Answer command
			let payload = Payload::new_with_addr(&data, node_addr, addr_to_nrf24_hq_pipe(node_addr));

			led.toggle();
			radio.transmit(&payload).unwrap(); // Send sets tx, sends
			led.toggle();
		}

		en_wdi_and_idle();
	}
}
