

use gpio_cdev::{Line,LineHandle,LineRequestFlags,EventRequestFlags,LineEventHandle};
use spidev::{Spidev,SpidevTransfer,SpidevOptions,SpiModeFlags};
use crate::{bit,cond_set_bit,set_bit,Error,set_bit_to,sleep};
use num_traits::FromPrimitive;
use num_derive::FromPrimitive;
use std::convert::TryInto;
use std::os::unix::io::AsRawFd;
use std::fmt;
use std::time::{Instant,Duration};
use std::convert::TryFrom;
use nix::poll;

pub struct Rfm69 {
	rst: LineHandle,
	en: LineHandle,
	dios: [Option<Line>; 6],
	spi: Spidev,
	verbose: bool,
	pc: PacketConfig,
	sc: SyncConfig,
	mode: Mode,
	dio_maps: [u8; 6],
	fifo_thresh: u8,
	bitrate: u32,
	preamble: u16
}
struct IrqWait<'a> {
	line: Option<LineEventHandle>,
	irq: Irq,
	rf: &'a Rfm69,
	high: bool, 
	check: Duration // holds the duration in which the interrupt should be polled
}
impl IrqWait<'_> {
	fn check_flag(rfm: &Rfm69, irq: Irq) -> Result<bool, std::io::Error> {
		let reg = Register::from_u8(Register::IrqFlags1 as u8 + (irq as u8) / 8).unwrap();
		let b = irq as u8 % 8;	
		let flags = rfm.read(reg)?;
		Ok(bit(flags, b))
	}
	fn wait(&self, timeout: Duration) -> Result<(), Error> {
		if let Some(leh) = &self.line {
			let fd = leh.as_raw_fd();
			let pf = poll::PollFd::new(fd, poll::PollFlags::POLLIN);	
			let mut c_timeout = timeout.as_millis();
			if timeout.as_nanos() % 1_000_000 > 0 { c_timeout += 1 } // round up time
			match poll::poll(&mut [pf], c_timeout.try_into().unwrap()) {
				Ok(i) => if i > 0 { // an interrupt event occurred
					Ok(())
				} else { // no interrupt occured
					Err(Error::Timeout("Interrupt poll timeout!".to_string())) 
				},
				Err(e) => Err(Error::Timeout(format!("Interrupt poll error ({:?})!", e))) // a poll error
			}
		} else {
			let start = Instant::now();
			let reg = Register::from_u8(Register::IrqFlags1 as u8 + (self.irq as u8) / 8).unwrap();
			let b = self.irq as u8 % 8;
			let target = Instant::now() + timeout - self.check; // 
			loop { // 
				let flags = self.rf.read(reg)?;
				if bit(flags, b) == self.high {
					return Ok(())
				}
				if Instant::now() > target { break; }
				sleep(self.check);	
			}
			#[cfg(test)]
			{
				let reg = self.rf.read_all()?;
				for i in reg.iter().enumerate() {
					let reg = Register::from_usize(i.0 + 1).unwrap();
					eprintln!("{:>15?}: {:#04x}", reg, i.1);
				}
			}
			Err(Error::Timeout(format!("Interrupt ({:?}) timeout!", self.irq)))
		}
	}
	fn check(&self) -> Result<bool,Error> {
		if let Some(leh) = &self.line {
			Ok((leh.get_value()? != 0) == self.high)
		} else {
			let reg = Register::from_u8(Register::IrqFlags1 as u8 + (self.irq as u8) / 8).unwrap();
			let flags = self.rf.read(reg)?;
			Ok(bit(flags, self.irq as u8 % 8) == self.high)
		}
	}

	fn check_and_wait(&self, timeout: Duration) -> Result<(), Error> {
		if !self.check()? {
			self.wait(timeout)?;
		}
		Ok(())
	}
	fn is_true_irq(&self) -> bool { self.line.is_some() }
}

const FXOSC: u32 = 32_000_000;
const FSTEP: f64 = (FXOSC as f64) / 524_288.0; // FOSC/2^19
const MIN_FREQ: u32 = 290_000_000;
const MAX_FREQ: u32 = 1020_000_000;

impl Rfm69 {
	pub fn new(rst: Line, en: Line, mut spi: Spidev) -> Result<Self, Error> {
		let flags = LineRequestFlags::OUTPUT;
		let rst = rst.request(flags, 1, "rfm69_reset")?;
		sleep(Duration::from_millis(1)); // let settle
		rst.set_value(0)?;
		sleep(Duration::from_millis(5)); // let settle 

		let en = en.request(flags, 1, "rfm69_enable")?;

		let options = SpidevOptions::new()
			.bits_per_word(8)
			.max_speed_hz(5_000_000)
			.mode(SpiModeFlags::SPI_MODE_0)
			.build();

		spi.configure(&options)?;

		let dios = [None, None, None, None, None, None];
		let mut rfm = Rfm69 { spi, rst, en, dios, verbose: false, bitrate: 4800, preamble: 0x03,
			pc: PacketConfig::default(), mode: Mode::Standby, dio_maps: [0; 6] , fifo_thresh: 15,
			sc: SyncConfig::default()};
		rfm.validate_version()?;
		rfm.configure_defaults()?;
		Ok(rfm)
	}
	pub fn set_preamble_len(&mut self, len: u16) -> Result<(), Error> {
		unimplemented!();
	}
	pub fn preamble_len(&self) -> u16 {
		self.preamble
	}
	pub fn preamble_len_dev(&self) -> Result<u16, Error> {
		unimplemented!();
	}
	pub fn get_version(&self) -> Result<u8,std::io::Error> {
		self.read(Register::Version)
	}
	pub fn configure_defaults(&mut self) -> Result<(), Error> {
		self.set_config(&PacketConfig::default())?;
		self.set_sync(&SyncConfig::default())
	}
	pub fn reset(mut self) -> Result<Self, Error> {
		self.rst.set_value(1)?;
		sleep(Duration::from_millis(1));
		self.rst.set_value(0)?;
		sleep(Duration::from_millis(5));
		self.mode = Mode::Standby;
		self.bitrate = 4800;
		self.fifo_thresh = 15;
		self.validate_version()?;
		self.configure_defaults()?;
		Ok(self)
	}
	fn validate_version(&self) -> Result<(), Error> {
		let version = self.get_version()?;
		if version == 0x24 {
			Ok(())
		} else {
			Err(Error::InitError(format!("Version mismatch! expected 0x24, found {:#X}", version)))
		}
	}
	fn read(&self, reg: Register) -> Result<u8, std::io::Error> {
		let mut buf = [0];
		self.read_many(reg, &mut buf)?;
		Ok(buf[0])
	}
	fn read_many(&self, reg: Register, buf: &mut [u8]) -> Result<(), std::io::Error> {
		let reg = [(reg as u8) & 0x7F];
		let addr_xfer =  SpidevTransfer::write(&reg);
		let read_xfer = SpidevTransfer::read(buf);
		let mut xfers = [addr_xfer, read_xfer];
		self.spi.transfer_multiple(&mut xfers)
		
	}
	fn write(&self, reg: Register, val: u8) -> Result<(), std::io::Error> {
		let buf = [val];
		self.write_many(reg, &buf)
	}
	fn write_many(&self, reg: Register, buf: &[u8]) -> Result<(), std::io::Error> {
		let reg = [(reg as u8) | 0x80];
		let addr_xfer = SpidevTransfer::write(&reg);
		let write_xfer = SpidevTransfer::write(buf);
		let mut xfers = [addr_xfer, write_xfer];
		self.spi.transfer_multiple(&mut xfers)
	}
	pub fn aes(&self, key: &[u8; 16]) -> Result<(), std::io::Error> {
		self.write_many(Register::AesKey1, key)
	}
	pub fn read_all(&self) -> Result<[u8; 0x4E],std::io::Error> {
		let mut ret = [0; 0x4E];	
		self.read_many(Register::OpMode, &mut ret)?;
		Ok(ret)
	}
	pub fn set_bitrate(&mut self, bitrate: u32) -> Result<(), Error> {
		if let Ok(v) = ((FXOSC as f64 / bitrate as f64).round() as u64).try_into() {
			let v: u16 = v;
			self.write_many(Register::BitrateMsb, &v.to_be_bytes())?;
			self.bitrate = bitrate;
			Ok(())
		} else {
			Err(Error::BadInputs("Bitrate out of bounds!".to_string()))
		}
	}
	pub fn bitrate_dev(&self) -> Result<u32, Error> {
		let mut v = [0, 0];	
		self.read_many(Register::BitrateMsb, &mut v)?;
		let v = u16::from_be_bytes(v);
		let v = (FXOSC as f64 / v as f64).round() as u32;
		Ok(v)
	}
	pub fn bitrate(&self) -> u32 {
		self.bitrate
	}
	pub fn set_frequency(&self, frequency: u32) -> Result<(), Error> {
		if frequency < MIN_FREQ {
			Err(Error::BadInputs(format!("Frequency must be at least {} Mhz! Depending on the specific radio module the actually minimum frequency may be higher.", MIN_FREQ / 1_000_000)))
		} else if frequency > MAX_FREQ {
			Err(Error::BadInputs(format!("Frequency must be less than or equal to {} Mhz! Depending on the specific radio module the actually maximum frequency may be lower.", MAX_FREQ / 1_000_000)))
		} else {
			let v =  ((frequency as f64) / FSTEP ).round() as u32;
			let v = v.to_be_bytes();
			self.write_many(Register::FrfMsb, &v[1..4])?;
			Ok(())
		}
		
	}
	pub fn frequency(&self) -> Result<u32, Error> {
		let mut v = [0; 4];	
		self.read_many(Register::FrfMsb, &mut v[1..4])?;
		let v = u32::from_be_bytes(v);
		let ret = (v as f64 * FSTEP).round() as u32;
		Ok(ret)
	}
	pub fn set_sync(&self, config: &SyncConfig) -> Result<(), Error> {
		let v: [u8; 9] = config.try_into()?;
		self.write_many(Register::SyncConfig, &v)?;
		Ok(())
	}
	pub fn sync(&self) -> Result<SyncConfig, std::io::Error> {
		let mut buf = [0; 9];
		self.read_many(Register::SyncConfig, &mut buf)?;
		Ok((&buf).into())
	}
	pub fn temp(&self) -> Result<i8, Error> {
		self.write(Register::Temp1, 0x08)?;
		for _ in 0..5 {
			sleep(Duration::from_micros(20));
			let mut buf = [0, 0];
			self.read_many(Register::Temp1, &mut buf)?;
			if buf[0] & 0x04 == 0 {
				return Ok(buf[1] as i8);
			}
		}
		Err(Error::Timeout("Temperature reading timed out!".to_string()))
	}
	pub fn set_config<T: AsRef<PacketConfig>>(&mut self, config: T) -> Result<(), Error> {
		// validate config
		let config = config.as_ref();
		config.validate()?;

		self.write_many(Register::PacketConfig1, &config.0)?;
		self.write_many(Register::FifoThresh, &config.1)?;
		self.pc = *config;
		Ok(())

	}
	pub fn config(&self) -> PacketConfig {
		self.pc
	}
	pub fn config_dev(&self) -> Result<PacketConfig, Error> {
		let mut packet1 = [0; 4];
		self.read_many(Register::PacketConfig1, &mut packet1)?;
		
		let mut packet2 = [0, 0];
		self.read_many(Register::FifoThresh, &mut packet2)?;
		let pc = PacketConfig(packet1, packet2);	
		Ok(pc)
	}

	fn get_fifo_not_empty(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::FifoNotEmpty, high: true, check};
		if let Some(line) = &self.dios[0] {
			if self.dio_maps[1] == 2 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g1")?;
				irq.line = Some(leh);
				return Ok(irq);
			}
		}
		if let Some(line) = &self.dios[2] {
			if self.dio_maps[2] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g2")?;
				irq.line = Some(leh);
				return Ok(irq);
			}
		}
		Ok(irq)
	}
	fn get_fifo_overrun(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64); 
		let irq = IrqWait { line: None, rf: self, irq: Irq::FifoOverrun, high: true, check };
		Ok(irq)
	}
	fn get_packet_sent(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64); 
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::PacketSent, high: true, check };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Tx && self.dio_maps[0] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
			}
		}
		Ok(irq)

	}
	fn get_payload_crc(&self) -> Result<IrqWait,Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::PayloadReady, high: true, check };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Rx && ((self.pc.crc() && self.dio_maps[0] == 0) || (self.dio_maps[0] == 1)) {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
			}
		}
		if self.pc.crc() { irq.irq = Irq::CrcOk; }
		Ok(irq)

	}
	fn get_payload_ready(&self) -> Result<IrqWait,Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::PayloadReady, high: true, check };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Rx && self.dio_maps[0] == 1 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
			}
		}
		Ok(irq)
	}

	fn get_fifo_full(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::FifoFull, high: false, check };
		if let Some(line) = &self.dios[1] {
			if self.dio_maps[1] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::FALLING_EDGE, "rfm69_g1")?;
				irq.line = Some(leh);
				return Ok(irq);
			}
		} 
		if let Some(line) = &self.dios[3] {
			if self.dio_maps[3] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::FALLING_EDGE, "rfm69_g3")?;
				irq.line = Some(leh);
				return Ok(irq);
			}

		}
		Ok(irq)
	}
	fn get_mode_ready(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_millis(1);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::ModeReady, high: true, check };
		if let Some(line) = &self.dios[4] {
			if self.mode == Mode::Tx && self.dio_maps[4] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g4")?;
				irq.line = Some(leh);
				return Ok(irq);
			}

		}
		if let Some(line) = &self.dios[5] {
			if self.dio_maps[5] == 3 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g5")?;
				irq.line = Some(leh);
				return Ok(irq);

			}
		}
		Ok(irq)
	}
	fn get_sync_address(&self) -> Result<IrqWait, Error> {
		let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::SyncAddressMatch, high: true, check };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Rx && self.dio_maps[0] == 0x2 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
				return Ok(irq);
			}
		}
		if let Some(line) = &self.dios[3] {
			if self.mode == Mode::Rx && self.dio_maps[3] == 0x2 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
				return Ok(irq);
			}
		}
		Ok(irq)

	}
	fn fifo_write(&self, buf: &[u8]) -> Result<(), Error> {
		// write inits bytes to fifo
		let mut sent = 0;
		let first = usize::min(buf.len(), 66);
		self.write_many(Register::Fifo, &buf[0..first])?;
		sent += first;
		// ensure mode is ready before writing more
		let ready = self.get_mode_ready()?;
		ready.check_and_wait(Duration::from_millis(10))?;
		drop(ready);

		// send remaining bytes if there are any
		let iw = self.get_fifo_full()?;
		let fifo_wait = Duration::from_secs_f64(80.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10
		while sent < buf.len() {
			iw.check_and_wait(fifo_wait)?; // we need to check if the fifo is full
			self.write(Register::Fifo, buf[sent])?;
			sent += 1;
		}
		Ok(())

	}
	fn send_fixed(&self, buf: &[u8]) -> Result<(), Error> {
		if buf.len() == self.pc.len() as usize {
			self.fifo_write(buf)
		} else {
			Err(Error::BadInputs(format!("Buffer length ({}) must match fixed length ({})!", buf.len(), self.pc.len())))
		}
	}

	fn send_variable(&self, payload: &[u8]) -> Result<(), Error> {
		if payload.len() > 256 {
			return Err(Error::BadInputs(format!("Buffer length ({}) cannot be greater than 255!", payload.len())));
		} else if self.pc.aes() {
			if self.pc.filtering() != 0 {
				if payload.len() > 50 {
					return Err(Error::BadInputs("When AES is enabled with filterin, payload length must be less than 51".to_string()));
				}
			}
			else if payload.len() > 65 {
				return Err(Error::BadInputs("When AES is enabled, payload length must be less than 66".to_string()));
			}
			
		} else if self.pc.is_variable() && payload.len() - 1 != payload[0] as usize {
			return Err(Error::BadInputs("When using the variable length format, the first byte must be the length of the buffer.".to_string()));
		}

		self.fifo_write(payload)
	}
	fn send_unlimited(&self, payload: &[u8]) -> Result<(), Error> {
		unimplemented!()	
	}
	pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
		self.set_mode_internal(Mode::Tx)?;
		if self.pc.is_variable() { // variable packet
			self.send_variable(payload)
		} else if self.pc.is_unlimited() {
			self.send_unlimited(payload)
		} else {  // fixed length
			self.send_fixed(payload)
		}?;
		let send_timeout = Duration::from_secs_f64((273.0 + self.preamble as f64) * 8.0 / self.bitrate as f64);
		self.get_packet_sent()?.check_and_wait(send_timeout)
	}
	fn fifo_read(&self, count: usize, timeout: Duration) -> Result<Vec<u8>, Error> {
		#[cfg(test)]
		eprintln!("fifo_read(): {}", count);
		let mut ret = Vec::with_capacity(count);
		// wait for the byte to be received
		let fifo = self.get_fifo_not_empty()?;
		fifo.check_and_wait(timeout)?;

		// loop bytes
		eprintln!("reception started");
		let fifo_wait = Duration::from_secs_f64(160.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10
		for i in 0..(count-1) {
			if let Err(e) = fifo.check_and_wait(fifo_wait) {
				if IrqWait::check_flag(&self, Irq::FifoOverrun)? {
					return Err(Error::Timeout("Fifo overran while reading! Reading was too slow.".to_string()));
				} else {
					return Err(e);
				}
			}
			ret.push(self.read(Register::Fifo)?);
		}
		// at this point either CrcOk or PayloadReady should fire
		let ready = self.get_payload_crc()?;
		if let Err(e) = ready.check_and_wait(fifo_wait) { 
			/* if the sync word and address didnt match we would have timed-out above so we dont need to check
			   sync word interrupt.
			 */ 
			let s = if self.pc.crc() { 
				// check if this is a Crc Error or something else
				"A CRC Error occurred!".to_string()
			} else { 
				let s = "A payload ready was never received, after starting message reception. With no CRC, this should never happen.".to_string();
				return Err(Error::ChipMalfunction(s));
			};
			let bad_fifo = if !self.pc.clear()  {
				//if clear is not set we should get the bad payload 
				ret.push(self.read(Register::Fifo)?);
				Some(ret)
			} else { None };
			Err(Error::BadMessage(s, bad_fifo))
		} else { 
			ret.push(self.read(Register::Fifo)?);
			Ok(ret)	
		}
	
	}
	fn fifo_read_no_check(&self, count: usize, timeout: Duration) -> Result<Vec<u8>, Error> {
		let mut ret = vec![0; count];
		let irq = self.get_payload_crc()?;
		if let Err(e) = irq.check_and_wait(timeout) {
			if self.sc.on || self.pc.filtering() != 0 {
				if !IrqWait::check_flag(&self, Irq::SyncAddressMatch)? {
					// the sync and address never seen, normal timeout
					return Err(e);

				}
			} else {
				// if the sync and address filtering are disabled then this is a normal timeout
				return Err(e);
			}
			// From here on, it can be assumed a sync word or address was received
			if !self.pc.crc() {
				/* If CRC error didnt happen then it is something else
				   I don't know if this is even possible. Should this be unreachable arm instead?
				   With out CRC checking anything would be accept, so this are would represent a chip 
				   malfunction.
			     */ 
				let s = "A payload ready was never received, after starting message reception. With no CRC, this should never happen.".to_string();
				return Err(Error::ChipMalfunction(s));
			}
			let s = "A CRC Error occured!".to_string();
			let bad_fifo = if !self.pc.clear() {
				// if clear is not set we should get bad payload for error message
				self.read_many(Register::Fifo, &mut ret[..])?;
				Some(ret)
			} else { None };
			Err(Error::BadMessage(s, bad_fifo))
		} else {
			self.read_many(Register::Fifo, &mut ret)?;
			Ok(ret)
		}
	}
	fn recv_fixed(&self, timeout: Duration) -> Result<Vec<u8>, Error> {
		let len = self.pc.len() as usize;
		if len <= 66 {
			// short messages can be read in one go
			eprintln!("Doing fixed_recv() short message");
			self.fifo_read_no_check(len, timeout)
		} else {
			// long messages are required to be read continously
			self.fifo_read(len as usize, timeout)
		}
	}
	fn recv_variable(&self, timeout: Duration) -> Result<Vec<u8>, Error> {
		let irq = self.get_fifo_not_empty()?;
		irq.check_and_wait(timeout)?;
		let len = self.read(Register::Fifo)? as usize;
		if len <= 66 {
			eprintln!("Doing variable_recv() short message {}", len);
			self.fifo_read_no_check(len, timeout)
		} else {
			self.fifo_read(len , timeout)
		}

	}
	pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>,Error> {
		self.set_mode(Mode::Rx)?;
		if self.pc.is_variable() {
			self.recv_variable(timeout)
		} else if self.pc.is_unlimited() {
			unimplemented!();
		} else {
			self.recv_fixed(timeout)
		}
	}
	fn set_mode_internal(&mut self, mode: Mode) -> Result<(), std::io::Error> {
		let ret = if self.mode == mode {
			Ok(())
		} else if self.mode == Mode::Listen {
			self.write(Register::OpMode, set_bit(mode as u8, 5))?;
			self.write(Register::OpMode, mode as u8)
		} else {
			self.write(Register::OpMode, mode as u8)	
		};
		self.mode = mode;
		ret
	}
	pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
		self.set_mode_internal(mode)?;
		// wait for mode to be set
		let iw = self.get_mode_ready()?;
		iw.check_and_wait(Duration::from_millis(10))
	}
	pub fn mode(&self) -> Mode {
		self.mode
	}
	pub fn mode_dev(&self) -> Result<Mode, std::io::Error> {
		let mode = self.read(Register::OpMode)? & 0x7F;
		Ok(Mode::from_u8(mode).unwrap())

	}
}
impl Drop for Rfm69 {
	fn drop(&mut self) {
		if let Err(e) = self.en.set_value(0) {
			if self.verbose {
				eprintln!("Failed to set enable pin to off on drop: {:?}", e);
			}
		}
		if let Err(e) = self.rst.set_value(1) {
			if self.verbose {
				eprintln!("Failed to reset on drop to off on drop: {:?}", e);
			}
		}
	}
}

#[derive(FromPrimitive,Clone,Copy,Debug)]
pub enum Register {
    Fifo = 0x00,
    OpMode = 0x01,
    DataModul = 0x02,
    BitrateMsb = 0x03,
    BitrateLsb = 0x04,
    FdevMsb = 0x05,
    FdevLsb = 0x06,
    FrfMsb = 0x07,
    FrfMid = 0x08,
    FrfLsb = 0x09,
    Osc1 = 0x0A,
    AfcCtrl = 0x0B,
    LowBat = 0x0C,
    Listen1 = 0x0D,
    Listen2 = 0x0E,
    Listen3 = 0x0F,
    Version = 0x10,
    PaLevel = 0x11,
    PaRamp = 0x12,
    Ocp = 0x13,
    AgcRef = 0x14,
    AgcThresh1 = 0x15,
    AgcThresh2 = 0x16,
    AgcThresh3 = 0x17,
    Lna = 0x18,
    RxBw = 0x19,
    AfcBw = 0x1A,
    OokPeak = 0x1B,
    OokAvg = 0x1C,
    OokFix = 0x1D,
    AfcFei = 0x1E,
    AfcMsb = 0x1F,
    AfcLsb = 0x20,
    FeiMsb = 0x21,
    FeiLsb = 0x22,
    RssiConfig = 0x23,
    RssiValue = 0x24,
    DioMapping1 = 0x25,
    DioMapping2 = 0x26,
    IrqFlags1 = 0x27,
    IrqFlags2 = 0x28,
    RssiThresh = 0x29,
    RxTimeout1 = 0x2A,
    RxTimeout2 = 0x2B,
    PreambleMsb = 0x2C,
    PreambleLsb = 0x2D,
    SyncConfig = 0x2E,
    SyncValue1 = 0x2F,
    SyncValue2 = 0x30,
    SyncValue3 = 0x31,
    SyncValue4 = 0x32,
    SyncValue5 = 0x33,
    SyncValue6 = 0x34,
    SyncValue7 = 0x35,
    SyncValue8 = 0x36,
    PacketConfig1 = 0x37,
    PayloadLength = 0x38,
    NodeAddrs = 0x39,
    BroadcastAddrs = 0x3A,
    AutoModes = 0x3B,
    FifoThresh = 0x3C,
    PacketConfig2 = 0x3D,
    AesKey1 = 0x3E,
    AesKey2 = 0x3F,
    AesKey3 = 0x40,
    AesKey4 = 0x41,
    AesKey5 = 0x42,
    AesKey6 = 0x43,
    AesKey7 = 0x44,
    AesKey8 = 0x45,
    AesKey9 = 0x46,
    AesKey10 = 0x47,
    AesKey11 = 0x48,
    AesKey12 = 0x49,
    AesKey13 = 0x4A,
    AesKey14 = 0x4B,
    AesKey15 = 0x4C,
    AesKey16 = 0x4D,
    Temp1 = 0x4E,
    Temp2 = 0x4F,
    TestLna = 0x58,
    TestPa1 = 0x5A,
    TestPa2 = 0x5C,
    TestDagc = 0x6F,
}

#[derive(Debug,PartialEq)]
pub struct SyncConfig {
	pub syncword: [u8; 8],
	pub len: u8,
	pub diff: u8,
	pub condition: bool,
	pub on: bool
}
impl From<&[u8; 9]> for SyncConfig {
	fn from(bytes: &[u8; 9]) -> Self {
		eprintln!("making sync config from {:?}", bytes);
		let mut ret = SyncConfig::default();
		ret.on = bit(bytes[0], 7);
		ret.condition = bit(bytes[0], 6);
		ret.len = ((bytes[0] >> 3) & 0x07) + 1;
		ret.diff = bytes[0] & 0x07;
		for (i, v) in bytes[1..9].iter().enumerate() {
			ret.syncword[i] = *v;	
		}
		ret
	}
}
impl TryFrom<&SyncConfig> for [u8; 9] {
	type Error = Error;
	fn try_from(sc: &SyncConfig) -> Result<Self, Self::Error> {
		let mut ret = [0; 9];
		ret[0] = cond_set_bit(ret[0], 7, sc.on);
		ret[0] = cond_set_bit(ret[0], 6, sc.condition);
		if sc.len > 8  || sc.len == 0 {
			return Err(Error::BadInputs("Length must be [1,8].".to_string()));
		}
		ret[0] |= (sc.len - 1) << 3;
		if sc.diff > 7 {
			return Err(Error::BadInputs("Diff must be less than 8.".to_string()));
		}
		ret[0] |= sc.diff;
		for (i, v) in sc.syncword.iter().enumerate() {
			ret[i + 1] = *v;
		}
		eprintln!("made bytes from syncconfig: {:?}", ret);
		Ok(ret)
	}
}


impl Default for SyncConfig {
	fn default() -> Self {
		SyncConfig { 
			syncword: [1; 8], 
			len:  4,
			diff: 0,
			condition: false,
			on: true
		}
	}
}

#[derive(Clone,Copy,PartialEq,Debug)]
pub struct PacketConfig([u8; 4],[u8; 2]);

impl PacketConfig {
	#[inline]
	pub fn is_variable(&self) -> bool {
		bit(self.0[0], 7)
	}
	#[inline]
	pub fn set_variable(&mut self, val: bool) -> &mut Self {
		self.0[0] = set_bit_to(self.0[0], 7, val);
		self
	}
	#[inline]
	pub fn is_fixed(&self) -> bool {
		!self.is_variable() && self.len() != 0
	}
	#[inline]
	pub fn is_unlimited(&self) -> bool {
		!self.is_variable() && self.len() == 0
	}
	#[inline]
	pub fn len(&self) -> u8 {
		self.0[1]
	}
	#[inline]
	pub fn set_len(&mut self, len: u8) -> &mut Self {
		self.0[1] = len;
		self
	}
	#[inline]
	pub fn aes(&self) -> bool {
		bit(self.1[1], 0)
	}
	#[inline]
	pub fn restart(&self) -> bool {
		bit(self.1[1], 1)	
	}
	#[inline]
	pub fn delay(&self) -> u8 {
		(self.1[1] >> 4) & 0x0F
	}
	#[inline]
	pub fn threshold(&self) -> u8 {
		(self.1[0] >> 1) & 0x7F
	}
	#[inline]
	pub fn set_threshold(&mut self, threshold: u8) -> &mut Self {
		assert!(threshold < 0x80);
		self.1[0] = (self.1[0] & 0xF0) | threshold;
		self
	}
	#[inline]
	pub fn start(&self) -> bool {
		bit(self.1[0], 7)
	}
	#[inline]
	pub fn filtering(&self) -> u8 {
		(self.0[0] >> 1) & 0x03
	}
	#[inline]
	pub fn clear(&self) -> bool {
		bit(self.0[0], 3)
	}
	#[inline]
	pub fn crc(&self) -> bool {
		bit(self.0[0], 4)
	}
	#[inline]
	pub fn dc(&self) -> u8 {
		(self.0[0] >> 5) & 0x03
	}
	pub fn validate(&self) -> Result<(), Error> {
		if self.aes() { // perform aes validations
			// variable length validation is done when sedning messages, not configuring
			if self.is_unlimited() {
				return Err(Error::BadInputs("AES and unlimited packet size are incompatiable.".to_string()));
			}
			if self.is_fixed() {
				if self.filtering() != 1 {
					if  self.len() >= 66 {
						return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering enabled, length must be less than 66".to_string()));
					}
				} else {
					if self.len() >= 65 {
						return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering disabled, length must be less than 65".to_string()));

					}
				}
			}
		}
		Ok(())
	}
}
impl AsRef<PacketConfig> for PacketConfig {
	fn as_ref(&self) -> &PacketConfig {
		self
	}
}
impl fmt::Display for PacketConfig {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "{{ variable: {}, length: {} }}", self.is_variable(), self.len())
	}

}

fn byte_to_binary(u: u8) {
	for i in 0..8 {
		let shift = 1 << i;
		print!("{}", (u & shift) / shift);
	}
	println!("\n76543210");
}

impl Default for PacketConfig {
	fn default() -> Self {
		PacketConfig([0x10, 0x40, 0x00, 0x00],[0x8F, 0x02])
	}
}

#[derive(FromPrimitive,PartialEq,Clone,Copy,Debug)]
pub enum Mode {
	Listen = 0x40,
	Sleep = 0x00,
	Standby = 0x04,
	FS = 0x08,
	Tx = 0x0C,
	Rx = 0x10
}

#[derive(PartialEq,Clone,Copy,Debug)]
pub enum Irq {
	ModeReady = 0x07,
	RxReady = 0x06,
	TxReady = 0x05,
	PllLock = 0x04,
	Rssi =  0x03,
	Timeout = 0x02,
	AutoMode = 0x01,
	SyncAddressMatch = 0x00,
	FifoFull = 0x0F,
	FifoNotEmpty = 0x0E,
	FifoLevel = 0x0D,
	FifoOverrun = 0x0C,
	PacketSent = 0x0B,
	PayloadReady = 0x0A,
	CrcOk = 0x09
}



	
