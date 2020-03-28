

use gpio_cdev::{Line,LineHandle,LineRequestFlags,EventRequestFlags,LineEventHandle};
use spidev::{Spidev,SpidevTransfer,SpidevOptions,SpiModeFlags};
use crate::{bit,cond_set_bit,set_bit,Error};
use num_traits::FromPrimitive;
use num_derive::FromPrimitive;
use std::convert::TryInto;
use std::os::unix::io::AsRawFd;
use std::thread::sleep;
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
	mode: Mode,
	dio_maps: [u8; 6],
	fifo_thresh: u8,
	bitrate: u32,
}
struct IrqWait<'a> {
	line: Option<LineEventHandle>,
	irq: Irq,
	rf: &'a Rfm69,
	high: bool
}
impl IrqWait<'_> {
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
			let check = timeout / 10;
			let start = Instant::now();
			let reg = Register::from_u8(Register::IrqFlags1 as u8 + (self.irq as u8) / 8).unwrap();
			let b = self.irq as u8 % 8;
			while Instant::now().duration_since(start) < timeout {
				let flags = self.rf.read(reg)?;
				if bit(flags, b) == self.high {
					return Ok(())
				}
				sleep(check);	
			}
			#[cfg(test)]{
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
			.max_speed_hz(1)
			.mode(SpiModeFlags::SPI_MODE_0)
			.build();

		spi.configure(&options)?;
		let dios = [None, None, None, None, None, None];
		let rfm = Rfm69 { spi, rst, en, dios, verbose: false, bitrate: 4800,
			pc: PacketConfig::default(), mode: Mode::Standby, dio_maps: [0; 6] , fifo_thresh: 15 };
		rfm.validate_dev()?;
		Ok(rfm)
	}
	pub fn get_version(&self) -> Result<u8,std::io::Error> {
		self.read(Register::Version)
	}
	pub fn reset(self) -> Result<Self, Error> {
		self.rst.set_value(1)?;
		sleep(Duration::from_millis(1));
		self.rst.set_value(0)?;
		sleep(Duration::from_millis(5));
		self.validate_dev()?;
		Ok(self)
	}
	fn validate_dev(&self) -> Result<(), Error> {
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
		Ok(SyncConfig::from(&buf))
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
	pub fn set_config(&mut self, config: &PacketConfig) -> Result<(), Error> {
		// validate config
		config.validate()?;

		// set config
		let mut buf = [0; 4];
		buf[0] = 0x80 * config.variable as u8; // bit 7
		buf[0] |= (config.dc & 0x03) << 5; // bit 6-5
		buf[0] |= (config.crc as u8) << 4; // bit 4
		buf[0] |= (config.clear as u8) << 3; // bit 3
		buf[0] |= (config.filtering & 0x03) << 1; // bit 2-1
		buf[1] = config.length;
		buf[2] = config.address;
		buf[3] = config.broadcast;
		self.write_many(Register::PacketConfig1, &buf)?;

		let mut buf = [0, 0];
		buf[0] = 0x80 * config.variable as u8; // bit 7
		buf[0] |= 0x7F & config.thresh; // bit 6-0
		buf[1] = (0x0F & config.delay) << 4; //bit 7-4
		buf[1] |= (config.restart as u8) << 1; // bit 1
		buf[1] |= config.aes as u8;  // bit 0
		self.write_many(Register::FifoThresh, &mut buf)?;
		self.pc = *config;
		Ok(())

	}
	fn config(&self) -> PacketConfig {
		self.pc
	}

	fn get_fifo_not_empty(&self) -> Result<IrqWait, Error> {
		unimplemented!();	
	}
	fn get_packet_sent(&self) -> Result<IrqWait, Error> {
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::PacketSent, high: true };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Tx && self.dio_maps[0] == 0 {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
			}
		}
		Ok(irq)

	}
	fn get_payload_ready(&self) -> Result<IrqWait,Error> {
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::PayloadReady, high: true };
		if let Some(line) = &self.dios[0] {
			if self.mode == Mode::Rx && ((self.pc.crc && self.dio_maps[0] == 0) || (self.dio_maps[0] == 1)) {
				let leh = line.events(LineRequestFlags::INPUT, EventRequestFlags::RISING_EDGE, "rfm69_g0")?;
				irq.line = Some(leh);
			}
		}
		if self.pc.crc { irq.irq = Irq::CrcOk; }
		Ok(irq)

	}

	fn get_fifo_full(&self) -> Result<IrqWait, Error> {
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::FifoFull, high: false };
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
		let mut irq = IrqWait { line: None, rf: self, irq: Irq::ModeReady, high: true };
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
		if buf.len() == self.pc.length as usize {
			self.fifo_write(buf)?;
			let fifo_wait = Duration::from_secs_f64(66.0 * 80.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10

			self.get_packet_sent()?.check_and_wait(fifo_wait)
		} else {
			Err(Error::BadInputs(format!("Buffer length ({}) must match fixed length ({})!", buf.len(), self.pc.length)))
		}
	}

	fn send_variable(&self, payload: &[u8]) -> Result<(), Error> {
		if payload.len() > 256 {
			return Err(Error::BadInputs(format!("Buffer length ({}) cannot be greater than 255!", payload.len())));
		} else if self.pc.aes {
			if self.pc.filtering != 0 {
				if payload.len() > 50 {
					return Err(Error::BadInputs("When AES is enabled with filterin, payload length must be less than 51".to_string()));
				}
			}
			else if payload.len() > 65 {
				return Err(Error::BadInputs("When AES is enabled, payload length must be less than 66".to_string()));
			}
			
		} else if self.pc.variable && payload.len() - 1 != payload[0] as usize {
			return Err(Error::BadInputs("When using the variable length format, the first byte must be the length of the buffer.".to_string()));
		}

		self.fifo_write(payload)
	}
	fn send_unlimited(&self, payload: &[u8]) -> Result<(), Error> {
		unimplemented!()	
	}
	pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
		self.set_mode_internal(Mode::Tx)?;
		if self.pc.variable { // variable packet
			self.send_variable(payload)
		} else if self.pc.is_unlimited() {
			self.send_unlimited(payload)
		} else {  // fixed length
			self.send_fixed(payload)
		}
	}
	fn fifo_read(&self, count: usize, timeout: Duration) -> Result<Vec<u8>, Error> {
		let mut ret = Vec::with_capacity(count);
		let mut recvd = 0;
		let fifo_wait = Duration::from_secs_f64(80.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10
		let fifo = self.get_fifo_not_empty()?;
		fifo.check_and_wait(timeout)?;
		while recvd + 1 < count {
			fifo.check_and_wait(fifo_wait)?;
			ret.push(self.read(Register::Fifo)?);
			recvd += 1;
		}
		let ready = self.get_payload_ready()?;
		ready.check_and_wait(fifo_wait)?;
		ret.push(self.read(Register::Fifo)?);
		Ok(ret)	
	
	}
	fn fifo_read_no_check(&self, count: usize) -> Result<Vec<u8>, Error> {
		let mut ret = vec![0; count];
		self.read_many(Register::Fifo, &mut ret)?;
		Ok(ret)
	}
	fn recv_fixed(&self, timeout: Duration) -> Result<Vec<u8>, Error> {
		if self.pc.length <= 66 {
			eprintln!("Doing fixed_recv() short message");
			let irq = self.get_payload_ready()?;
			irq.check_and_wait(timeout)?;
			eprintln!("Doing fixed_recv() payload ready");
			self.fifo_read_no_check(self.pc.length as usize)
		} else {
			self.fifo_read(self.pc.length as usize, timeout)
		}
	}
	fn recv_variable(&self, timeout: Duration) -> Result<Vec<u8>, Error> {
		let irq = self.get_fifo_not_empty()?;
		irq.check_and_wait(timeout)?;
		let len = self.read(Register::Fifo)?;
		if len <= 66 {
			let irq = self.get_payload_ready()?;
			irq.check_and_wait(timeout)?;
			self.fifo_read_no_check(self.pc.length as usize)
		} else {
			self.fifo_read(len as usize, timeout)
		}

	}
	pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>,Error> {
		self.set_mode(Mode::Rx)?;
		if self.pc.variable {
			self.recv_variable(timeout)
		} else if self.pc.is_unlimited() {
			unimplemented!();
		} else {
			self.recv_fixed(timeout)

		}
	}
	fn set_mode_internal(&mut self, mode: Mode) -> Result<(), std::io::Error> {
		if self.mode == mode {
			Ok(())
		} else if self.mode == Mode::Listen {
			self.write(Register::OpMode, set_bit(mode as u8, 5))?;
			self.write(Register::OpMode, mode as u8)
		} else {
			self.write(Register::OpMode, mode as u8)	
		}
	}
	pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
		self.set_mode_internal(mode)?;
		// wait for mode to be set
		let iw = self.get_mode_ready()?;
		iw.check_and_wait(Duration::from_millis(10))
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
		eprintln!("{:?}", bytes);
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
		eprintln!("{:?}", ret);
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

#[derive(Clone,Copy)]
pub struct PacketConfig {
	pub variable: bool,
	pub length: u8,
	pub dc: u8,
	pub crc: bool,
	pub clear: bool,
	pub filtering: u8,
	pub address: u8,
	pub broadcast: u8,
	pub aes: bool,
	pub restart: bool,
	pub delay: u8,
	pub start: bool,
	pub thresh: u8
}
impl PacketConfig {
	pub fn is_fixed(&self) -> bool {
		(!self.variable) && (self.length !=0)
	}
	pub fn is_unlimited(&self) -> bool {
		(!self.variable) && (self.length == 0)
	}
	pub fn validate(&self) -> Result<(), Error> {
		if self.aes { // perform aes validations
			// variable length validation is done when sedning messages, not configuring
			if self.is_unlimited() {
				return Err(Error::BadInputs("AES and unlimited packet size are incompatiable.".to_string()));
			}
			if self.is_fixed() {
				if self.filtering != 1 {
					if  self.length >= 66 {
						return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering enabled, length must be less than 66".to_string()));
					}
				} else {
					if self.length >= 65 {
						return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering disabled, length must be less than 65".to_string()));

					}
				}
			}
		}
		Ok(())
	}
}
pub struct FifoConfig {

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
		PacketConfig {
			variable: false,
			dc: 0x00,
			crc: true,
			clear: false,
			length: 0x40,
			filtering: 0x00,
			address: 0x00,
			broadcast: 0x00,
			delay: 0x00,
			aes: false,
			restart: true,
			start: true,
			thresh: 0x0F
		}
	}
}

#[derive(PartialEq,Clone,Copy)]
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



	
