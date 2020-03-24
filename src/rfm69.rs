

use gpio_cdev::{Line,LineHandle,LineRequestFlags};
use spidev::{Spidev,SpidevTransfer,SpidevOptions,SpiModeFlags};
use crate::Error;
use std::convert::TryInto;
use std::thread::sleep;
use std::time::Duration;

pub struct Rfm69 {
	rst: LineHandle,
	en: LineHandle,
	g0: LineHandle,
	spi: Spidev,
	_g1: Option<LineHandle>,
	_g2: Option<LineHandle>,
	_g3: Option<LineHandle>,
	_g4: Option<LineHandle>,
	_g5: Option<LineHandle>,
	verbose: bool
}

const FXOSC: u32 = 32_000_000;
const FSTEP: f64 = (FXOSC as f64) / 524_288.0; // FOSC/2^19
const MIN_FREQ: u32 = 290_000_000;
const MAX_FREQ: u32 = 1020_000_000;

impl Rfm69 {
	pub fn new(rst: Line, en: Line, g0: Line, mut spi: Spidev) -> Result<Self, Error> {
		let flags = LineRequestFlags::OUTPUT;
		let rst = rst.request(flags, 1, "rfm69_reset")?;
		sleep(Duration::from_millis(1)); // let settle
		rst.set_value(0)?;
		sleep(Duration::from_millis(5)); // let settle 

		let en = en.request(flags, 1, "rfm69_enable")?;
		let flags = LineRequestFlags::INPUT;
		let g0 = g0.request(flags, 0, "rfm69_g0")?;


		let options = SpidevOptions::new()
			.bits_per_word(8)
			.max_speed_hz(1)
			.mode(SpiModeFlags::SPI_MODE_0)
			.build();

		spi.configure(&options)?;

		let rfm = Rfm69 { spi, rst, en , g0, _g1: None, _g2: None, _g3: None, _g4: None, _g5: None, verbose: false };
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
		let mut addr_xfer = SpidevTransfer::write(&reg);
		addr_xfer.cs_change = 1;
		let mut write_xfer = SpidevTransfer::write(buf);
		write_xfer.cs_change = 0;
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
	pub fn set_bitrate(&self, bitrate: u32) -> Result<(), Error> {
		if let Ok(v) = ((FXOSC as f64 / bitrate as f64).round() as u64).try_into() {
			let v: u16 = v;
			self.write_many(Register::BitrateMsb, &v.to_be_bytes())?;
			Ok(())
		} else {
			Err(Error::BadInputs("Bitrate out of bounds!".to_string()))
		}
	}
	pub fn bitrate(&self) -> Result<u32, Error> {
		let mut v = [0, 0];	
		self.read_many(Register::BitrateMsb, &mut v)?;
		let v = u16::from_be_bytes(v);
		let v = (FXOSC as f64 / v as f64).round() as u32;
		Ok(v)
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
		if config.len > 8 {
			Err(Error::BadInputs("The syncword length must be [0,8]".to_string()))
		} else if config.diff > 7 {
			Err(Error::BadInputs("The allowed bit diff must be length must be [0,7]".to_string()))
		} else {
			let mut v = [0; 9];
			v[0] = 0x80 * config.on as u8;
			v[0] |= 0x40 * config.condition as u8;
			v[0] |= (config.len - 1) << 3;
			v[0] |= config.diff;
			for (i, n) in config.syncword.iter().enumerate() {
				v[i + 1] = *n;	
			}
			self.write_many(Register::SyncConfig, &v)?;
			Ok(())
		}
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

pub struct SyncConfig {
	pub syncword: [u8; 8],
	pub len: u8,
	pub diff: u8,
	pub condition: bool,
	pub on: bool
}
impl From<&[u8; 9]> for SyncConfig {
	fn from(bytes: &[u8; 9]) -> Self {
		let mut ret = SyncConfig::default();
		ret.on = bytes[0] & 0x80 != 0;
		ret.condition = bytes[0] & 0x40 != 0;
		ret.len = (bytes[0] & 0x38) >> 3;
		ret.diff = bytes[0] & 0x07;
		for (i, v) in bytes[1..9].iter().enumerate() {
			ret.syncword[i] = *v;	
		}
		ret
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
