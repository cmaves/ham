

pub mod rfm69;

#[derive(Debug)]
pub enum Error {
	GpioError(gpio_cdev::errors::Error),
	InitError(String),
	IoError(std::io::Error),
	BadInputs(String),
	Timeout(String)
}
impl From<gpio_cdev::errors::Error> for Error {
	fn from(v: gpio_cdev::errors::Error) -> Self {
		Error::GpioError(v)
	}
}
impl From<std::io::Error> for Error {
	fn from(v: std::io::Error) -> Self {
		Error::IoError(v)
	}
}

#[inline]
fn bit(byte: u8, bit: u8) -> bool {
	byte & (1 << bit) != 0
}
#[inline]
fn set_bit(byte: u8, bit: u8) -> u8 {
	byte | (1 << bit) 
}
#[inline]
fn unset_bit(byte: u8, bit: u8) -> u8 {
	byte & 0xFE_u8.rotate_left(bit as u32)
}
#[inline] 
fn cond_set_bit(byte: u8, bit: u8, cond: bool) -> u8 {
	byte | ((1 << bit) * (cond as u8))
}

#[cfg(test)]
mod tests {
	use gpio_cdev::Chip;
	use crate::rfm69::{PacketConfig,Rfm69,SyncConfig};
	use spidev::Spidev;
	use std::thread::{spawn,sleep};
	use std::time::Duration;

	#[test]
	fn make_rfm69() {
		let mut chip = Chip::new("/dev/gpiochip0").unwrap();
		let rst = chip.get_line(24).unwrap();
		let en = chip.get_line(3).unwrap();
		let spidev = Spidev::open("/dev/spidev0.0").unwrap();
		let rfm69 = Rfm69::new(rst, en, spidev).unwrap();
		for reg in rfm69.read_all().unwrap().iter().enumerate() {
			println!("Reg {:#04X}: {:#04X}", reg.0 + 1, reg.1);
		}
	}
	#[test]
	fn test_bitrate() {
		let mut chip = Chip::new("/dev/gpiochip0").unwrap();
		let rst = chip.get_line(24).unwrap();
		let en = chip.get_line(3).unwrap();
		let spidev = Spidev::open("/dev/spidev0.0").unwrap();
		let mut rfm69 = Rfm69::new(rst, en, spidev).unwrap();
		rfm69.set_bitrate(10_000).unwrap();
		assert_eq!(rfm69.bitrate(), 10_000);
		assert_eq!(rfm69.bitrate_dev().unwrap(), 10_000);
		
	}
	#[test]
	fn sync_word() {
		let mut chip = Chip::new("/dev/gpiochip0").unwrap();
		let rst = chip.get_line(24).unwrap();
		let en = chip.get_line(3).unwrap();
		let spidev = Spidev::open("/dev/spidev0.0").unwrap();
		let mut rfm69 = Rfm69::new(rst, en, spidev).unwrap();
		let mut sc = SyncConfig::default();
		for i in 0..8 {
			sc.syncword[i] = i as u8 + 1;	
		}
		rfm69.set_sync(&sc).unwrap();
		let ret = rfm69.sync().unwrap();
		assert_eq!(sc, ret);
	}
	#[test]
	fn sent_recv_fixed() {
		let mut chip = Chip::new("/dev/gpiochip0").unwrap();
		let rst = chip.get_line(24).unwrap();
		let en = chip.get_line(3).unwrap();
		let spidev = Spidev::open("/dev/spidev0.0").unwrap();
		let mut rfm1 = Rfm69::new(rst, en, spidev).unwrap();
		let mut pc = PacketConfig::default();
		pc.length = 16;
		rfm1.set_config(&pc).unwrap();

		let rst = chip.get_line(2).unwrap();
		let en = chip.get_line(4).unwrap();
		let spidev = Spidev::open("/dev/spidev0.1").unwrap();
		let mut rfm2 = Rfm69::new(rst, en, spidev).unwrap();
		rfm2.set_config(&pc).unwrap();
		let msg: Vec<u8> = (0..16_u8).collect();
		let recv = spawn(move || {
			let msg = rfm1.recv(Duration::from_secs(5)).unwrap();
			let comp: Vec<u8> = (0..16_u8).collect();
			assert_eq!(msg,comp);
			rfm1
		});	
		sleep(Duration::from_secs(1));
		rfm2.send(&msg).unwrap();
		recv.join().unwrap(); // check if other thread paniced
	}

}
