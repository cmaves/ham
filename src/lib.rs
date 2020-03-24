

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

#[cfg(test)]
mod tests {
	use gpio_cdev::Chip;
	use crate::rfm69::Rfm69;
	use spidev::Spidev;

	#[test]
	fn make_rfm69() {
		let mut chip = Chip::new("/dev/gpiochip0").unwrap();
		let rst = chip.get_line(24).unwrap();
		let en = chip.get_line(26).unwrap();
		let g0 = chip.get_line(25).unwrap();
		let spidev = Spidev::open("/dev/spidev0.0").unwrap();
		let rfm69 = Rfm69::new(rst, en, g0, spidev).unwrap();
		for reg in rfm69.read_all().unwrap().iter().enumerate() {
			println!("Reg {:#04X}: {:#04X}", reg.0 + 1, reg.1);
		}
	}
}
