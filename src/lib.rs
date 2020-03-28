

pub mod rfm69;

#[cfg(test)]
pub mod tests;

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
