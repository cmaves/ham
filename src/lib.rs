
use std::thread;
use std::time::Duration;

#[cfg(feature = "hp-sleep")]
use spin_sleep;

pub mod rfm69;


#[derive(Debug)]
pub enum Error {
	GpioError(gpio_cdev::errors::Error), // error with Gpio device
	Init(String), // error initializing device TODO: should this be removed
	IoError(std::io::Error), // IO error, typically SPI
	BadInputs(String), // input given to message was bad
	BadMessage(String,Option<usize>), // a mangled message was received
	Timeout(String), // 
	ChipMalfunction(String), // chip is behaving unexpectedly
	BufferOverflow(usize), // a buffer that was passed was to small
	Unrecoverable(String) // device is unrecoverable state
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
fn sleep(dur: Duration) {
	// a high persision sleep version
	if cfg!(feature = "hp-sleep") {
		spin_sleep::sleep(dur)
	} else {
		thread::sleep(dur)
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

#[inline]
fn set_bit_to(byte: u8, bit: u8, val: bool) -> u8 {
	cond_set_bit(unset_bit(byte, bit), bit, val)
}


trait PacketReceiver {
	fn cur_time(&self) -> Result<u32, Error>;
	fn recv_packet(&mut self) -> Result<(Vec<u8>, Address), Error>;
}
trait IntoPacketReceiver {
	type Recv: PacketReceiver;
	fn into_packet_receiver(self) -> Result<Self::Recv, Error>;
}
impl <T: PacketReceiver> IntoPacketReceiver for T {
	type Recv = T;
	#[inline] fn into_packet_receiver(self) -> Result<Self::Recv, Error> { Ok(self) }
}

trait NetworkPacketReceiver<N>: PacketReceiver {
	fn set_network(&mut self, netaddr: N) -> Result<(), Error>;
}
trait AddressPacketReceiver<N, A>: NetworkPacketReceiver<N> {
	fn set_addr(&mut self, addr: A) -> Result<(), Error>;
}
trait BroadcastPacketReceiver<N, A>: AddressPacketReceiver<N, A> {
	fn set_broadcast(&mut self, addr: A) -> Result<(), Error>;
}
trait VerifiedPacketReceiver: PacketReceiver {
	fn recv_v_packet(&mut self, bytes: &mut [u8]) -> Result<usize, Error>;
}

trait PacketSender {
	fn send_packet(&mut self, msg: &[u8], start_time: u32) -> Result<(), Error>;
}
trait IntoPacketSender {
	type Send: PacketSender;
	fn into_packet_sender(self) -> Result<Self::Send, Error>;
}
trait AddressPacketSender<N, A>: NetworkPacketReceiver<N> {
	fn send_packet_to(&mut self, addr: A) -> Result<(), Error>;
}
trait VerifiedPacketSender: PacketSender {
	fn send_v_packet(&self) -> Result<(), Error>;
}
impl <T: PacketSender> IntoPacketSender for T {
	type Send = T;
	fn into_packet_sender(self) -> Result<Self::Send, Error> {
		Ok(self)
	}
}

enum ConfigMessage<N,A> 
{
	SetNetwork(N),
	SetAddr(A),
	SetBroadcast(A),
	SendMessage(Vec<u8>),
	Terminate,
	Pause,
	Start
}
pub enum Address {
	None,
	Address,
	Broadcast
}

pub enum Void {}
