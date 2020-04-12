
use crate::{bit,cond_set_bit,set_bit,Error,set_bit_to,sleep,unset_bit,ConfigMessage,Void};
use crate::{PacketReceiver,NetworkPacketReceiver,AddressPacketReceiver,BroadcastPacketReceiver};
use crate::{IntoPacketReceiver,Address};
use crate::rfm69::{SyncConfig,Rfm69,PacketConfig,DCFree,Filtering,Mode};

use reed_solomon::Decoder;
use std::sync::mpsc::{SyncSender,Receiver,sync_channel,channel};
use std::sync::{Arc,RwLock};
use std::thread::{JoinHandle,spawn};
use std::time::{Instant,Duration};


pub struct Rfm69PR {
	rfm_thread: JoinHandle<Result<Rfm69,(Error, Rfm69)>>,
	conf_sender: SyncSender<ConfigMessage<[u8; 8], u8>>,
	msg_recv: Receiver<(Vec<u8>, Address)>,
	clock: Arc<RwLock<(Instant, u32)>>
}
impl Rfm69PR {
	pub fn terminate(mut self) -> Result<Rfm69, (Error, Option<Rfm69>)> {
		self.conf_sender.send(ConfigMessage::Terminate).ok(); // ignore error
		self.rfm_thread.join().map_err(|_| {(Error::Unrecoverable("The receiver thread paniced!".to_string()), None)})?
			.map_err(|e| { (e.0, Some(e.1)) })
	}
}
fn config_net(netaddr: [u8; 8], rfm: &mut Rfm69) -> Result<(), Error> {
	let mut sc = SyncConfig::default();
	let len = netaddr.iter().position(|x| *x == 0x00).unwrap_or(8);
	sc.set_sync_word(&netaddr[..len]);
	sc.set_on(len != 0);
	let mut pc = rfm.config();
	if len != 0 {
		sc.set_error(0);
		sc.set_len(len as u8);
		if pc.dc() != DCFree::Whitening { rfm.set_config(pc.set_dc(DCFree::Whitening))? }
	} else if pc.dc() != DCFree::None {
		rfm.set_config(pc.set_dc(DCFree::None))?;
	}
	rfm.set_sync(sc)
}
fn config_addr(addr: u8, rfm: &mut Rfm69) -> Result<(), Error> {
	let mut pc = *(rfm.config().set_address(addr));
	if addr == 0 {
		pc.set_filtering(Filtering::None);		
	} else if pc.broadcast() == 0 {
		pc.set_filtering(Filtering::Address);
	} else {
		pc.set_filtering(Filtering::Both);
	}
	if pc != rfm.config() { rfm.set_config(pc) } else { Ok(()) }
}
fn config_broad(addr: u8, rfm: &mut Rfm69) -> Result<(), Error> { 
	let mut pc = *(rfm.config().set_broadcast(addr));
	if addr == 0 {
		pc.set_filtering(Filtering::None);		
	} else if pc.broadcast() == 0 {
		pc.set_filtering(Filtering::Address);
	} else {
		pc.set_filtering(Filtering::Both);
	}
	if pc != rfm.config() { rfm.set_config(pc) } else { Ok(()) }
}

impl IntoPacketReceiver for Rfm69 {
	type Recv = Rfm69PR;
	fn into_packet_receiver(mut self) -> Result<Self::Recv, Error> {
		self.set_mode_internal(Mode::Standby)?;
		let (conf_sender, conf_recv) = sync_channel(0);
		let (msg_sender, msg_recv) = channel();
		let clock = Arc::new(RwLock::new((Instant::now(), 0)));
		let clock_clone = clock.clone();
		let rfm_thread = spawn(move ||{ 
			let clock = clock_clone;
			let mut init_dev = ||{
				let pc = *PacketConfig::default().set_variable(true).set_crc(false);
				self.set_config(pc)?;
				self.get_mode_ready()?.check_and_wait(Duration::from_millis(10))?;
				self.set_mode(Mode::Rx)
			};
			// configure Rfm69 device for receiving and catch error
			if let Err(e) = init_dev() {
				return Err((Error::Init(format!("Reader configuration failed: {:?}", e)), self));
			}
			let mut paused = true;
			let decoder = Decoder::new(16);
			loop {
				// allocate space for the message
				if paused {
					loop {
						if let Ok(v) = conf_recv.recv() {
							if let Err(e) = match v {
								// TODO: is thre some way to eliminate the duplicate code on lines 816
								ConfigMessage::SetNetwork(netaddr) => config_net(netaddr, &mut self),
								ConfigMessage::SetAddr(addr) => config_addr(addr, &mut self),
								ConfigMessage::SetBroadcast(addr) => config_broad(addr, &mut self),
								ConfigMessage::Terminate => return Ok(self),
								ConfigMessage::Start => {paused = false; break;},
								ConfigMessage::Pause => Ok(()),
								_ => unreachable!()
							} 
							{
								return Err((Error::Unrecoverable(format!("Reader thread: Device could not be configured!: {:?}", e)), self));
							}
						} else {
							return Err((Error::Unrecoverable("Reader thread: Receiver is paused and conf_recv is disconnected".to_string()), self));
						
						}
					}
				} else {
					let mut buf = [0; 255];
					let res = self.recv(&mut buf, Duration::from_millis(1));
					let now = Instant::now();
					if let Err(e) = &res {
						// handle error on 
						let unrecoverable = match e {
							Error::BadMessage(_,_)|Error::Timeout(_) => false, 
							_ => true
						};
						if unrecoverable {
							return Err((Error::Unrecoverable(format!("Receive thread: unrecoverable error occurred when receiving: {:?}", e)), self));
						}
					}
					let cfg_msg = conf_recv.try_recv().ok();
					if let Some(v) = cfg_msg {
						if let Err(e) = match v {
							ConfigMessage::SetNetwork(netaddr) => config_net(netaddr, &mut self),
							ConfigMessage::SetAddr(addr) => config_addr(addr, &mut self),
							ConfigMessage::SetBroadcast(addr) => config_broad(addr, &mut self),
							ConfigMessage::Terminate => return Ok(self),
							ConfigMessage::Start => {paused = false; Ok(())},
							ConfigMessage::Pause => Ok(()),
							_ => unreachable!()
						} 
						{
								return Err((Error::Unrecoverable(format!("Reader thread: Device could not be configured!: {:?}", e)), self));
						}
					} else {
						let size = res.unwrap();
						let sc = self.sync();
						let sync_len = (sc.on() as u8 * sc.len()) as u32;
						let start = now.checked_sub(Duration::from_secs_f64(8.0 * (size as u32 + sync_len + self.preamble_len() as u32 + 1) as f64 / self.bitrate() as f64)).unwrap_or(now);
						if size >= 16 {
							let data = &buf[..size];
							if let Ok(buf) =  decoder.correct(&data, None) {
								let buf = buf.data();
								let mut time = [0; 4];
								time.copy_from_slice(&buf[0..4]);
								let time = u32::from_be_bytes(time);
								let mut vec = Vec::new();
								let t = match self.pc.filtering() {
									Filtering::None => { 
										vec.extend_from_slice(&buf[4..]);
										Address::None
									},
									Filtering::Address|Filtering::Both => {
										vec.extend_from_slice(&buf[5..]);
										if buf[0] == self.pc.address() { Address::Address } else { Address::Broadcast }
									},
									_ => unreachable!()
										
								};
								if let Err(_) = msg_sender.send((vec, t)) {
									return Err((Error::Unrecoverable("Reader thread: Receiver Message is disconnected".to_string()), self));
								}
								if let Ok(mut lock) = clock.write() {
									*lock = (start, time);
								} else {
									return Err((Error::Unrecoverable("Reader thread: Clock lock is poisoned!".to_string()), self));
								}
							}
						}
					}
				}
			}
		});
		Ok(Rfm69PR{rfm_thread, conf_sender, msg_recv, clock})
	}

}

impl PacketReceiver for Rfm69PR {
	fn cur_time(&self) -> Result<u32, Error> {
		let time = self.clock.read().map_err(|_| {Error::Unrecoverable("Packet receiver poisoned the clock lock!".to_string())})?;
		let diff = Instant::now().duration_since(time.0).as_micros() as u32;
		Ok(diff.wrapping_add(time.1))
	}
	fn recv_packet(&mut self) -> Result<(Vec<u8>, Address), Error> {
		self.msg_recv.recv().map_err(|_| {Error::Unrecoverable("Packet receiver thread is disconnected!".to_string())})
	}
}
impl NetworkPacketReceiver<&[u8]> for Rfm69PR {
	fn set_network(&mut self, netaddr: &[u8]) -> Result<(), Error> {
		assert!(netaddr.len() <= 8);
		let mut msg = [0;8];
		msg[..netaddr.len()].copy_from_slice(&netaddr[..netaddr.len()]);
		self.conf_sender.send(ConfigMessage::SetNetwork(msg)).map_err(|_| Error::Unrecoverable("Packet receiver thread is disconnected.".to_string()))
	}
}
impl AddressPacketReceiver<&[u8],u8> for Rfm69PR {
	fn set_addr(&mut self, addr: u8) -> Result<(), Error> {
		self.conf_sender.send(ConfigMessage::SetAddr(addr)).map_err(|_| Error::Unrecoverable("Packet receiver thread is disconnected.".to_string()))
	}
}
impl BroadcastPacketReceiver<&[u8], u8> for Rfm69PR {
	fn set_broadcast(&mut self, addr: u8) -> Result<(), Error> {
		self.conf_sender.send(ConfigMessage::SetBroadcast(addr)).map_err(|_| Error::Unrecoverable("Packet receiver thread is disconnected.".to_string()))
	}
}

