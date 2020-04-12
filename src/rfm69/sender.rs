
use crate::{bit,cond_set_bit,set_bit,Error,set_bit_to,sleep,unset_bit,ConfigMessage,Void};
use crate::{PacketSender,NetworkPacketReceiver,AddressPacketReceiver,BroadcastPacketReceiver};
use crate::IntoPacketSender;
use crate::rfm69::{SyncConfig,Rfm69,PacketConfig,DCFree,Filtering,Mode};

use reed_solomon::Encoder;
use std::sync::mpsc::{channel,Sender};
use std::thread::{JoinHandle,spawn};
use std::time::Duration;

pub struct Rfm69PS {
	rfm_thread: JoinHandle<Result<Rfm69,(Error, Rfm69)>>,
	conf_sender: Sender<ConfigMessage<[u8; 8], u8>>,
	encoder: Encoder
}
impl Rfm69PS {
	pub fn terminate(self) -> Result<Rfm69, (Error, Option<Rfm69>)> {
		self.conf_sender.send(ConfigMessage::Terminate).ok();
		self.rfm_thread.join().map_err(|_| (Error::Unrecoverable("The sender thread panicked!".to_string()), None))?
			.map_err(|e| (e.0, Some(e.1)))
	}
}

impl PacketSender for Rfm69PS {
	fn send_packet(&mut self, msg: &[u8], start_time: u32) -> Result<(), Error> {
		assert!(msg.len() <= 235);
		let mut vec = Vec::with_capacity(msg.len() + 16 + 4);
		vec.extend_from_slice(&start_time.to_be_bytes());
		vec.extend_from_slice(msg);
		let encoded = self.encoder.encode(&vec[..]);
		vec.extend_from_slice(encoded.ecc());
		let vec = ConfigMessage::SendMessage(vec);
		self.conf_sender.send(vec).map_err(|_| Error::Unrecoverable("Sending thread is disconnected!".to_string()))
	}
}
	

impl IntoPacketSender for Rfm69 {
	type Send = Rfm69PS;
	fn into_packet_sender(mut self) -> Result<Self::Send, Error> {
		self.set_mode_internal(Mode::Standby)?;
		let (conf_sender, conf_recv) = channel();
		let encoder = Encoder::new(16);
		let rfm_thread = spawn(move || {
			let mut init_dev = ||{
				let pc = *PacketConfig::default().set_variable(true).set_crc(false);
				self.set_config(pc)?;
				self.get_mode_ready()?.check_and_wait(Duration::from_millis(10))?;
				self.set_mode(Mode::Tx)
			};
			// configure Rfm69 device for receiving and catch error
			if let Err(e) = init_dev() {
				return Err((Error::Init(format!("Reader configuration failed: {:?}", e)), self));
			}
			loop {
				if let Ok(v) = conf_recv.recv() {
					match v {
						ConfigMessage::SendMessage(msg) => {
							if let Err(e) = self.send(&msg) {
								return Err((Error::Unrecoverable(format!("Receive error: error occured when sending message!: {:?}", e)), self))
							}
						},
						ConfigMessage::Terminate => return Ok(self),
						_ => unreachable!()
					}
					unimplemented!();
				} else {
					return Err((Error::Unrecoverable("Sender thread: Sender message channel is disconnected!".to_string()), self))

				}
			}
		});
		Ok(Rfm69PS{conf_sender, rfm_thread, encoder})
	}
}

