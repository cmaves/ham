use crate::rfm69::{DCFree, Mode, PacketConfig, Rfm69, SyncConfig};
use crate::sleep;
use crate::IntoPacketSender;
use crate::{ConfigMessage, Error};
use crate::{NetworkPacketSender, PacketSender};

use reed_solomon::Encoder;
use std::sync::mpsc::{sync_channel, SyncSender};
use std::thread::{Builder as ThreadBuilder, JoinHandle};
use std::time::{Duration, Instant};

pub struct Rfm69PS {
    rfm_thread: JoinHandle<Result<Rfm69, (Error, Rfm69)>>,
    conf_sender: SyncSender<ConfigMessage<[u8; 4], u8>>,
    encoder: Encoder,
    verbose: u8,
}
impl Rfm69PS {
    pub fn terminate(self) -> Result<Rfm69, (Error, Option<Rfm69>)> {
        self.conf_sender.send(ConfigMessage::Terminate).ok();
        self.rfm_thread
            .join()
            .map_err(|_| {
                (
                    Error::Unrecoverable("The sender thread panicked!".to_string()),
                    None,
                )
            })?
            .map_err(|e| (e.0, Some(e.1)))
    }
    fn configure(&self, conf_msg: ConfigMessage<[u8; 4], u8>) -> Result<(), Error> {
        self.conf_sender
            .send(conf_msg)
            .map_err(|_| Error::Unrecoverable("Packet sender thread is disconnected.".to_string()))
    }
    pub fn alive(&mut self) -> Result<(), Error> {
        self.configure(ConfigMessage::Alive)
    }
    pub fn set_verbose(&mut self, verbose: u8) -> Result<(), Error> {
        self.configure(ConfigMessage::Verbose(verbose))?;
        self.verbose = verbose;
        Ok(())
    }
}

impl PacketSender for Rfm69PS {
    fn send_packet(&mut self, msg: &[u8], start_time: u32) -> Result<(), Error> {
        assert!(msg.len() <= self.mtu());
        let now = Instant::now();
        let msglen = msg.len() + 16 + 4 + 1; // msg + ecc + time + zero-address byte
        let mut vec = Vec::with_capacity(msglen + 1); // msglen + len byte
        vec.push(msglen as u8);
        vec.push(0);
        vec.extend_from_slice(&start_time.to_be_bytes());
        vec.extend_from_slice(msg);
        let encoded = self.encoder.encode(&vec[1..]);
        vec.extend_from_slice(encoded.ecc());
        let vec = ConfigMessage::SendMessage(vec, now);
        self.conf_sender
            .send(vec)
            .map_err(|_| Error::Unrecoverable("Sending thread is disconnected!".to_string()))
    }
    #[inline]
    fn mtu(&self) -> usize {
        233
    }
}

impl NetworkPacketSender<&[u8]> for Rfm69PS {
    fn set_network(&mut self, netaddr: &[u8]) -> Result<(), Error> {
        assert!(netaddr.len() <= 4);
        let mut msg = [0; 4];
        msg[..netaddr.len()].copy_from_slice(&netaddr[..netaddr.len()]);
        self.configure(ConfigMessage::SetNetwork(msg))
    }
}

impl IntoPacketSender for Rfm69 {
    type Send = Rfm69PS;
    fn into_packet_sender(mut self, msg_buf: usize) -> Result<Self::Send, Error> {
        self.set_mode_internal(Mode::Standby)?;
        let (conf_sender, conf_recv) = sync_channel(msg_buf);
        let encoder = Encoder::new(16);
        let builder = ThreadBuilder::new().name("rfm69_sender".to_string());
        let rfm_thread = builder.spawn(move || {
            let mut verbose = 0;
			let mut init_dev = ||{
				let pc = PacketConfig::default().set_variable(true).set_crc(false).set_dc(DCFree::Whitening);
				self.set_config(pc)?;
				let sc = *SyncConfig::default().set_sync_word(&[0x56, 0xa9, 0x0b, 0x9a]).set_len(4);
				self.set_sync(sc)?;
				self.get_mode_ready()?.check_and_wait(Duration::from_millis(10))?;
				self.set_mode(Mode::Tx)
			};
			// configure Rfm69 device for receiving and catch error
			if let Err(e) = init_dev() {
				return Err((Error::Init(format!("Reader configuration failed: {:?}", e)), self));
			}
            let mut last_msg_settle = Instant::now();
			loop {
				if let Ok(v) = conf_recv.recv() {
					match v {
						ConfigMessage::SendMessage(mut msg, start) => {
                            let time = ((msg[2] as u32) << 24) + ((msg[3] as u32) << 16) + ((msg[4] as u32) << 8) + msg[5] as u32;
                            sleep(last_msg_settle.saturating_duration_since(Instant::now()));
                            let diff = Instant::now().duration_since(start).as_micros() as u32;
                            if verbose >= 3 {
                                eprintln!("rfm69_sender thread: channel delay {} ms", diff);
                            }
                            let time = time.wrapping_add(diff).to_be_bytes();
                            msg[2..6].copy_from_slice(&time);
                            #[cfg(debug)]{
                            if verbose >= 4 {
                                eprintln!("rfm69_sender thread: sending {:2x?}", msg);
                            }}
							if let Err(e) = self.send(&msg) {
								return Err((Error::Unrecoverable(format!("Receive error: error occured when sending message!: {:?}", e)), self))
							}
                            last_msg_settle =  Instant::now() + Duration::from_secs_f64(1.0 * 8.0 / self.bitrate() as f64);
						},
						ConfigMessage::Terminate => return Ok(self),
						ConfigMessage::Alive => (),
						ConfigMessage::Verbose(v) => {
							verbose = v;
							self.set_verbose(v != 0)
						}
						_ => unreachable!()
					}
				} else {
					return Err((Error::Unrecoverable("Sender thread: Sender message channel is disconnected!".to_string()), self))

				}
			}
		}).unwrap();
        Ok(Rfm69PS {
            conf_sender,
            rfm_thread,
            encoder,
            verbose: 0,
        })
    }
}
