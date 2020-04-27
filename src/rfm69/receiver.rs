use crate::rfm69::{DCFree, Filtering, Mode, PacketConfig, Rfm69, SyncConfig};
use crate::IntoPacketReceiver;
use crate::{
    AddressPacketReceiver, BroadcastPacketReceiver, NetworkPacketReceiver, PacketReceiver,
};
use crate::{ConfigMessage, Error};

use reed_solomon::Decoder;
use std::sync::mpsc::{channel, sync_channel, Receiver, RecvTimeoutError, SyncSender};
use std::sync::{Arc, RwLock};
use std::thread::{Builder as ThreadBuilder, JoinHandle};
use std::time::{Duration, Instant};

pub struct Rfm69PR {
    rfm_thread: JoinHandle<Result<Rfm69, (Error, Rfm69)>>,
    conf_sender: SyncSender<ConfigMessage<[u8; 4], u8>>,
    msg_recv: Receiver<(Vec<u8>, u8, Instant, u32)>,
    clock_instant: Instant,
    clock_time: u32,
    started: bool,
    verbose: u8,
}
impl Rfm69PR {
    pub fn terminate(self) -> Result<Rfm69, (Error, Option<Rfm69>)> {
        self.conf_sender.send(ConfigMessage::Terminate).ok(); // ignore error
        self.rfm_thread
            .join()
            .map_err(|_| {
                (
                    Error::Unrecoverable("The receiver thread paniced!".to_string()),
                    None,
                )
            })?
            .map_err(|e| (e.0, Some(e.1)))
    }
    fn configure(&self, conf_msg: ConfigMessage<[u8; 4], u8>) -> Result<(), Error> {
        self.conf_sender.send(conf_msg).map_err(|_| {
            Error::Unrecoverable("Packet receiver thread is disconnected.".to_string())
        })
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
fn message(
    conf_msg: ConfigMessage<[u8; 4], u8>,
    rfm: &mut Rfm69,
    paused: &mut bool,
    verbose: &mut u8,
) -> Result<bool, Error> {
    match conf_msg {
        ConfigMessage::SetNetwork(netaddr) => {
            let mut sc = SyncConfig::default();
            let sync_word = [
                0x56, 0xA9, 0x09, 0x9A, netaddr[0], netaddr[1], netaddr[2], netaddr[3],
            ];
            let len = netaddr.iter().position(|x| *x == 0x00).unwrap_or(4) + 4;
            sc.set_sync_word(&sync_word[..len]);
            if sc != rfm.sync() {
                rfm.set_sync(sc)?;
            }
            Ok(false)
        }
        ConfigMessage::SetAddr(addr) => {
            let pc = rfm.config().set_address(addr);
            if pc != rfm.config() {
                rfm.set_config(pc)?;
            }
            Ok(false)
        }
        ConfigMessage::SetBroadcast(addr) => {
            let pc = rfm.config().set_broadcast(addr);
            if pc != rfm.config() {
                rfm.set_config(pc)?;
            }
            Ok(false)
        }
        ConfigMessage::Verbose(v) => {
            *verbose = v;
            rfm.set_verbose(v != 0);
            Ok(false)
        }
        ConfigMessage::Start => {
            if let Err(_) = rfm.set_mode(Mode::Rx) {
                Err(Error::Unrecoverable(
                    "Reader thread: Rx mode could not be started.".to_string(),
                ))
            } else {
                *paused = false;
                Ok(false)
            }
        }
        ConfigMessage::Pause => {
            if let Err(_) = rfm.set_mode(Mode::Standby) {
                Err(Error::Unrecoverable(
                    "Reader thread: Rx mode could not be started.".to_string(),
                ))
            } else {
                *paused = true;
                Ok(false)
            }
        }
        ConfigMessage::Terminate => Ok(true),
        ConfigMessage::Alive => Ok(false),
        _ => unreachable!(),
    }
}

impl IntoPacketReceiver for Rfm69 {
    type Recv = Rfm69PR;
    fn into_packet_receiver(mut self) -> Result<Self::Recv, Error> {
        self.set_mode_internal(Mode::Standby)?;
        let (conf_sender, conf_recv) = sync_channel(0);
        let (msg_sender, msg_recv) = channel();
        let builder = ThreadBuilder::new().name("rfm69_sender".to_string());
        let rfm_thread = builder.spawn(move ||{
			let mut init_dev = ||{
				let pc = PacketConfig::default().set_variable(true).set_len(255).set_crc(false).set_dc(DCFree::Whitening);
				self.set_config(pc)?;
				let sc = *SyncConfig::default().set_sync_word(&[0x56, 0xa9, 0x0b, 0x9a]).set_len(4);
				self.set_sync(sc)?;
				self.get_mode_ready()?.check_and_wait(Duration::from_millis(10))
			};
			// configure Rfm69 device for receiving and catch error
			if let Err(e) = init_dev() {
				return Err((Error::Init(format!("Reader configuration failed: {:?}", e)), self));
			}
			let mut paused = true;
			let mut verbose = 0;
			let decoder = Decoder::new(16);
			loop {
				// allocate space for the message
				if paused {
					if let Ok(v) = conf_recv.recv() {
						match message(v, &mut self, &mut paused, &mut verbose) {
							Ok(terminate) => if terminate { return Ok(self); },
							Err(e) => return Err((e, self))
						}
					} else {
						return Err((Error::Unrecoverable("Reader thread: Receiver is paused and conf_recv is disconnected".to_string()), self));
					}
				} else {
					let mut buf = [0; 255];
					let res = self.recv(&mut buf, Duration::from_millis(1000));
					let now = Instant::now();
					if let Err(e) = &res {
						// handle error on 
						match e {
							Error::BadMessage(_,_)|Error::Timeout(_) =>(),
							_=> return Err((Error::Unrecoverable(format!("Receive thread: unrecoverable error occurred when receiving: {:?}", e)), self))
						}
					}
					let cfg_msg = conf_recv.try_recv().ok();
					if let Some(v) = cfg_msg {
						match message(v, &mut self, &mut paused, &mut verbose) {
							Ok(terminate) => if terminate { return Ok(self); },
							Err(e) => return Err((e, self))
						}
					} else {
						let size = if let Ok(size) = res { size } else { continue; };
						let sc = self.sync();
						let sync_len = (sc.on() as u8 * sc.len()) as u32;
						let start = now.checked_sub(Duration::from_secs_f64(8.0 * (size as u32 + sync_len + self.preamble_len() as u32 + 1) as f64 / self.bitrate() as f64)).unwrap_or(now);
						if size >= 16 {
							let data = &buf[..size];
                            if verbose >= 3 {
                                eprintln!("Reader thread: Rssi: {:?}", self.rssi());
                            }
							if let Ok(buf) =  decoder.correct(&data, None) {
								let buf = buf.data();
								let mut time = [0; 4];
								time.copy_from_slice(&buf[1..5]);
								let time = u32::from_be_bytes(time);
								let mut vec = Vec::new();
								vec.extend_from_slice(&buf[5..]);
								if let Err(_) = msg_sender.send((vec, buf[0], start, time)) {
									return Err((Error::Unrecoverable("Reader thread: Receiver Message is disconnected".to_string()), self));
								}
							} else {
								if verbose >= 1 { eprintln!("Message received, but bad decode occurred!: {:X?}", data); }
							}
						}
					}
				}
			}
		}).unwrap();
        Ok(Rfm69PR {
            rfm_thread,
            conf_sender,
            msg_recv,
            clock_instant: Instant::now(),
            clock_time: 0,
            started: false,
            verbose: 0,
        })
    }
}

impl PacketReceiver for Rfm69PR {
    #[inline]
    fn cur_time(&self) -> u32 {
        let diff = Instant::now()
            .duration_since(self.clock_instant)
            .as_micros() as u32;
        diff.wrapping_add(self.clock_time)
    }
    #[inline]
    fn last_time(&self) -> u32 {
        self.clock_time
    }
    fn recv_pkt(&mut self) -> Result<Vec<u8>, Error> {
        assert!(self.started);
        let (msg, _, inst, time) = self.msg_recv.recv().map_err(|_| {
            Error::Unrecoverable("Packet receiver thread is disconnected!".to_string())
        })?;
        self.clock_instant = inst;
        self.clock_time = time;
        Ok(msg)
    }
    fn recv_pkt_to(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        assert!(self.started);
        let (msg, _, inst, time) = self.msg_recv.recv_timeout(timeout).map_err(|e| match e {
            RecvTimeoutError::Timeout => Error::Timeout("Packet reception timed out.".to_string()),
            RecvTimeoutError::Disconnected => {
                Error::Unrecoverable("Packet receiver thread is disconnected!".to_string())
            }
        })?;
        self.clock_instant = inst;
        self.clock_time = time;
        Ok(msg)
    }
    fn start(&mut self) -> Result<(), Error> {
        self.configure(ConfigMessage::Start)?;
        self.started = true;
        Ok(())
    }
    fn pause(&mut self) -> Result<(), Error> {
        self.configure(ConfigMessage::Pause)?;
        self.started = false;
        Ok(())
    }
    #[inline]
    fn mtu(&self) -> usize {
        233
    }
}
impl NetworkPacketReceiver<&[u8]> for Rfm69PR {
    fn set_network(&mut self, netaddr: &[u8]) -> Result<(), Error> {
        assert!(netaddr.len() <= 4);
        let mut msg = [0; 4];
        msg[..netaddr.len()].copy_from_slice(&netaddr[..netaddr.len()]);
        self.configure(ConfigMessage::SetNetwork(msg))
    }
}
impl AddressPacketReceiver<&[u8], u8> for Rfm69PR {
    fn recv_pkt_addr(&mut self) -> Result<(Vec<u8>, u8), Error> {
        assert!(self.started);
        let (msg, addr, inst, time) = self.msg_recv.recv().map_err(|_| {
            Error::Unrecoverable("Packet receiver thread is disconnected!".to_string())
        })?;
        self.clock_instant = inst;
        self.clock_time = time;
        Ok((msg, addr))
    }
    fn recv_pkt_to_addr(&mut self, timeout: Duration) -> Result<(Vec<u8>, u8), Error> {
        assert!(self.started);
        let (msg, addr, inst, time) = self.msg_recv.recv_timeout(timeout).map_err(|e| match e {
            RecvTimeoutError::Timeout => Error::Timeout("Packet reception timed out.".to_string()),
            RecvTimeoutError::Disconnected => {
                Error::Unrecoverable("Packet receiver thread is disconnected!".to_string())
            }
        })?;
        self.clock_instant = inst;
        self.clock_time = time;
        Ok((msg, addr))
    }
    #[inline]
    fn set_addr(&mut self, addr: u8) -> Result<(), Error> {
        self.configure(ConfigMessage::SetAddr(addr))
    }
}
impl BroadcastPacketReceiver<&[u8], u8> for Rfm69PR {
    #[inline]
    fn set_broadcast(&mut self, addr: u8) -> Result<(), Error> {
        self.configure(ConfigMessage::SetAddr(addr))
    }
}
