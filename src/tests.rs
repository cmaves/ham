use gpio_cdev::Chip;
use crate::rfm69::{PacketConfig,Rfm69,SyncConfig};
use spidev::Spidev;
use std::thread::{spawn,sleep};
use std::time::Duration;
use rand::prelude::*;

fn test_sync_config() -> SyncConfig {
	let mut sc = SyncConfig::default();
	for i in 0..8 {
		sc.syncword[i] = i as u8 + 1;	
	}
	sc
}
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
	let rfm69 = Rfm69::new(rst, en, spidev).unwrap();
	let sc = test_sync_config();
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
	let sc = test_sync_config();

	pc.set_len(16);
	rfm1.set_config(&pc).unwrap();
	rfm1.set_sync(&sc).unwrap();

	let rst = chip.get_line(2).unwrap();
	let en = chip.get_line(4).unwrap();
	let spidev = Spidev::open("/dev/spidev0.1").unwrap();
	let mut rfm2 = Rfm69::new(rst, en, spidev).unwrap();
	rfm2.set_config(&pc).unwrap();
	rfm2.set_sync(&sc).unwrap();

	let mut msg = vec![0_u8; 16];
	let mut rng = thread_rng();
	rng.try_fill(msg.as_mut_slice()).unwrap();
	let copy = msg.clone();
	let recv = spawn(move || {
		let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
		assert_eq!(recvd,copy);
		rfm1
	});	
	sleep(Duration::from_secs(1));
	rfm2.send(&msg).unwrap();
	let mut rfm1 = recv.join().unwrap(); // check if other thread paniced

	pc.set_len(32);
	rfm1.set_config(&pc).unwrap();
	rfm2.set_config(&pc).unwrap();

	msg.resize(32, 0);
	rng.try_fill(msg.as_mut_slice()).unwrap();
	
	let copy = msg.clone();
	let recv = spawn(move || {
		let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
		assert_eq!(recvd,copy);
		rfm1
	});	

	sleep(Duration::from_secs(1));
	rfm2.send(&msg).unwrap();
	let mut rfm1 = recv.join().unwrap();

	pc.set_len(128);
	rfm1.set_config(&pc).unwrap();
	rfm2.set_config(&pc).unwrap();

	msg.resize(128, 0);
	rng.try_fill(msg.as_mut_slice()).unwrap();
	let copy = msg.clone();
	let recv = spawn(move || {
		let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
		assert_eq!(recvd,copy);
		rfm1
	});	

	sleep(Duration::from_secs(1));
	rfm2.send(&msg).unwrap();
	let mut rfm1 = recv.join().unwrap();

	pc.set_len(255);
	rfm1.set_config(&pc).unwrap();
	rfm2.set_config(&pc).unwrap();

	msg.resize(255, 0);
	rng.try_fill(msg.as_mut_slice()).unwrap();
	let copy = msg.clone();
	let recv = spawn(move || {
		let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
		assert_eq!(recvd,copy);
		rfm1
	});	

	sleep(Duration::from_secs(1));
	rfm2.send(&msg).unwrap();
	recv.join().unwrap();

}

#[test]
fn config() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
	let mut pc = PacketConfig::default();
	pc.set_len(255);
	
	rfm.set_config(&pc).unwrap();
	assert_eq!(pc, rfm.config());

	let copy = rfm.config_dev().unwrap();
	assert_eq!(pc, copy);
}
