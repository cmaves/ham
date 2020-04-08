use gpio_cdev::Chip;
use crate::rfm69::{PacketConfig,Rfm69,SyncConfig,Mode};
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
fn send_recv_fixed() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm1 = Rfm69::new(rst, en, spidev).unwrap();
	let mut pc = PacketConfig::default();
	let sc = test_sync_config();

	rfm1.set_sync(&sc).unwrap();

	let rst = chip.get_line(2).unwrap();
	let en = chip.get_line(4).unwrap();
	let spidev = Spidev::open("/dev/spidev0.1").unwrap();
	let mut rfm2 = Rfm69::new(rst, en, spidev).unwrap();
	rfm2.set_sync(&sc).unwrap();

	let mut msg = Vec::with_capacity(255);
	let mut rng = thread_rng();

	for i in &[8_u8, 16, 32, 64, 128, 255] {
		pc.set_len(*i);
		rfm1.set_config(&pc).unwrap();
		rfm2.set_config(&pc).unwrap();
	
		msg.resize(*i as usize, 0);
		rng.try_fill(msg.as_mut_slice()).unwrap();
		let copy = msg.clone();
		let recv = spawn(move || {
			let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
			assert_eq!(recvd,copy);
			rfm1
		});	
		sleep(Duration::from_secs(1));
		rfm2.send(&msg).unwrap();
		rfm1 = recv.join().unwrap(); // check if other thread paniced
	}
}
#[test]
fn send_recv_variable() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm1 = Rfm69::new(rst, en, spidev).unwrap();
	let pc = *PacketConfig::default().set_variable(true).set_threshold(4).set_len(255);


	let rst = chip.get_line(2).unwrap();
	let en = chip.get_line(4).unwrap();
	let spidev = Spidev::open("/dev/spidev0.1").unwrap();
	let mut rfm2 = Rfm69::new(rst, en, spidev).unwrap();

	let mut msg = Vec::with_capacity(256);
	let mut rng = thread_rng();

	rfm1.set_config(&pc).unwrap();
	rfm2.set_config(&pc).unwrap();
	let sc = test_sync_config();
	rfm1.set_sync(&sc).unwrap();
	rfm2.set_sync(&sc).unwrap();
	for i in &[8_u8, 16, 32, 64, 65, 67, 128, 255] {
	
		msg.resize(*i as usize + 1, 0);
		rng.try_fill(msg.as_mut_slice()).unwrap();
		msg[0] = *i;
		let copy = Vec::from(&msg[1..]);
		let recv = spawn(move || {
			let recvd = rfm1.recv(Duration::from_secs(5)).unwrap();
			rfm1.set_mode(Mode::Standby).unwrap();
			assert_eq!(recvd,copy);
			rfm1
		});	
		sleep(Duration::from_secs(1));
		rfm2.send(&msg).unwrap();
		sleep(Duration::from_secs_f64((*i as f64 + 8.0 + 2.0 + 2.0) * 8.0 / rfm2.bitrate() as f64));
		rfm2.send(&msg).unwrap();
		rfm1 = recv.join().unwrap(); // check if other thread paniced
	}

}

#[test]
fn config() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
	let mut pc = PacketConfig::default();
	pc.set_len(255).set_variable(true);
	
	rfm.set_config(&pc).unwrap();
	assert_eq!(pc, rfm.config());

	let copy = rfm.config_dev().unwrap();
	assert_eq!(pc, copy);
}

#[test]
fn mode() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm = Rfm69::new(rst, en, spidev).unwrap();

	for mode in [Mode::Rx, Mode::Tx, Mode::Standby, Mode::Rx, Mode::Tx, Mode::Sleep].iter() {
		let mode = *mode;
		rfm.set_mode(mode).unwrap();
		assert_eq!(rfm.mode(), mode);
		assert_eq!(rfm.mode_dev().unwrap(), mode);
	}
}
