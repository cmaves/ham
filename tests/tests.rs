use gpio_cdev::Chip;
use ham::rfm69::{PacketConfig,Rfm69,SyncConfig,Mode};
use spidev::Spidev;
use std::thread::{spawn,sleep};
use std::time::Duration;
use rand::prelude::*;

fn test_sync_config() -> SyncConfig {
	let mut sc = SyncConfig::default();
	let mut syncword = [0; 8];
	for i in 0..8 {
		syncword[i] = i as u8 + 1;	
	}
	sc.set_sync_word(&syncword);
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
	let mut rfm69 = Rfm69::new(rst, en, spidev).unwrap();
	let sc = test_sync_config();
	rfm69.set_sync(&sc).unwrap();
	let ret = rfm69.sync_dev().unwrap();
	assert_eq!(sc, ret);
	assert_eq!(sc, rfm69.sync());
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

	let mut msg = [0; 255];
	let mut rng = thread_rng();

	for i in &[8_usize, 16, 32, 64, 128, 255] {
		let i = *i;
		pc.set_len(i as u8);
		rfm1.set_config(&pc).unwrap();
		rfm2.set_config(&pc).unwrap();
		rng.try_fill(&mut msg[..i]).unwrap();
		let mut cpy = [0; 255];
		cpy[..i].copy_from_slice(&msg[..i]);
		let recv = spawn(move || {
			let mut recvd = [0; 255];
			let len = rfm1.recv(&mut recvd[..i], Duration::from_secs(5)).unwrap();
			rfm1.set_mode(Mode::Standby).unwrap();
			assert_eq!(len,i);
			assert_eq!(recvd[..i],cpy[..i]);
			rfm1
		});	
		sleep(Duration::from_secs(1));
		rfm2.send(&msg[..i]).unwrap();
		sleep(Duration::from_secs_f64((i as f64 + 8.0 + 2.0 + 2.0) * 8.0 / rfm2.bitrate() as f64));
		rfm2.send(&msg[..i]).unwrap();
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

	let mut msg = [0; 256];
	let mut rng = thread_rng();

	rfm1.set_config(&pc).unwrap();
	rfm2.set_config(&pc).unwrap();
	let sc = test_sync_config();
	rfm1.set_sync(&sc).unwrap();
	rfm2.set_sync(&sc).unwrap();
	for i in &[8_usize, 16, 32, 64, 65, 67, 128, 255] {
		let i = *i;
		rng.try_fill(&mut msg[..i+1]).unwrap();
		msg[0] = i as u8;
		let mut cpy = [0; 255];
		cpy[..i].copy_from_slice(&msg[1..i+1]);
		let recv = spawn(move || {
			let mut recvd = [0; 255];
			let len = rfm1.recv(&mut recvd[..i], Duration::from_secs(5)).unwrap();
			rfm1.set_mode(Mode::Standby).unwrap();
			eprintln!("Rssi: {}", rfm1.rssi().unwrap());
			assert_eq!(len,i);
			assert_eq!(recvd[..i],cpy[..i]);
			rfm1
		});	
		sleep(Duration::from_secs(1));
		rfm2.send(&msg[..i+1]).unwrap();
		sleep(Duration::from_secs_f64((i as f64 + 8.0 + 2.0 + 2.0) * 8.0 / rfm2.bitrate() as f64));
		rfm2.send(&msg[..i+1]).unwrap();
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
		rfm.validate_dev().unwrap();
	}
}

#[test]
fn power() {
	let mut chip = Chip::new("/dev/gpiochip0").unwrap();
	let rst = chip.get_line(24).unwrap();
	let en = chip.get_line(3).unwrap();
	let spidev = Spidev::open("/dev/spidev0.0").unwrap();
	let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
	rfm.set_mode(Mode::Tx).unwrap();
	assert_eq!(rfm.power().unwrap(), 13);
	rfm.validate_dev().unwrap();
	rfm.set_power(-18).unwrap();
	assert_eq!(rfm.power().unwrap(), -18);
	rfm.validate_dev().unwrap();
	rfm.set_power(2).unwrap();
	assert_eq!(rfm.power().unwrap(), 2);
	rfm.validate_dev().unwrap();
	rfm.set_power(17).unwrap();
	assert_eq!(rfm.power().unwrap(), 17);
	rfm.validate_dev().unwrap();
	rfm.set_power(20).unwrap();
	assert_eq!(rfm.power().unwrap(), 20);
	rfm.validate_dev().unwrap();
	rfm.set_power(13).unwrap();
	assert_eq!(rfm.power().unwrap(), 13);
	rfm.validate_dev().unwrap();
}

