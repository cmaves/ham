use gpio_cdev::Chip;
use ham::rfm69::{DioMapping, Mode, PacketConfig, Rfm69, SyncConfig};
use ham::{IntoPacketReceiver, IntoPacketSender};
use ham::{PacketReceiver, PacketSender};
use rand::prelude::*;
use spidev::Spidev;
use std::cmp::Ordering;
use std::thread::{sleep, spawn};
use std::time::Duration;

const GPIOCHIP: &'static str = "/dev/gpiochip0";

const RFM1_RST: u32 = 24;
const RFM1_EN: u32 = 3;
const RFM1_SPI: &'static str = "/dev/spidev0.0";
const RFM1_G0: u32 = 17;
const RFM1_G1: u32 = 27;

const RFM2_RST: u32 = 2;
const RFM2_EN: u32 = 4;
const RFM2_SPI: &'static str = "/dev/spidev0.1";

fn test_sync_config() -> SyncConfig {
    let mut sc = SyncConfig::default();
    let mut syncword = [0; 8];
    for i in 0..8 {
        syncword[i] = i as u8 + 1;
    }
    sc.set_sync_word(&syncword);
    sc
}
fn get_some() -> Rfm69 {
    let mut chip = Chip::new(GPIOCHIP).unwrap();
    let rst = chip.get_line(RFM1_RST).unwrap();
    let en = chip.get_line(RFM1_EN).unwrap();
    let spidev = Spidev::open(RFM1_SPI).unwrap();
    let rfm1 = Rfm69::new(rst, en, spidev).unwrap();
    rfm1
}
fn get_pair() -> (Rfm69, Rfm69) {
    let rfm1 = get_some();
    let mut chip = Chip::new(GPIOCHIP).unwrap();
    let rst = chip.get_line(RFM2_RST).unwrap();
    let en = chip.get_line(RFM2_EN).unwrap();
    let spidev = Spidev::open(RFM2_SPI).unwrap();
    let rfm2 = Rfm69::new(rst, en, spidev).unwrap();
    (rfm1, rfm2)
}
#[test]
fn make_rfm69() {
    let rfm69 = get_some();
    for reg in rfm69.read_all().unwrap().iter().enumerate() {
        println!("Reg {:#04X}: {:#04X}", reg.0 + 1, reg.1);
    }
}
#[test]
fn test_bitrate() {
    let mut rfm69 = get_some();
    rfm69.set_bitrate(10_000).unwrap();
    assert_eq!(rfm69.bitrate(), 10_000);
    assert_eq!(rfm69.bitrate_dev().unwrap(), 10_000);
}
#[test]
fn test_set_dio_maps() {
    let mut rfm = get_some();
    let mut maps = [0_u8; 6];
    for (i, map) in maps.iter_mut().enumerate() {
        *map = i as u8 % 4;
    }
    let dm = rfm.dio_mapping().set_maps(&maps).set_clkout(0x5);
    rfm.set_dio_mapping(dm);
    assert_eq!(rfm.dio_mapping(), dm);
    assert_eq!(rfm.dio_mapping_dev().unwrap(), dm);
}
#[test]
fn sync_word() {
    let mut rfm69 = get_some();
    let sc = test_sync_config();
    rfm69.set_sync(&sc).unwrap();
    let ret = rfm69.sync_dev().unwrap();
    assert_eq!(sc, ret);
    assert_eq!(sc, rfm69.sync());
}
#[test]
fn send_recv_fixed() {
    let (mut rfm1, mut rfm2) = get_pair();

    let sc = test_sync_config();
    rfm1.set_sync(&sc).unwrap();
    rfm2.set_sync(&sc).unwrap();

    let mut msg = [0; 255];
    let mut cpy = [0; 255];
    let mut rng = thread_rng();
    let mut pc = PacketConfig::default();
    for i in &[8_usize, 16, 32, 64, 128, 255] {
        let i = *i;
        pc = pc.set_len(i as u8);
        rfm1.set_config(pc).unwrap();
        rfm2.set_config(pc).unwrap();
        rng.try_fill(&mut msg[..i]).unwrap();
        cpy[..i].copy_from_slice(&msg[..i]);
        let recv = spawn(move || {
            let mut recvd = [0; 255];
            let len = rfm1.recv(&mut recvd[..i], Duration::from_secs(5)).unwrap();
            rfm1.set_mode(Mode::Standby).unwrap();
            assert_eq!(len, i);
            assert_eq!(recvd[..i], cpy[..i]);
            rfm1
        });
        sleep(Duration::from_secs(1));
        rfm2.send(&msg[..i]).unwrap();
        sleep(Duration::from_secs_f64(
            (i as f64 + 8.0 + 2.0 + 2.0) * 8.0 / rfm2.bitrate() as f64,
        ));
        rfm2.send(&msg[..i]).unwrap();
        rfm1 = recv.join().unwrap(); // check if other thread paniced
    }
}
#[test]
fn send_recv_variable_without_dio() {
    let (rfm1, rfm2) = get_pair();
    send_recv_variable(rfm1, rfm2);
}

#[test]
fn send_recv_variable_with_dio() {
    let (mut rfm1, mut rfm2) = get_pair();
    //TODO add dio pins:
    let mut chip = Chip::new(GPIOCHIP).unwrap();
    let rfm1_dios = [
        Some(chip.get_line(RFM1_G0).unwrap()),
        Some(chip.get_line(RFM1_G1).unwrap()),
        None,
        None,
        None,
        None,
    ];
    let rfm2_dios = [None, None, None, None, None, None];

    rfm1.set_dios(&rfm1_dios);
    rfm2.set_dios(&rfm2_dios);
    let dm = DioMapping::default().set_map(1, 0x02);
    rfm1.set_dio_mapping(dm).unwrap();
    send_recv_variable(rfm1, rfm2);
}

fn send_recv_variable(mut rfm1: Rfm69, mut rfm2: Rfm69) {
    let pc = PacketConfig::default()
        .set_variable(true)
        .set_threshold(4)
        .set_len(255);

    let mut msg = [0; 256];
    let mut cpy = [0; 255];
    let mut rng = thread_rng();

    rfm1.set_config(pc).unwrap();
    rfm2.set_config(pc).unwrap();
    let sc = test_sync_config();
    rfm1.set_sync(&sc).unwrap();
    rfm2.set_sync(&sc).unwrap();
    for i in &[8_usize, 16, 32, 64, 65, 67, 128, 255] {
        let i = *i;
        rng.try_fill(&mut msg[..i + 1]).unwrap();
        msg[0] = i as u8;
        cpy[..i].copy_from_slice(&msg[1..i + 1]);
        let recv = spawn(move || {
            let mut recvd = [0; 255];
            let len = rfm1.recv(&mut recvd[..i], Duration::from_secs(5)).unwrap();
            rfm1.set_mode(Mode::Standby).unwrap();
            assert_eq!(len, i);
            assert_eq!(recvd[..i], cpy[..i]);
            rfm1
        });
        sleep(Duration::from_secs(1));
        rfm2.send(&msg[..i + 1]).unwrap();
        sleep(Duration::from_secs_f64(
            (i as f64 + 8.0 + 2.0 + 2.0) * 8.0 / rfm2.bitrate() as f64,
        ));
        rfm2.send(&msg[..i + 1]).unwrap();
        rfm1 = recv.join().unwrap(); // check if other thread paniced
    }
}

#[test]
fn config() {
    let mut rfm = get_some();
    let pc = PacketConfig::default();
    let pc = pc.set_len(255).set_variable(true);

    rfm.set_config(pc).unwrap();
    assert_eq!(rfm.config(), pc);
    assert_eq!(rfm.config_dev().unwrap(), pc);
}

#[test]
fn mode() {
    let mut rfm = get_some();

    for mode in [
        Mode::Rx,
        Mode::Tx,
        Mode::Standby,
        Mode::Rx,
        Mode::Tx,
        Mode::Sleep,
    ]
    .iter()
    {
        let mode = *mode;
        rfm.set_mode(mode).unwrap();
        assert_eq!(rfm.mode(), mode);
        rfm.validate_dev().unwrap();
    }
}

#[test]
fn power() {
    let mut rfm = get_some();
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
#[test]
fn packetreceiver_sender() {
    let (rfm1, rfm2) = get_pair();
    let bitrate = rfm1.bitrate();
    let mut receiver = rfm1.into_packet_receiver().unwrap();
    let mut sender = rfm2.into_packet_sender(2).unwrap();
    sender.set_verbose(true).unwrap();

    let mut msg = [0; 234];
    let mut cpy = [0; 234];
    let mut rng = thread_rng();
    for i in &[0_usize, 1, 8, 16, 32, 64, 128, 233] {
        let i = *i;
        rng.try_fill(&mut msg[..i]).unwrap();
        cpy[..i].copy_from_slice(&msg[..i]);
        let time = i as u32 * 1_000_000;
        receiver.set_verbose(true).unwrap();
        receiver.start().unwrap();
        let recv = spawn(move || {
            eprintln!("Waiting up to 5 seconds for reception....");
            match receiver.recv_pkt_to(Duration::from_secs(5)) {
                Ok(recvd) => {
                    assert_eq!(recvd[..i], cpy[..i]);
                    let cur_time = receiver.cur_time();
                    assert!(cur_time.wrapping_sub(time) <= 5_000_000);
                    receiver
                        .terminate()
                        .ok()
                        .unwrap()
                        .into_packet_receiver()
                        .unwrap()
                }
                Err(e) => panic!(
                    "Error {:?}, getting error from thread: {:?}",
                    e,
                    receiver.terminate().map_err(|(e, _)| e).err().unwrap()
                ),
            }
        });
        sleep(Duration::from_secs(1));
        let mut send = || {
            sender.send_packet(&msg[..i], time)?;
            sender.send_packet(&msg[..i], time)
        };
        if let Err(_) = send() {
            panic!(
                "Receive thread disconnected, getting error: {:?}",
                sender.terminate().map_err(|(e, _)| e).err().unwrap()
            );
        }
        receiver = match recv.join() {
            // check if other thread panicked
            Ok(v) => v,
            Err(_) => {
                // the other thread will panic if sending thread didnt succeed. It is more useful to get this error message
                if let Err(_) = sender.alive() {
                    panic!(
                        "Receive thread disconnected, getting error: {:02?}",
                        sender.terminate().map_err(|(e, _)| e).err().unwrap()
                    );
                }
                panic!("Test receiving thread panicked!")
            }
        }
    }
}
#[test]
fn try_iter() {
    let (rfm1, mut rfm2) = get_pair();
    let bitrate = rfm1.bitrate();
    // rfm2.set_preamble_len(100).unwrap();
    let mut receiver = rfm1.into_packet_receiver().unwrap();
    receiver.set_verbose(true).unwrap();
    let mut sender = rfm2.into_packet_sender(8).unwrap();
    let mut msgs = [[0_u8; 8]; 8];
    let mut rng = thread_rng();
    receiver.start().unwrap();
    sleep(Duration::from_millis(200)); // let settle
    for msg in msgs.iter_mut() {
        rng.try_fill(msg).unwrap();
        sender.send_packet(msg, 0).unwrap();
    }
    sleep(Duration::from_secs_f64(
        (msgs[0].len() as f64 + 8.0 + 2.0 + 2.0 + 21.0) * 8.0 * msgs.len() as f64 / bitrate as f64
            + 0.1,
    ));
    let r_iter = receiver.try_iter();
    if !r_iter.eq(msgs.iter()) {
        let rfm1 = receiver.terminate().ok().unwrap();
        let mut receiver = rfm1.into_packet_receiver().unwrap();
        receiver.set_verbose(true).unwrap();
        receiver.start().unwrap();
        sleep(Duration::from_millis(200)); // let settle
        for msg in msgs.iter() {
            sender.send_packet(msg, 0).unwrap();
        }
        sleep(Duration::from_secs_f64(
            (msgs[0].len() as f64 + 8.0 + 2.0 + 2.0 + 21.0) * 8.0 * msgs.len() as f64
                / bitrate as f64
                + 0.1,
        ));
        let r_iter = receiver.try_iter();
        assert!(r_iter.eq(msgs.iter()));
    }
}
