use gpio_cdev::Chip;
use ham::rfm69::SyncConfig;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::thread::sleep;
use std::time::{Duration, Instant};

fn main() {
    let mut sc = SyncConfig::default();
    let chip = Chip::new("/dev/gpiochip0").unwrap();
    let lines = chip.lines();
    for line in lines {
        println!("{:?}", line.info().unwrap());
    }
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(4096)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();

    spi.configure(&options).unwrap();
    let mut buf = [0; 2048];
    let mut xfer = SpidevTransfer::read(&mut buf);
    xfer.cs_change = 1;
    xfer.delay_usecs = std::u16::MAX;
    let mut buf2 = [1; 2048];
    let mut xfer2 = SpidevTransfer::read(&mut buf2);
    xfer2.cs_change = 0;
    xfer2.delay_usecs = std::u16::MAX;
    let mut xfers = [xfer, xfer2];

    println!("starting transfer");
    let start = Instant::now();
    spi.transfer_multiple(&mut xfers).unwrap();
    let elapsed = Instant::now().duration_since(start);
    println!(
        "transfers done: {:.2} B/s, {}",
        (4096 * 8) as f32 / elapsed.as_secs_f32(),
        elapsed.as_secs_f32()
    );
    sleep(Duration::from_secs(5));
}
