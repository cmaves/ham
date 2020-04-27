use crate::{bit, set_bit, set_bit_to, sleep, unset_bit, Error};

use gpio_cdev::{EventRequestFlags, Line, LineEventHandle, LineHandle, LineRequestFlags};
use nix::poll;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use reed_solomon::Decoder;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::convert::TryInto;
use std::fmt;
use std::ops::Index;
use std::os::unix::io::AsRawFd;
use std::time::{Duration, Instant};

mod receiver;
pub use crate::rfm69::receiver::*;

mod sender;
pub use crate::rfm69::sender::*;

pub struct Rfm69 {
    rst: LineHandle,
    en: LineHandle,
    dios: [Option<Line>; 6],
    spi: Spidev,
    verbose: bool,
    pc: PacketConfig,
    sc: SyncConfig,
    mode: Mode,
    dio_map: DioMapping,
    fifo_thresh: u8,
    bitrate: u32,
    preamble: u16,
}
struct IrqWait<'a> {
    line: Option<LineEventHandle>,
    irq: Irq,
    rf: &'a Rfm69,
    high: bool,
    check: Duration, // holds the duration in which the interrupt should be polled
}
impl IrqWait<'_> {
    fn check_flag(rfm: &Rfm69, irq: Irq) -> Result<bool, std::io::Error> {
        let reg = Register::from_u8(Register::IrqFlags1 as u8 + (irq as u8) / 8).unwrap();
        let b = irq as u8 % 8;
        let flags = rfm.read(reg)?;
        Ok(bit(flags, b))
    }
    fn wait(&self, timeout: Duration) -> Result<(), Error> {
        if let Some(leh) = &self.line {
            let fd = leh.as_raw_fd();
            let pf = poll::PollFd::new(fd, poll::PollFlags::POLLIN);
            let mut c_timeout = timeout.as_millis();
            if timeout.as_nanos() % 1_000_000 > 0 {
                c_timeout += 1
            } // round up time
            match poll::poll(&mut [pf], c_timeout.try_into().unwrap()) {
                Ok(i) => {
                    if i > 0 {
                        // an interrupt event occurred
                        Ok(())
                    } else {
                        // no interrupt occured
                        Err(Error::Timeout("Interrupt poll timeout!".to_string()))
                    }
                }
                Err(e) => Err(Error::Timeout(format!("Interrupt poll error ({:?})!", e))), // a poll error
            }
        } else {
            let reg = Register::from_u8(Register::IrqFlags1 as u8 + (self.irq as u8) / 8).unwrap();
            let b = self.irq as u8 % 8;
            let target = Instant::now() + timeout - self.check; //
            loop {
                //
                let flags = self.rf.read(reg)?;
                if bit(flags, b) == self.high {
                    return Ok(());
                }
                if Instant::now() > target {
                    break;
                }
                sleep(self.check);
            }
            #[cfg(test)]
            {
                let reg = self.rf.read_all()?;
                for i in reg.iter().enumerate() {
                    let reg = Register::from_usize(i.0 + 1).unwrap();
                    eprintln!("{:>15?}: {:#04x}", reg, i.1);
                }
            }
            Err(Error::Timeout(format!(
                "Interrupt ({:?}) timeout!",
                self.irq
            )))
        }
    }
    fn check(&self) -> Result<bool, Error> {
        if let Some(leh) = &self.line {
            Ok((leh.get_value()? != 0) == self.high)
        } else {
            let reg = Register::from_u8(Register::IrqFlags1 as u8 + (self.irq as u8) / 8).unwrap();
            let flags = self.rf.read(reg)?;
            Ok(bit(flags, self.irq as u8 % 8) == self.high)
        }
    }

    fn check_and_wait(&self, timeout: Duration) -> Result<(), Error> {
        if !self.check()? {
            self.wait(timeout)?;
        }
        Ok(())
    }
    fn is_true_irq(&self) -> bool {
        self.line.is_some()
    }
}

const FXOSC: u32 = 32_000_000;
const FSTEP: f64 = (FXOSC as f64) / 524_288.0; // FOSC/2^19
const MIN_FREQ: u32 = 290_000_000;
const MAX_FREQ: u32 = 1020_000_000;

impl Rfm69 {
    /// Creates a new instance of the device to control the RFM69HCW chip.
    ///
    /// `rst` represents the reset pin on the chip.
    /// `en` should be enable pin on the chip.
    /// `spi` uses the all of the SPI pins, including the corresponding CS/CE pins.
    ///  All SPI commands performed by `Rfm69` are atomic allowing other SPI devices to be used alongside with the RFM69 chip.
    ///  By default, the DIO pins are not used by this driver.
    ///  This function configures some the RFM69 registers to recommened values and does some basic validation to ensure the device appears to be wired correctly.
    ///
    ///  # Example
    ///
    /// ```
    /// // On a Raspberry Pi 3
    /// use gpio_cdev::Chip;
    /// use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// let rst = chip.get_line(24).unwrap();
    /// let en = chip.get_line(3).unwrap();
    /// let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(0x24, rfm.version().unwrap());
    /// ```
    pub fn new(rst: Line, en: Line, mut spi: Spidev) -> Result<Self, Error> {
        let flags = LineRequestFlags::OUTPUT;
        let rst = rst.request(flags, 1, "rfm69_reset")?;
        sleep(Duration::from_millis(1)); // let settle
        rst.set_value(0)?;
        sleep(Duration::from_millis(5)); // let settle

        let en = en.request(flags, 1, "rfm69_enable")?;

        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(5_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build();

        spi.configure(&options)?;

        let dios = [None, None, None, None, None, None];
        let mut rfm = Rfm69 {
            spi,
            rst,
            en,
            dios,
            verbose: false,
            bitrate: 4800,
            preamble: 0x03,
            pc: PacketConfig::default(),
            mode: Mode::Standby,
            dio_map: DioMapping::default(),
            fifo_thresh: 15,
            sc: SyncConfig::default(),
        };
        (|| {
            rfm.validate_version()?;
            rfm.configure_defaults()?;
            rfm.get_mode_ready()?
                .check_and_wait(Duration::from_millis(10))?;
            rfm.validate_dev()
        })()
        .map_err(|e| Error::Init(format!("Device failed to init!: {:?}", e)))?;
        Ok(rfm)
    }
    /// Sets the [`Mode`] of the Rfm69 chip and waits until device signals is ready.
    ///
    /// If the `Rfm69`'s mode is already in the given `mode` then this function does nothing.
    /// After setting the mode on the device this method will wait up to 10 ms, before timing out.
    /// The device starts in `Standby` mode.
    ///
    /// This function changes [`OpMode`]  on the RFM69 chip.
    ///
    /// [`Mode`]: ./enum.Mode.html
    /// [`OpMode`]: ./enum.Register.html
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
        self.set_mode_internal(mode)?;
        // wait for mode to be set
        let iw = self.get_mode_ready()?;
        iw.check_and_wait(Duration::from_millis(10))
    }
    /// Returns the [`Mode`] of the device stored by the controller.
    ///
    /// The value returned is the `Mode` stored by the controller.
    /// This could differ from the actual device's mode if an error has occurred or if the [`write()`]/[`write_many()`] methods are used directly are used directly.
    /// [`mode_dev()`] will get the `Mode` from the device, instead of the stored mode.
    ///
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,Mode};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.mode(), Mode::Standby);
    /// ```
    /// [`Mode`]: ./enum.Mode.html
    /// [`mode_dev()`]: ./struct.Rfm69.html#method.mode_dev
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    #[inline]
    pub fn mode(&self) -> Mode {
        self.mode
    }
    /// Returns the [`Mode`] of the actual RFM69 device.
    ///
    /// The value returned is the `Mode` stored on the actual device.
    /// This is in contrast to [`mode()`] which gets the stored `Mode` from the controller.
    ///
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,Mode};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.mode_dev().unwrap(), Mode::Standby);
    /// ```
    /// [`Mode`]: ./enum.Mode.html
    /// [`mode()`]: ./struct.Rfm69.html#method.mode
    #[inline]
    pub fn mode_dev(&self) -> Result<Mode, std::io::Error> {
        let mode = self.read(Register::OpMode)? & 0x7F;
        Ok(Mode::from_u8(mode).unwrap())
    }
    /// Sets the bitrate of the RFM69 device.
    ///
    /// The RFM69 device will have better range at a lower.
    /// This function sets the [`BitrateMsb/Lsb`] registers.
    /// This function returns `Err` if `bitrate` is out of range or on IO errors[`write()`]/[`write_many()`]
    ///
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// rfm.set_bitrate(50_000).unwrap(); // 50 kbps
    /// assert_eq!(rfm.bitrate(), 50_000);
    /// assert_eq!(rfm.bitrate_dev().unwrap(), 50_000);
    /// ```
    /// [`BitrateMsb/Lsb`]: ./enum.Register.html
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    pub fn set_bitrate(&mut self, bitrate: u32) -> Result<(), Error> {
        if bitrate > 300000 {
            Err(Error::BadInputs("Bitrate out of bounds!".to_string()))
        } else if let Ok(v) = ((FXOSC as f64 / bitrate as f64).round() as u64).try_into() {
            let v: u16 = v;
            self.write_many(Register::BitrateMsb, &v.to_be_bytes())?;
            self.bitrate = bitrate;
            Ok(())
        } else {
            Err(Error::BadInputs("Bitrate out of bounds!".to_string()))
        }
    }
    /// Returns the bitrate of the device stored by the controller in bits per second.
    ///
    /// The difference between this function and [`bitrate_dev()`] is this function returns a stored bitrate, while `bitrate_dev()` reads from the RFM69 chip and computes the bitrate.
    /// This function is faster but may differ from the actual device bitrate if [`write()`]/[`write_many()`] are used.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.bitrate(), 4800); // 4.8 kpbs
    /// ```
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    /// [`bitrate_dev()`]:
    #[inline]
    pub fn bitrate(&self) -> u32 {
        self.bitrate
    }
    /// Returns the bitrate from the RFM69 chip in bits per second.
    ///
    /// The difference between this function and [`bitrate()`] is this function returns the bitrate from the actual device, while `bitrate()` read a value stored by the controller.
    /// Reads from the [`BitrateMsb/Lsb`] registers.
    /// This function returns `Err` if there is an SPI IO error.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.bitrate_dev().unwrap(), 4800); // 4.8 kpbs
    /// ```
    /// [`BitrateMsb/Lsb`]: ./enum.Register.html
    pub fn bitrate_dev(&self) -> Result<u32, Error> {
        let mut v = [0, 0];
        self.read_many(Register::BitrateMsb, &mut v)?;
        let v = u16::from_be_bytes(v);
        let v = (FXOSC as f64 / v as f64).round() as u32;
        Ok(v)
    }
    pub fn set_frequency(&self, frequency: u32) -> Result<(), Error> {
        if frequency < MIN_FREQ {
            Err(Error::BadInputs(format!("Frequency must be at least {} Mhz! Depending on the specific radio module the actually minimum frequency may be higher.", MIN_FREQ / 1_000_000)))
        } else if frequency > MAX_FREQ {
            Err(Error::BadInputs(format!("Frequency must be less than or equal to {} Mhz! Depending on the specific radio module the actually maximum frequency may be lower.", MAX_FREQ / 1_000_000)))
        } else {
            let v = ((frequency as f64) / FSTEP).round() as u32;
            let v = v.to_be_bytes();
            self.write_many(Register::FrfMsb, &v[1..4])?;
            Ok(())
        }
    }
    pub fn frequency(&self) -> Result<u32, Error> {
        let mut v = [0; 4];
        self.read_many(Register::FrfMsb, &mut v[1..4])?;
        let v = u32::from_be_bytes(v);
        let ret = (v as f64 * FSTEP).round() as u32;
        Ok(ret)
    }

    /// Sets the packet configuration of the RFM69 chip.
    ///
    /// `config` represents a variety of different options to configure the RFM69 chip.
    /// Before writing the configuration to the device, the configuration is validated with [`PacketConfig::validate()`] to ensure make sure there are no conflicting options set.
    /// By default these options are set to the [`Default`] implementation for `PacketConfig` values when the RFM69 controller is created.
    /// These options are detailed more on the [`PacketConfig`] page.
    /// This function sets the `PacketConfig1`, `PacketConfig2`, `PayloadLength`, `NodeAddrs`, `BroadcastAddrs`, and `FifoThresh` [`registers`].
    /// This function returns `Err` if the configuration fails to validate or there is an SPI IO error.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,PacketConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// let mut pc = PacketConfig::default();
    /// pc = pc.set_variable(true); // set the RFM69 chip the receive packets in variable format
    /// pc = pc.set_len(128); // set the maximum length of packets to be received.
    /// rfm.set_config(pc).unwrap();
    /// assert_eq!(rfm.config(), pc);
    /// assert_eq!(rfm.config_dev().unwrap(), pc);
    /// ```
    /// [`PacketConfig`]: ./struct.PacketConfig.html
    /// [`Default`]: ./struct.PacketConfig.html#impl-Default
    /// [`PacketConfig::validate()`]: ./struct.PacketConfig.html#method.validate
    /// [`registers`]: ./enum.Register.html
    pub fn set_config(&mut self, config: PacketConfig) -> Result<(), Error> {
        // validate config./struct.Rfm69.html#method.config
        config.validate()?;
        self.write_many(Register::PacketConfig1, &config.0)?;
        self.write_many(Register::FifoThresh, &config.1)?;
        self.pc = config;
        Ok(())
    }
    /// Returns the packet configuration from the controller.
    ///
    /// The difference between this function and [`config_dev()`] is this function returns the configuration stored by the controller, while the latter reads the configuration from the RFM69 chip.
    /// This function is faster but may differ from the device configuration if [`write()`]/[`write_many()`] are used.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,PacketConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.config(), PacketConfig::default());
    /// ```
    ///
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    /// [`config_dev()`]: ./struct.Rfm69.html#method.config_dev
    pub fn config(&self) -> PacketConfig {
        self.pc
    }
    /// Reads and returns the packet configuration from the RFM69 chip.
    ///
    /// The difference between this function and [`config()`] is that this function reads the configuration for the actual device, while the latter returns the configuration stored by the controller.
    /// This function reads the `PacketConfig1`, `PacketConfig2`, `PayloadLength`, `NodeAddrs`, `BroadcastAddrs`, and `FifoThresh` [`registers`].
    /// This function returns `Err` if there is an SPI IO error.
    ///
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,PacketConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.config_dev().unwrap(), PacketConfig::default());
    /// ```
    /// [`registers`]: ./enum.Register.html
    /// [`config()`]: ./struct.Rfm69.html#method.config
    pub fn config_dev(&self) -> Result<PacketConfig, Error> {
        let mut packet1 = [0; 4];
        self.read_many(Register::PacketConfig1, &mut packet1)?;

        let mut packet2 = [0, 0];
        self.read_many(Register::FifoThresh, &mut packet2)?;
        let pc = PacketConfig(packet1, packet2);
        Ok(pc)
    }
    /// Sets the sync word configuration of the RFM69 chip.
    ///
    /// `config` represents the Sync word ocnfiguration used to tell which packets should be received by the Rfm69 chip.
    /// By default, the sync word configuration is set to the values specified by the [`Default`] implementation for `SyncConfig` when the controller is created.
    /// More information on sync word configuration options can be found on [`SyncConfig`] page.
    /// This function sets the `SyncConfig`, and `SyncValue1-9` [`registers`]
    /// This function returns `Err` if there is an SPI IO error.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,SyncConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// let sc = *SyncConfig::default().set_sync_word(&[0x56, 0xa9, 0x0b, 0x9a]).set_len(4); // this is the configuration used by the PacketReceiver/Sender impls
    /// rfm.set_sync(sc).unwrap();
    /// assert_eq!(rfm.sync(), sc);
    /// assert_eq!(rfm.sync_dev().unwrap(), sc);
    /// ```
    ///
    /// [`SyncConfig`]: ./struct.SyncConfig.html
    /// [`Default`]: ./struct.SyncConfig.html#impl-Default
    /// [`registers`]: ./enum.Register.html
    pub fn set_sync<T: AsRef<SyncConfig>>(&mut self, config: T) -> Result<(), Error> {
        let config = config.as_ref();
        self.write_many(Register::SyncConfig, &config.0)?;
        self.sc = *config;
        Ok(())
    }
    /// Returns the sync word configuration from the controller.
    ///
    /// The difference between this function and [`sync_dev()`] is this function returns the configuration stored by the controller, while the latter reads the configuration from the RFM69 chip.
    /// This function is faster but may differ from the actual device bitrate if [`write()`]/[`write_many()`] are used.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,SyncConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.sync(), SyncConfig::default());
    /// ```
    ///
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    /// [`sync_dev()`]:  ./struct.Rfm69.html#method.sync_dev
    #[inline]
    pub fn sync(&self) -> SyncConfig {
        self.sc
    }
    /// Reads and returns the sync word configuration from the RFM69 chip.
    ///
    /// The difference between this function and [`sync()`] is this function reads the sync word configuration from the actual device, while the latter returns the configuration stored by the controller.
    /// This function reads the `SyncConfig`, and `SyncValue1-9` [`registers`]
    /// This function returns `Err` if there is an SPI IO error.
    ///
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,SyncConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.sync_dev().unwrap(), SyncConfig::default());
    /// ```
    /// [`registers`]: ./enum.Register.html
    /// [`sync()`]: ./struct.Rfm69.html#method.sync
    pub fn sync_dev(&self) -> Result<SyncConfig, std::io::Error> {
        let mut buf = [0; 9];
        self.read_many(Register::SyncConfig, &mut buf)?;
        Ok(SyncConfig(buf))
    }
    /// Sets the AES key to be used when sending or receiving packets.
    ///
    /// This method only sets the AES key; it does not enable AES encryption/decryption.
    /// It can be enabled with [`set_config()`].
    /// This function set the [`AesKey1-16`] registers.
    /// This function returns `Err` if there is an SPI IO error.
    /// [`set_config()`]:
    /// [`AesKey1-16`]:
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// ```
    /// [`set_config()`]: ./struct.Rfm69.html#method.set_config
    /// [`AesKey1-16`]: ./enum.Register.html
    pub fn set_aes(&self, key: &[u8; 16]) -> Result<(), std::io::Error> {
        self.write_many(Register::AesKey1, key)
    }
    /// Sets the preamble length in bytes of the packets sent or received by the RFM69 chip
    ///
    /// This function sets the [`PreambleMsb/Lsb`] on the device.
    /// This function returns `Err` if there is an SPI IO error.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// rfm.set_preamble_len(1024).unwrap();
    /// assert_eq!(rfm.preamble_len(), 1024);
    /// assert_eq!(rfm.preamble_len_dev().unwrap(), 1024);
    /// ```
    /// [`PreambleMsb/Lsb`]: ./enum.Register.html
    pub fn set_preamble_len(&mut self, len: u16) -> Result<(), Error> {
        self.write_many(Register::PreambleMsb, &len.to_be_bytes())?;
        self.preamble = len;
        Ok(())
    }
    /// Returns the preamble length in bytes of packets.
    ///
    /// The difference between this function and [`preamble_len_dev()`] is this function returns the preamble length stored by the controller, while the latter reads the actual preamble length from the device.
    /// This function is faster but may differ from the actual device if [`write()`]/[`write_many()`] are used.
    ///
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.preamble_len(), 3);
    /// ```
    /// [`preamble_len_dev()`]: ./struct.Rfm69.html#method.preamble_len_dev
    /// [`write()`]: ./struct.Rfm69.html#method.write
    /// [`write_many()`]:  ./struct.Rfm69.html#method.write_many
    #[inline]
    pub fn preamble_len(&self) -> u16 {
        self.preamble
    }
    /// Returns the preamble length in bytes of packet.
    ///
    /// The difference between this function and [`preamble_len()`], is this function returns the preamble length from the actual device, while the latter returns the preamble length from a stored value by the controller.
    /// This function returns `Err` if there is an SPI IO error.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.preamble_len_dev().unwrap(), 3);
    /// ```
    /// [`preamble_len()`]: ./struct.Rfm69.html#method.preamble_len
    pub fn preamble_len_dev(&self) -> Result<u16, Error> {
        let mut len = [0; 2];
        self.read_many(Register::PreambleMsb, &mut len)?;
        Ok(u16::from_be_bytes(len))
    }
    /// Adds or removes Gpio lines to correspond to the different DIO pins.
    ///
    /// The RFM69 chip has 6 DIO pins that provide various configurable interrupts.
    /// If `dios` is shorter than 6 elements, then only the first lines are overwritten while the remaining remain intact.
    /// If `dios` is longer than 6 elements, then the remainging elements after the first 6 are ignored.
    /// This controller can operate without using any DIO pins by repeatingly querying the device over SPI for the status of certain interrupts.
    /// Adding them may improve CPU usage by eliminating busy waiting for certain operations.
    ///
    /// # Example
    /// ```
    /// use gpio_cdev::Chip;
    /// use spidev::Spidev;
    /// use ham::rfm69::{Rfm69};
    ///
    /// let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// let rst = chip.get_line(24).unwrap();
    /// let en = chip.get_line(3).unwrap();
    /// let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// let d0 = chip.get_line(17).unwrap();
    /// let d1 = chip.get_line(27).unwrap();
    /// rfm.set_dios(&[Some(d0), Some(d1)]);
    /// ```
    #[inline]
    pub fn set_dios(&mut self, dios: &[Option<Line>]) {
        for (dst, src) in self.dios.iter_mut().zip(dios.iter()) {
            *dst = src.clone();
        }
    }
    /// Adds or removes GPIO lines that correspond to the different DIO pins.
    ///
    /// The RFM69 chip has 6 DIO pins that provide various configurable interrupts.
    /// This controller can operate without using any DIO pins by repeatingly querying the device over SPI for the status of certain interrupts.
    /// Adding them may improve CPU usage by eliminating busy waiting for certain operations.
    ///
    /// # Example
    /// ```
    /// use gpio_cdev::Chip;
    /// use spidev::Spidev;
    /// use ham::rfm69::{Rfm69};
    ///
    /// let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// let rst = chip.get_line(24).unwrap();
    /// let en = chip.get_line(3).unwrap();
    /// let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// let d0 = chip.get_line(17).unwrap();
    /// rfm.set_dio(0, Some(d0)); // set the DIO pin zero
    /// ```
    /// # Panics
    /// This function will panic if `index` is greater than or equal to 6.
    #[inline]
    pub fn set_dio(&mut self, index: u8, dio: Option<Line>) {
        self.dios[index as usize] = dio.clone();
    }
    #[inline]
    pub fn dios(&self) -> &[Option<Line>; 6] {
        &self.dios
    }
    #[inline]
    pub fn set_dio_mapping(&mut self, dio_map: DioMapping) -> Result<(), Error> {
        let bytes: [u8; 2] = dio_map.into();
        self.write_many(Register::DioMapping1, &bytes)?;
        self.dio_map = dio_map;
        Ok(())
    }
    #[inline]
    pub fn dio_mapping(&self) -> DioMapping {
        self.dio_map
    }
    #[inline]
    pub fn dio_mapping_dev(&self) -> Result<DioMapping, Error> {
        let mut bytes = [0; 2];
        self.read_many(Register::DioMapping1, &mut bytes)?;
        Ok(bytes.into())
    }
    /// Configures the RFM69 chip its recommended default settings.
    ///
    /// The method is called by new() during initialization.
    /// This method the [`PacketConfig`], [`SyncConfig`], and [`DioMapping`] to their [`Default`] values.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,PacketConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap().reset().unwrap();
    /// assert_eq!(rfm.config_dev().unwrap(), PacketConfig::rst_value());
    /// rfm.configure_defaults().unwrap();
    /// assert_eq!(rfm.config_dev().unwrap(), PacketConfig::default());
    /// ```

    /// [`PacketConfig`]: ./struct.PacketConfig.html
    /// [`SyncConfig`]: ./struct.SyncConfig.html
    /// [`DioMapping`]: ./struct.DioMapping.html
    /// [`Default`]: https://doc.rust-lang.org/nightly/core/default/trait.Default.html
    pub fn configure_defaults(&mut self) -> Result<(), Error> {
        self.set_config(PacketConfig::default())?;
        self.set_sync(&SyncConfig::default())?;
        self.set_dio_mapping(DioMapping::default())
    }
    /// Resets the RFM69 chip using the RST pin, and then performs simple validations on the chip.
    ///
    /// Resetting the RFM69 chip clears every register on the device, and resets them to their built-in reset values.
    /// It is important to note that these values differ from the default values set by [`Rfm69::new()`] .
    /// When `new()` is called it configures the Rfm69 chip to recommended defaults provided by the `default` methods for the different configuration structs.
    /// This method is the easiest way to restore the built-in reset values.
    /// This function will return `Err` if the device version fails to validate after the reset or if the `Mode` is not reset to `Standby`.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::{Rfm69,PacketConfig};
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.config_dev().unwrap(), PacketConfig::default()); // config is set to recommended values
    /// let rfm = rfm.reset().unwrap(); // Reset the chip
    /// assert_eq!(rfm.config_dev().unwrap(), PacketConfig::rst_value()); // config is now set to reset values
    /// ```
    /// [`Rfm69::new()`]: ./struct.Rfm69.html#method.new
    pub fn reset(mut self) -> Result<Self, Error> {
        self.rst.set_value(1)?;
        sleep(Duration::from_millis(1));
        self.rst.set_value(0)?;
        sleep(Duration::from_millis(5));
        self.mode = Mode::Standby;
        self.bitrate = 4800;
        self.fifo_thresh = 15;
        self.pc = PacketConfig::rst_value();
        self.validate_version()?;
        self.get_mode_ready()?
            .check_and_wait(Duration::from_millis(10))?;
        Ok(self)
    }
    pub fn validate_dev(&self) -> Result<(), Error> {
        self.validate_version()?;
        self.power()?;
        let mode_dev = self.mode_dev()?;
        if self.mode != mode_dev {
            let err_str = format!(
                "Stored mode doesn't match device!: {:?} (dev) != {:?}",
                mode_dev, self.mode
            );
            return Err(Error::ChipMalfunction(err_str));
        }
        if self.config() != self.config_dev()? {
            return Err(Error::ChipMalfunction(
                "Stored packet config doesn't match device.".to_string(),
            ));
        }
        if self.sync() != self.sync_dev()? {
            return Err(Error::ChipMalfunction(
                "Stored sync config doesn't match device.".to_string(),
            ));
        }
        if self.bitrate() != self.bitrate_dev()? {
            return Err(Error::ChipMalfunction(
                "Stored bitrate doesn't match device.".to_string(),
            ));
        }
        if self.preamble_len() != self.preamble_len_dev()? {
            return Err(Error::ChipMalfunction(
                "Stored premable length doesn't match device.".to_string(),
            ));
        }
        Ok(())
    }
    pub fn read(&self, reg: Register) -> Result<u8, std::io::Error> {
        let mut buf = [0];
        self.read_many(reg, &mut buf)?;
        Ok(buf[0])
    }
    pub fn read_many(&self, reg: Register, buf: &mut [u8]) -> Result<(), std::io::Error> {
        let reg = [(reg as u8) & 0x7F];
        let addr_xfer = SpidevTransfer::write(&reg);
        let read_xfer = SpidevTransfer::read(buf);
        let mut xfers = [addr_xfer, read_xfer];
        self.spi.transfer_multiple(&mut xfers)
    }
    fn read_many_count(&self, reg: Register, buf: &mut [u8], count: u8) -> Result<(), Error> {
        let count = count as usize;
        let reg = [(reg as u8) & 0x7F];
        let mut local = [0; 255];
        let i_buf = &mut local[..count.saturating_sub(buf.len())];

        let addr_xfer = SpidevTransfer::write(&reg);
        let buf = if count >= buf.len() {
            buf
        } else {
            &mut buf[..count]
        };
        let read_xfer = SpidevTransfer::read(buf);
        let buf_xfer = SpidevTransfer::read(i_buf);
        let mut xfers = [addr_xfer, read_xfer, buf_xfer];
        self.spi.transfer_multiple(&mut xfers)?;
        if i_buf.len() != 0 {
            Err(Error::BufferOverflow(count))
        } else {
            Ok(())
        }
    }
    pub fn write(&self, reg: Register, val: u8) -> Result<(), std::io::Error> {
        let buf = [val];
        self.write_many(reg, &buf)
    }
    pub fn write_many(&self, reg: Register, buf: &[u8]) -> Result<(), std::io::Error> {
        let reg = [(reg as u8) | 0x80];
        let addr_xfer = SpidevTransfer::write(&reg);
        let write_xfer = SpidevTransfer::write(buf);
        let mut xfers = [addr_xfer, write_xfer];
        self.spi.transfer_multiple(&mut xfers)
    }
    pub fn rssi(&self) -> Result<f32, Error> {
        let mut buf = [0; 2];
        self.read_many(Register::RssiConfig, &mut buf)?;
        if (buf[0] & 0x02) == 0x02 {
            //check if an Rssi measurement is in progress
            Ok(buf[1] as f32 / -2.0)
        } else {
            // RssiValue read is in progress which takes 2 bit cycles. Will wait 4 cycles for a buffer
            sleep(Duration::from_secs_f64(4.0 / self.bitrate as f64));
            self.read_many(Register::RssiConfig, &mut buf)?;
            if (buf[0] & 0x02) == 0x02 {
                Ok(buf[1] as f32 / -2.0)
            } else {
                Err(Error::Timeout("Rssi read timed out!".to_string()))
            }
        }
    }
    pub fn read_all(&self) -> Result<[u8; 0x4E], std::io::Error> {
        let mut ret = [0; 0x4E];
        self.read_many(Register::OpMode, &mut ret)?;
        Ok(ret)
    }
    pub fn set_power(&self, power: i8) -> Result<(), Error> {
        if power < -18 || power > 20 {
            return Err(Error::BadInputs(format!(
                "power must be [-18,20] dBm, ({})",
                power
            )));
        }
        if power <= 13 {
            // set power
            self.write_many(Register::TestPa1, &[0x55, 0x40, 0x70])?;
            self.write(Register::Ocp, 0x1A)?; // set Over current protection to 95 mA
            let power = (power - (-18)) as u8;
            self.write(Register::PaLevel, 0x80 | power)?;
        } else if power <= 17 {
            self.write_many(Register::TestPa1, &[0x55, 0x40, 0x70])?;
            self.write(Register::Ocp, 0x1F)?; // set Over current protection to 120 mA
            let power = (power - (-14)) as u8;
            self.write(Register::PaLevel, 0x60 | power)?;
        } else {
            self.write(Register::Ocp, 0x0F)?; // disable Over current protection
            self.write_many(Register::TestPa1, &[0x5D, 0x40, 0x7C])?; // high power mode
            let power = (power - (-11)) as u8;
            self.write(Register::PaLevel, 0x60 | power)?;
        }
        Ok(())
    }
    pub fn power(&self) -> Result<i8, Error> {
        // get the pa register
        let palevel = self.read(Register::PaLevel)?;
        // pa[012] are stored in bits 7, 6 & 5
        let pa012 = (palevel >> 5) & 0x07;
        if pa012 > 4 || pa012 < 2 {
            // the only valid values are pa0 on, pa1 on, or, pa1 & pa2 on
            return Err(Error::ChipMalfunction(format!(
                "Power amplifiers are in unknown state. RegPaLevel: {}",
                palevel
            )));
        }

        // the level is stored as an offset [0,31] stored in bits 4-0
        let palevel = palevel & 0x1F;
        if pa012 != 4 && palevel < 16 {
            return Err(Error::ChipMalfunction(
                "Power levels must be at least 16 when PA0 is not in use".to_string(),
            ));
        }
        let mut testpa = [0, 0, 0]; // middle byte is unused
        self.read_many(Register::TestPa1, &mut testpa)?;
        // determine the shift used when setting the dBm
        let shift = if testpa[0] == 0x55 && testpa[2] == 0x70 {
            match pa012 {
                2 | 4 => -18,
                3 => -14,
                _ => unreachable!(),
            }
        } else if testpa[0] == 0x5D && testpa[2] == 0x7C {
            if pa012 != 0x03 {
                /* TODO: evaluate whether this should be pa012 == 0x04;
                   in other words is PA1 + !PA2 + HP valid
                */
                return Err(Error::ChipMalfunction(
                    "High power mode is enabled when PA1 and PA2 are not both enabled.".to_string(),
                ));
            }
            -11
        } else {
            return Err(Error::ChipMalfunction(format!(
                "High power mode registers are in invalid state. RegTestPa[12] {:#04X} {:#04X}.",
                testpa[0], testpa[2]
            )));
        };
        Ok(shift + palevel as i8)
    }
    pub fn temp(&self) -> Result<i8, Error> {
        self.write(Register::Temp1, 0x08)?;
        for _ in 0..5 {
            sleep(Duration::from_micros(20));
            let mut buf = [0, 0];
            self.read_many(Register::Temp1, &mut buf)?;
            if buf[0] & 0x04 == 0 {
                return Ok(buf[1] as i8);
            }
        }
        Err(Error::Timeout("Temperature reading timed out!".to_string()))
    }

    fn get_fifo_not_empty(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::FifoNotEmpty,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[0] {
            if self.dio_map.map(1) == 2 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g1",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        if let Some(line) = &self.dios[2] {
            if self.dio_map.map(2) == 0 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g2",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        Ok(irq)
    }
    fn get_fifo_overrun(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::FifoOverrun,
            high: true,
            check,
        };
        Ok(irq)
    }
    fn get_packet_sent(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::PacketSent,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[0] {
            if self.mode == Mode::Tx && self.dio_map.map(0) == 0 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g0",
                )?;
                irq.line = Some(leh);
            }
        }
        Ok(irq)
    }
    fn get_payload_crc(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::PayloadReady,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[0] {
            if self.mode == Mode::Rx
                && ((self.pc.crc() && self.dio_map.map(0) == 0) || (self.dio_map.map(0) == 1))
            {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g0",
                )?;
                irq.line = Some(leh);
            }
        }
        if self.pc.crc() {
            irq.irq = Irq::CrcOk;
        }
        Ok(irq)
    }
    fn get_payload_ready(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::PayloadReady,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[0] {
            if self.mode == Mode::Rx && self.dio_map.map(0) == 1 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g0",
                )?;
                irq.line = Some(leh);
            }
        }
        Ok(irq)
    }

    fn get_fifo_full(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::FifoFull,
            high: false,
            check,
        };
        if let Some(line) = &self.dios[1] {
            if self.dio_map.map(1) == 0 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::FALLING_EDGE,
                    "rfm69_g1",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        if let Some(line) = &self.dios[3] {
            if self.dio_map.map(3) == 0 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::FALLING_EDGE,
                    "rfm69_g3",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        Ok(irq)
    }
    fn get_mode_ready(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_millis(1);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::ModeReady,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[4] {
            if self.mode == Mode::Tx && self.dio_map.map(4) == 0 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g4",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        if let Some(line) = &self.dios[5] {
            if self.dio_map.map(5) == 3 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g5",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        Ok(irq)
    }
    fn get_sync_address(&self) -> Result<IrqWait, Error> {
        let check = Duration::from_secs_f64(8.0 / self.bitrate as f64);
        let mut irq = IrqWait {
            line: None,
            rf: self,
            irq: Irq::SyncAddressMatch,
            high: true,
            check,
        };
        if let Some(line) = &self.dios[0] {
            if self.mode == Mode::Rx && self.dio_map.map(0) == 0x2 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g0",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        if let Some(line) = &self.dios[3] {
            if self.mode == Mode::Rx && self.dio_map.map(3) == 0x2 {
                let leh = line.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::RISING_EDGE,
                    "rfm69_g0",
                )?;
                irq.line = Some(leh);
                return Ok(irq);
            }
        }
        Ok(irq)
    }
    fn fifo_write(&self, buf: &[u8]) -> Result<(), Error> {
        // write inits bytes to fifo
        let mut sent = 0;
        let first = buf.len().min(66);
        self.write_many(Register::Fifo, &buf[0..first])?;
        sent += first;
        // ensure mode is ready before writing more
        let ready = self.get_mode_ready()?;
        ready.check_and_wait(Duration::from_millis(10))?;
        drop(ready);

        // send remaining bytes if there are any
        let iw = self.get_fifo_full()?;
        let fifo_wait = Duration::from_secs_f64(80.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10
        while sent < buf.len() {
            iw.check_and_wait(fifo_wait)?; // we need to check if the fifo is full
            self.write(Register::Fifo, buf[sent])?;
            sent += 1;
        }
        Ok(())
    }
    fn send_fixed(&self, buf: &[u8]) -> Result<(), Error> {
        if buf.len() == self.pc.len() as usize {
            self.fifo_write(buf)
        } else {
            Err(Error::BadInputs(format!(
                "Buffer length ({}) must match fixed length ({})!",
                buf.len(),
                self.pc.len()
            )))
        }
    }

    fn send_variable(&self, payload: &[u8]) -> Result<(), Error> {
        if payload.len() > 256 {
            return Err(Error::BadInputs(format!(
                "Buffer length ({}) cannot be greater than 256!",
                payload.len()
            )));
        } else if payload.len() < 2 {
            return Err(Error::BadInputs(
                "Buffer length must be at least two!".to_string(),
            ));
        } else if self.pc.aes() {
            if self.pc.filtering() != Filtering::None {
                if payload.len() > 50 {
                    return Err(Error::BadInputs(
                        "When AES is enabled with filterin, payload length must be less than 51"
                            .to_string(),
                    ));
                }
            } else if payload.len() > 65 {
                return Err(Error::BadInputs(
                    "When AES is enabled, payload length must be less than 66".to_string(),
                ));
            }
        } else if self.pc.is_variable() && payload.len() - 1 != payload[0] as usize {
            return Err(Error::BadInputs("When using the variable length format, the first byte must be the length of the buffer.".to_string()));
        }
        if self.verbose {
            eprintln!("send_variable: len {}", payload[0]);
        }
        self.fifo_write(payload)
    }
    fn send_unlimited(&self, _payload: &[u8]) -> Result<(), Error> {
        unimplemented!()
    }
    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        self.set_mode_internal(Mode::Tx)?;
        if self.pc.is_variable() {
            // variable packet
            self.send_variable(payload)
        } else if self.pc.is_unlimited() {
            self.send_unlimited(payload)
        } else {
            // fixed length
            self.send_fixed(payload)
        }?;
        let send_timeout =
            Duration::from_secs_f64((273.0 + self.preamble as f64) * 8.0 / self.bitrate as f64);
        self.get_packet_sent()?.check_and_wait(send_timeout)
    }
    fn fifo_read(&self, buf: &mut [u8], count: u8, timeout: Duration) -> Result<(), Error> {
        let count = count as usize;
        // wait for the byte to be received
        let fifo = self.get_fifo_not_empty()?;
        fifo.check_and_wait(timeout)?;

        let fifo_wait = Duration::from_secs_f64(160.0 / self.bitrate as f64); // 80.0 comes from 8 bits in and a byte x10
        for i in 0..(count - 1) {
            if let Err(e) = fifo.check_and_wait(fifo_wait) {
                if IrqWait::check_flag(&self, Irq::FifoOverrun)? {
                    return Err(Error::Timeout(
                        "Fifo overran while reading! Reading was too slow.".to_string(),
                    ));
                } else {
                    return Err(e);
                }
            }
            let b = self.read(Register::Fifo)?;
            if i < buf.len() {
                buf[i] = b;
            }
        }
        // at this point either CrcOk or PayloadReady should fire
        let ready = self.get_payload_crc()?;
        if let Err(_) = ready.check_and_wait(fifo_wait) {
            /* if the sync word and address didnt match we would have timed-out above so we dont need to check
              sync word interrupt.
            */
            let s = if self.pc.crc() {
                // check if this is a Crc Error or something else
                "A CRC Error occurred!".to_string()
            } else {
                let s = "A payload ready was never received, after starting message reception. With no CRC, this should never happen.".to_string();
                return Err(Error::ChipMalfunction(s));
            };
            let bad_fifo = if !self.pc.clear() {
                //if clear is not set we should get the bad payload
                let b = self.read(Register::Fifo)?;
                if count - 1 < buf.len() {
                    buf[count - 1] = b;
                }
                Some(count as usize)
            } else {
                None
            };
            Err(Error::BadMessage(s, bad_fifo))
        } else {
            let b = self.read(Register::Fifo)?;
            if count <= buf.len() {
                buf[count - 1] = b;
            }
            // check if we have overflowed the buffer
            if count > buf.len() {
                Err(Error::BufferOverflow(count as usize))
            } else {
                Ok(())
            }
        }
    }
    fn fifo_read_no_check(
        &self,
        buf: &mut [u8],
        count: u8,
        timeout: Duration,
    ) -> Result<(), Error> {
        debug_assert!(count <= 66);
        let irq = self.get_payload_crc()?;
        if let Err(e) = irq.check_and_wait(timeout) {
            if self.sc.on() || self.pc.filtering() != Filtering::None {
                if !IrqWait::check_flag(&self, Irq::SyncAddressMatch)? {
                    // the sync and address never seen, normal timeout
                    return Err(e);
                }
            } else {
                // if the sync and address filtering are disabled then this is a normal timeout
                return Err(e);
            }
            // From here on, it can be assumed a sync word or address was received
            if !self.pc.crc() {
                /* If CRC error didnt happen then it is something else
                  I don't know if this is even possible. Should this be unreachable arm instead?
                  With out CRC checking anything would be accept, so this are would represent a chip
                  malfunction.
                  UPDATE: so clearly this is reachable but why, I'm not sure yet
                */
                let mut fifo_dump = [0; 66];
                self.read_many_count(Register::Fifo, &mut fifo_dump, count)?;
                let rs = Decoder::new(16);
                let s = format!("A payload ready was never received, after starting message reception. With no CRC, this should never happen! dumping fifo:\n{:#04X?}\ngood validation: {}",
					&fifo_dump[..count as usize], !rs.is_corrupted(&fifo_dump[..count as usize]));
                return Err(Error::ChipMalfunction(s));
            }
            let s = "A CRC Error occured!".to_string();
            let bad_fifo = if !self.pc.clear() {
                // if clear is not set we should get bad payload for error message
                if let Err(e) = self.read_many_count(Register::Fifo, buf, count) {
                    // if the message was bad the we ignore the BufferOverflow Error
                    if let Error::BufferOverflow(_) = e {
                        Some(count as usize)
                    } else {
                        return Err(e);
                    }
                } else {
                    Some(count as usize)
                }
            } else {
                None
            };
            Err(Error::BadMessage(s, bad_fifo))
        } else {
            self.read_many_count(Register::Fifo, buf, count)
        }
    }
    fn recv_fixed(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, Error> {
        let len = self.pc.len();
        let ret = if len as usize > buf.len() {
            buf.len()
        } else {
            len as usize
        };
        if len <= 66 {
            // short messages can be read in one go
            if self.verbose {
                eprintln!("Doing fixed_recv(): short message {} ", len);
            }
            self.fifo_read_no_check(buf, len, timeout).map(|_| ret)
        } else {
            // long messages are required to be read continously
            if self.verbose {
                eprintln!("Doing fixed_recv(): long message {} ", len);
            }
            self.fifo_read(buf, len, timeout).map(|_| ret)
        }
    }
    fn recv_variable(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, Error> {
        let irq = self.get_fifo_not_empty()?;
        irq.check_and_wait(timeout)?;
        let len = self.read(Register::Fifo)?;
        let ret = len as usize;
        if len <= 66 {
            if self.verbose {
                eprintln!("Doing recv_variable(): short message {}", len);
            }
            self.fifo_read_no_check(buf, len, timeout).map(|_| ret)
        } else {
            if self.verbose {
                eprintln!("Doing recv_variable(): long message {}", len);
            }
            self.fifo_read(buf, len, timeout).map(|_| ret)
        }
    }
    pub fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, Error> {
        self.set_mode(Mode::Rx)?;
        if self.pc.is_variable() {
            self.recv_variable(buf, timeout)
        } else if self.pc.is_unlimited() {
            unimplemented!();
        } else {
            self.recv_fixed(buf, timeout)
        }
    }
    fn set_mode_internal(&mut self, mode: Mode) -> Result<(), Error> {
        if self.mode == mode {
            return Ok(());
        }
        if self.mode == Mode::Listen {
            self.write(Register::OpMode, set_bit(mode as u8, 5))?;
        }
        self.write(Register::OpMode, mode as u8)?;
        if mode == Mode::Rx {
            self.set_power(13)?;
        }
        self.mode = mode;
        Ok(())
    }
    /// Sets if the controller should write debug information to `stderr`.
    ///
    /// By default `verbose` is false. This function has no affect on the actual operation of the RFm69 chip.
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let mut rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// rfm.set_verbose(true); // defaults to false
    /// ```
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose
    }
    /// Reads and returns the version from the device.
    ///
    /// This value is read from the [`Version`] register. The version should always be 0x24 (36).
    /// # Example
    /// ```
    /// # use gpio_cdev::Chip;
    /// # use spidev::Spidev;
    /// use ham::rfm69::Rfm69;
    ///
    /// # let mut chip = Chip::new("/dev/gpiochip0").unwrap();
    /// # let rst = chip.get_line(24).unwrap();
    /// # let en = chip.get_line(3).unwrap();
    /// # let spidev = Spidev::open("/dev/spidev0.0").unwrap();
    /// let rfm = Rfm69::new(rst, en, spidev).unwrap();
    /// assert_eq!(rfm.version().unwrap(), 0x24);
    /// ```
    /// [`Version`]: ./enum.Register.html
    pub fn version(&self) -> Result<u8, std::io::Error> {
        self.read(Register::Version)
    }
    fn validate_version(&self) -> Result<(), Error> {
        let version = self.version()?;
        if version == 0x24 {
            Ok(())
        } else {
            Err(Error::ChipMalfunction(format!(
                "Version mismatch! expected 0x24, found {:#X}",
                version
            )))
        }
    }
}
impl Drop for Rfm69 {
    fn drop(&mut self) {
        if let Err(e) = self.en.set_value(0) {
            if self.verbose {
                eprintln!("Failed to set enable pin to off on drop: {:?}", e);
            }
        }
        if let Err(e) = self.rst.set_value(1) {
            if self.verbose {
                eprintln!("Failed to reset on drop to off on drop: {:?}", e);
            }
        }
    }
}

/// The `Register` enum corresponds to the different registers present on the RFM69 chip.
/// Some of these registers are used to configure the [`Rfm69`] controller to configure different function provided by the RFM69 chip.
/// They can be manually read from using [`Rfm69::read()`]/[`Rfm69::read_many()`] and written to using [`Rfm69::write()`]/[`Rfm69::write_many()`]
///
/// [`Rfm69::write()`]: ./struct.Rfm69.html#method.write
/// [`Rfm69::write_many()`]:  ./struct.Rfm69.html#method.write_many
/// [`Rfm69::read()`]: ./struct.Rfm69.html#method.read
/// [`Rfm69::read_many()`]:  ./struct.Rfm69.html#method.read_many
/// [`Rfm69`]: ./struct.Rfm69.html
#[derive(FromPrimitive, Clone, Copy, Debug, PartialEq)]
pub enum Register {
    Fifo = 0x00,
    OpMode = 0x01,
    DataModul = 0x02,
    BitrateMsb = 0x03,
    BitrateLsb = 0x04,
    FdevMsb = 0x05,
    FdevLsb = 0x06,
    FrfMsb = 0x07,
    FrfMid = 0x08,
    FrfLsb = 0x09,
    Osc1 = 0x0A,
    AfcCtrl = 0x0B,
    LowBat = 0x0C,
    Listen1 = 0x0D,
    Listen2 = 0x0E,
    Listen3 = 0x0F,
    Version = 0x10,
    PaLevel = 0x11,
    PaRamp = 0x12,
    Ocp = 0x13,
    AgcRef = 0x14,
    AgcThresh1 = 0x15,
    AgcThresh2 = 0x16,
    AgcThresh3 = 0x17,
    Lna = 0x18,
    RxBw = 0x19,
    AfcBw = 0x1A,
    OokPeak = 0x1B,
    OokAvg = 0x1C,
    OokFix = 0x1D,
    AfcFei = 0x1E,
    AfcMsb = 0x1F,
    AfcLsb = 0x20,
    FeiMsb = 0x21,
    FeiLsb = 0x22,
    RssiConfig = 0x23,
    RssiValue = 0x24,
    DioMapping1 = 0x25,
    DioMapping2 = 0x26,
    IrqFlags1 = 0x27,
    IrqFlags2 = 0x28,
    RssiThresh = 0x29,
    RxTimeout1 = 0x2A,
    RxTimeout2 = 0x2B,
    PreambleMsb = 0x2C,
    PreambleLsb = 0x2D,
    SyncConfig = 0x2E,
    SyncValue1 = 0x2F,
    SyncValue2 = 0x30,
    SyncValue3 = 0x31,
    SyncValue4 = 0x32,
    SyncValue5 = 0x33,
    SyncValue6 = 0x34,
    SyncValue7 = 0x35,
    SyncValue8 = 0x36,
    PacketConfig1 = 0x37,
    PayloadLength = 0x38,
    NodeAddrs = 0x39,
    BroadcastAddrs = 0x3A,
    AutoModes = 0x3B,
    FifoThresh = 0x3C,
    PacketConfig2 = 0x3D,
    AesKey1 = 0x3E,
    AesKey2 = 0x3F,
    AesKey3 = 0x40,
    AesKey4 = 0x41,
    AesKey5 = 0x42,
    AesKey6 = 0x43,
    AesKey7 = 0x44,
    AesKey8 = 0x45,
    AesKey9 = 0x46,
    AesKey10 = 0x47,
    AesKey11 = 0x48,
    AesKey12 = 0x49,
    AesKey13 = 0x4A,
    AesKey14 = 0x4B,
    AesKey15 = 0x4C,
    AesKey16 = 0x4D,
    Temp1 = 0x4E,
    Temp2 = 0x4F,
    TestLna = 0x58,
    TestPa1 = 0x5A,
    TestPa2 = 0x5C,
    TestDagc = 0x6F,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct SyncConfig([u8; 9]);
impl AsRef<SyncConfig> for SyncConfig {
    fn as_ref(&self) -> &SyncConfig {
        self
    }
}

impl Default for SyncConfig {
    fn default() -> Self {
        let mut buf = [0x01; 9];
        buf[0] = 0x98;
        SyncConfig(buf)
    }
}
impl SyncConfig {
    #[inline]
    pub fn on(&self) -> bool {
        bit(self.0[0], 7)
    }
    #[inline]
    pub fn set_on(&mut self, val: bool) -> &mut Self {
        self.0[0] = set_bit_to(self.0[0], 7, val);
        self
    }
    #[inline]
    pub fn fill(&self) -> bool {
        bit(self.0[0], 6)
    }
    #[inline]
    pub fn set_fill(&mut self, val: bool) -> &mut Self {
        self.0[0] = set_bit_to(self.0[0], 6, val);
        self
    }
    #[inline]
    pub fn set_len(&mut self, size: u8) -> &mut Self {
        assert!(size >= 1 && size <= 8);
        self.0[0] = (self.0[0] & 0xC7) | ((size - 1) << 3);
        self
    }
    #[inline]
    pub fn len(&self) -> u8 {
        ((self.0[0] >> 3) & 0x07) + 1
    }
    #[inline]
    pub fn sync_word(&self) -> [u8; 8] {
        let mut ret = [0; 8];
        ret.copy_from_slice(&self.0[1..9]);
        ret
    }
    #[inline]
    pub fn set_sync_word(&mut self, word: &[u8]) -> &mut Self {
        assert!(word.len() <= 8);
        assert!(word.iter().position(|x| *x == 0x00) == None);
        for (i, v) in word.iter().enumerate() {
            self.0[i + 1] = *v;
        }
        for i in word.len()..8 {
            self.0[i + 1] = 0x01;
        }
        self
    }
    #[inline]
    pub fn error(&self) -> u8 {
        self.0[0] & 0x07
    }
    #[inline]
    pub fn set_error(&mut self, error: u8) -> &mut Self {
        assert!(error < 8);
        self.0[0] = (self.0[0] & 0xF8) | error;
        self
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PacketConfig([u8; 4], [u8; 2]);

impl PacketConfig {
    #[inline]
    pub fn rst_value() -> Self {
        PacketConfig([0x10, 0x40, 0x00, 0x00], [0x0F, 0x02])
    }
    #[inline]
    pub fn is_variable(&self) -> bool {
        bit(self.0[0], 7)
    }
    #[inline]
    pub fn set_variable(mut self, val: bool) -> Self {
        self.0[0] = set_bit_to(self.0[0], 7, val);
        self
    }
    #[inline]
    pub fn set_unlimited(mut self) -> Self {
        self.0[0] = unset_bit(self.0[0], 7);
        self.0[1] = 1;
        self
    }
    #[inline]
    pub fn is_fixed(&self) -> bool {
        !self.is_variable() && self.len() != 0
    }
    #[inline]
    pub fn is_unlimited(&self) -> bool {
        !self.is_variable() && self.len() == 0
    }
    #[inline]
    pub fn len(&self) -> u8 {
        self.0[1]
    }
    #[inline]
    pub fn set_len(mut self, len: u8) -> Self {
        self.0[1] = len;
        self
    }
    #[inline]
    pub fn aes(&self) -> bool {
        bit(self.1[1], 0)
    }
    #[inline]
    pub fn restart(&self) -> bool {
        bit(self.1[1], 1)
    }
    #[inline]
    pub fn delay(&self) -> u8 {
        (self.1[1] >> 4) & 0x0F
    }
    #[inline]
    pub fn threshold(&self) -> u8 {
        (self.1[0] >> 1) & 0x7F
    }
    #[inline]
    pub fn set_threshold(mut self, threshold: u8) -> Self {
        assert!(threshold < 0x80);
        self.1[0] = (self.1[0] & 0xF0) | threshold;
        self
    }
    #[inline]
    pub fn tx_start(&self) -> bool {
        bit(self.1[0], 7)
    }
    #[inline]
    pub fn filtering(&self) -> Filtering {
        Filtering::from_u8((self.0[0] >> 1) & 0x03).unwrap()
    }
    #[inline]
    pub fn set_filtering(mut self, filtering: Filtering) -> Self {
        assert_ne!(filtering, Filtering::Reserved);
        self.0[0] = (self.0[0] & 0xF9) | ((filtering as u8) << 1);
        self
    }
    #[inline]
    pub fn clear(&self) -> bool {
        bit(self.0[0], 3)
    }
    #[inline]
    pub fn set_clear(mut self, val: bool) -> Self {
        self.0[0] = set_bit_to(self.0[0], 3, val);
        self
    }
    #[inline]
    pub fn crc(&self) -> bool {
        bit(self.0[0], 4)
    }
    #[inline]
    pub fn set_crc(mut self, val: bool) -> Self {
        self.0[0] = set_bit_to(self.0[0], 4, val);
        self
    }
    #[inline]
    pub fn dc(&self) -> DCFree {
        DCFree::from_u8((self.0[0] >> 5) & 0x03).unwrap()
    }
    #[inline]
    pub fn set_dc(mut self, dc: DCFree) -> Self {
        assert_ne!(dc, DCFree::Reserved);
        self.0[0] = (self.0[0] & 0x9F) | ((dc as u8) << 5);
        self
    }
    #[inline]
    pub fn address(&self) -> u8 {
        self.0[2]
    }
    #[inline]
    pub fn set_address(mut self, addr: u8) -> Self {
        self.0[2] = addr;
        self
    }
    #[inline]
    pub fn broadcast(&self) -> u8 {
        self.0[3]
    }
    #[inline]
    pub fn set_broadcast(mut self, addr: u8) -> Self {
        self.0[3] = addr;
        self
    }
    pub fn validate(&self) -> Result<(), Error> {
        if self.aes() {
            // perform aes validations
            // variable length validation is done when sedning messages, not configuring
            if self.is_unlimited() {
                return Err(Error::BadInputs(
                    "AES and unlimited packet size are incompatiable.".to_string(),
                ));
            }
            if self.is_fixed() {
                if self.filtering() != Filtering::None {
                    if self.len() >= 66 {
                        return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering enabled, length must be less than 66".to_string()));
                    }
                } else {
                    if self.len() >= 65 {
                        return Err(Error::BadInputs("In fixed mode, when AES is enabled and filtering disabled, length must be less than 65".to_string()));
                    }
                }
            }
        }
        Ok(())
    }
}

impl fmt::Display for PacketConfig {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{{ variable: {}, length: {} }}",
            self.is_variable(),
            self.len()
        )
    }
}

impl Default for PacketConfig {
    fn default() -> Self {
        PacketConfig([0x10, 0x40, 0x00, 0x00], [0x8F, 0x02])
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct DioMapping(u16);
impl Default for DioMapping {
    fn default() -> Self {
        DioMapping(0x0007)
    }
}

impl DioMapping {
    #[inline]
    pub fn dio_map(&self) -> [u8; 6] {
        let mut ret = [0; 6];
        for (i, map) in ret.iter_mut().enumerate() {
            *map = ((self.0 >> ((5 - i) + 4)) & 0x3) as u8;
        }
        ret
    }
    #[inline]
    pub fn set_maps(mut self, maps: &[u8]) -> Self {
        assert!(maps.len() <= 6);
        assert_eq!(maps.iter().position(|x| *x >= 4), None);
        // zero the bits that are changing with a mask
        self.0 &= (0xFFF0_u16 << (maps.len() * 2)).reverse_bits();
        for (i, map) in maps.iter().enumerate() {
            self.0 |= (*map as u16) << (i * 2 + 4);
        }
        self
    }
    #[inline]
    pub fn clkout(&self) -> u8 {
        (self.0 as u8) & 0x07
    }
    #[inline]
    pub fn set_clkout(mut self, clkout: u8) -> Self {
        assert!(clkout < 8);
        self.0 &= 0xFFF8;
        self.0 |= clkout as u16;
        self
    }
    #[inline]
    pub fn map(&self, index: u8) -> u8 {
        assert!(index < 6);
        (self.0 >> ((5 - index) + 4)) as u8
    }
    #[inline]
    pub fn set_map(mut self, index: u8, map: u8) -> Self {
        assert!(index < 6);
        assert!(map < 4);
        self.0 &= 0xFFFC_u16.rotate_left((5 - index as u32) * 2 + 4); // zero target
        self.0 |= (map as u16) << ((5 - index) * 2 + 4);
        self
    }
}

impl From<DioMapping> for [u8; 2] {
    #[inline]
    fn from(dio_map: DioMapping) -> Self {
        dio_map.0.to_be_bytes()
    }
}
impl From<[u8; 2]> for DioMapping {
    #[inline]
    fn from(bytes: [u8; 2]) -> Self {
        DioMapping(u16::from_be_bytes(bytes))
    }
}

#[derive(FromPrimitive, PartialEq, Clone, Copy, Debug)]
pub enum Mode {
    Listen = 0x40,
    Sleep = 0x00,
    Standby = 0x04,
    FS = 0x08,
    Tx = 0x0C,
    Rx = 0x10,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum Irq {
    ModeReady = 0x07,
    RxReady = 0x06,
    TxReady = 0x05,
    PllLock = 0x04,
    Rssi = 0x03,
    Timeout = 0x02,
    AutoMode = 0x01,
    SyncAddressMatch = 0x00,
    FifoFull = 0x0F,
    FifoNotEmpty = 0x0E,
    FifoLevel = 0x0D,
    FifoOverrun = 0x0C,
    PacketSent = 0x0B,
    PayloadReady = 0x0A,
    CrcOk = 0x09,
}

#[derive(FromPrimitive, PartialEq, Debug)]
pub enum DCFree {
    None = 0x00,
    Manchester = 0x01,
    Whitening = 0x02,
    Reserved = 0x03,
}

#[derive(FromPrimitive, PartialEq, Debug)]
pub enum Filtering {
    None = 0x00,
    Address = 0x01,
    Both = 0x02,
    Reserved = 0x03,
}
