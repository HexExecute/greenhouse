use bsp::ehal;
use feather_m0 as bsp;

use bitflags::bitflags;
use ehal::blocking::i2c::WriteRead;
use ehal::blocking::i2c::Write;
use ehal::blocking::delay::DelayMs;


const DHT20_ADDRESS: u8 = 0x38;

bitflags! {
    struct StatusFlags: u8 {
        const BUSY = (1 << 7);
        const MODE = ((1 << 6) | (1 << 5));
        const CRC = (1 << 4);
        const CALIBRATION_ENABLE = (1 << 3);
        const FIFO_ENABLE = (1 << 2);
        const FIFO_FULL = (1 << 1);
        const FIFO_EMPTY = (1 << 0);
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    Uncalibrated,
    Bus(E),
    Checksum
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}


pub struct Humidity {
    h: u32
}

impl Humidity {
    fn rh(&self) -> f32 {
        100.0 * (self.h as f32) / ((1 << 20) as f32)
    }

    pub fn raw(&self) -> u32 { self.h }
}


pub struct Temperature {
    t: u32
}

impl Temperature {
    pub fn celsius(&self) -> f32 {
        (200.0 * (self.t as f32) / ((1 << 20) as f32)) - 50.0
    }

    pub fn raw(&self) -> u32 { self.t }
}


pub struct DHT20<I2C, D> {
    i2c: I2C,
    pub delay: D
}


impl<I2C, D, E> DHT20<I2C, D>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    D: DelayMs<u16>
{
    pub fn new(i2c: I2C, delay: D) -> Result<Self, Error<E>> {
        let mut dev = Self { i2c, delay };

        dev.reset()?;
        dev.calibrate()?;

        Ok(dev)
    }

    fn status(&mut self) -> Result<StatusFlags, E> {
        let buf = &mut [0u8; 1];

        self.i2c.write_read(DHT20_ADDRESS, &[0u8], buf)?;

        Ok(StatusFlags::from_bits_truncate(buf[0]))
    }

    pub fn calibrate(&mut self) -> Result<(), Error<E>> {
        self.i2c.write(DHT20_ADDRESS, &[0xE1, 0x08, 0x00])?;

        while self.status()?.contains(StatusFlags::BUSY) {
            self.delay.delay_ms(10);
        }

        while self.status()?.contains(StatusFlags::CALIBRATION_ENABLE) {
            // self.delay.delay_ms(10);
            return Err(Error::Uncalibrated);
        }

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), E> {
        self.i2c.write(DHT20_ADDRESS, &[0xBA])?;

        self.delay.delay_ms(20);

        Ok(())
    }

    pub fn read(&mut self) -> Result<(Humidity, Temperature), Error<E>> {
        // lazy_static! {
        //     static ref CRC: CrcAlgo<u8> = CrcAlgo::<u8>::new(49, 8, 0xFF, 0x00, false);
        // }

        self.i2c.write(DHT20_ADDRESS, &[0xAC, 0x33, 0x00])?;

        while self.status()?.contains(StatusFlags::BUSY) {
            self.delay.delay_ms(10);
        }

        let buf = &mut [0u8; 7];
        self.i2c.write_read(DHT20_ADDRESS, &[0u8], buf)?;

        // let crc = &mut 0u8;
        // CRC.init_crc(crc);

        // if CRC.update_crc(crc, &buf[..=5]) != buf[6] {
        //     return Err(Error::Checksum);
        // };

        let status = StatusFlags::from_bits_truncate(buf[0]);
        if !status.contains(StatusFlags::CALIBRATION_ENABLE) {
            return Err(Error::Uncalibrated);
        }

        let hum = ((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);
        let temp = (((buf[3] as u32) & 0x0f) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);

        Ok((Humidity { h: hum }, Temperature { t: temp }))
    }
}
