#![no_std]
#![no_main]

use bsp::ehal::blocking::i2c;
use bsp::hal::sercom::i2c::Pads;
use bsp::hal::sercom::Sercom3;
use bsp::i2c_master;
use bsp::pin_alias;
#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::ehal;
use bsp::hal;
use bsp::pac;
use feather_m0 as bsp;
use hal::time::U32Ext;

use bsp::{entry, I2c, I2cSercom, Scl, Sda};

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::time::Hertz;

use ehal::digital::v2::OutputPin;
use ehal::prelude::*;

use pac::{CorePeripherals, Peripherals, PM};

const TEMPERATURE_THRESHOLD: f32 = 15.0;
const BUFFER_SIZE: u8 = 7;

struct DHT20 {
    address: u8,
    i2c: I2c,
    temperature: f32,
    humidity: f32,
}

impl DHT20 {
    fn new(
        clocks: &mut GenericClockController,
        baud: impl Into<Hertz>,
        sercom: I2cSercom,
        pm: &mut PM,
        sda: impl Into<Sda>,
        scl: impl Into<Scl>,
        delay: &mut Delay,
        led: &mut bsp::RedLed,
    ) -> Self {
        let mut buffer = [0u8];

        // led.set_high().unwrap();

        let mut dht20 = Self {
            address: 0x38,
            // i2c: {
            //     let mut i2c = hal::sercom::i2c::Config::new(
            //         &pm,
            //         sercom,
            //         hal::sercom::i2c::Pads::<Sercom3, Sda, Scl>::new(sda.into(), scl.into()),
            //         10.mhz(),
            //     );
            //     i2c.set_baud(baud);
            //     i2c.enable()
            // },
            i2c: i2c_master(clocks, baud, sercom, pm, sda, scl),
            temperature: 0.0,
            humidity: 0.0,
        };

        delay.delay_ms(40u8); // delay 40ms for start

        // led.set_high().unwrap();

        dht20.i2c
            .read(0x71, &mut buffer)
            .unwrap();

        if buffer[0] != 1 {
            dht20
                .i2c
                .write(0xbe, &[0x08, 0x00])
                .unwrap();
            delay.delay_ms(10u8);
        }

        led.set_high().unwrap();

        dht20
    }

    fn measure(&mut self, delay: &mut Delay) {
        let mut measurement_data = [0u8; 7];

        self.i2c
            .write(self.address, &[0xAC, 0x33, 0x00])
            .unwrap();

        delay.delay_ms(80u8); // wait for measurement

        self.i2c
            .read(self.address, &mut measurement_data)
            .unwrap();

        let mut measurement_completed = false;
        while !measurement_completed {
            // check if Bit [7] of status value is 1
            if measurement_data[0] & (1 << 7) == 0 { measurement_completed = true; }

            delay.delay_ms(80u8); // wait for measurement

            self.i2c
                .read(self.address, &mut measurement_data)
                .unwrap();
        }

        let raw_humidity = u32::from_be_bytes([0, measurement_data[1], measurement_data[2], measurement_data[3]]);
        let raw_temperature = i32::from_be_bytes([measurement_data[4], measurement_data[5], 0, 0]);
        let _crc = measurement_data[6];

        self.humidity = raw_humidity as f32 / 10.0;
        self.temperature = raw_temperature as f32 / 10.0;

        // let mut measurement_completed = false;
        // while !measurement_completed {
        //     delay.delay_ms(80u8);
        //     self.i2c
        //         .write_read(self.address, &[0xAC], &mut measurement_data)
        //         .unwrap();
        //     measurement_completed = (measurement_data[0] & 0x80) == 0;
        // }
        //
        // let humidity_raw = u16::from_be_bytes([measurement_data[1], measurement_data[2]]);
        // let temperature_raw = u16::from_be_bytes([measurement_data[3], measurement_data[4]]);
        //
        // // Calculate humidity and temperature values
        // self.humidity = (humidity_raw as f32) / 10.0;
        // self.temperature = (temperature_raw as f32) / 10.0 as f32;
    }
}

#[entry]
fn main() -> ! {
    // initialize peripherals
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

    let pins = bsp::Pins::new(peripherals.PORT);

    // let pads = i2c::Pads::<Sercom3>::new(pins.pa)

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    // initialize delay
    let mut delay = Delay::new(core_peripherals.SYST, &mut clocks);

    let mut red_led: bsp::RedLed = pin_alias!(pins.red_led).into();

    // initialize DHT20

    let mut dht20 = DHT20::new(
        &mut clocks,
        100.khz(),
        peripherals.SERCOM3,
        &mut peripherals.PM,
        pins.sda,
        pins.scl,
        &mut delay,
        &mut red_led,
    );

    dht20.measure(&mut delay);

    if dht20.temperature > 0.0 {
        red_led.set_high().unwrap();
    }

    // turn LED on if temperature is above TEMPERATURE_THRESHOLD

    loop {
        //delay.delay_ms(200u8);
        //dht20.measure(&mut delay);
        //
        // red_led.set_high().unwrap();
        // if dht20.temperature >= TEMPERATURE_THRESHOLD {
        //     red_led.set_high().unwrap();
        // } else {
        //     red_led.set_low().unwrap();
        // }
        // delay.delay_ms(100u8);
        // red_led.set_high().unwrap();
        // delay.delay_ms(100u8);
        // red_led.set_low().unwrap();
    }
}
