#![no_std]
#![no_main]

use bsp::ehal::blocking::i2c;
use bsp::hal::sercom::Sercom3;
use bsp::hal::sercom::i2c::Pads;
use bsp::pin_alias;
#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::ehal;
use bsp::pac;
use hal::time::U32Ext;
use feather_m0 as bsp;

use bsp::{entry,I2cSercom,Sda,Scl,I2c};

use hal::delay::Delay;
use hal::time::Hertz;
use hal::clock::GenericClockController;

use ehal::prelude::*;

use pac::{CorePeripherals,Peripherals,PM};

const TEMPERATURE_THRESHOLD: f32 = 15.0;
const BUFFER_SIZE: u8 = 7;

struct DHT20 {
    address: u8,
    i2c: I2c,
    temperature: f32,
    humidity: f32
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
        led: &mut bsp::RedLed
    ) -> Self {
        let mut buffer = [0u8; 7];

        let config = i2c::Config::new(
            &pm,
            sercom,
            hal::sercom::i2c::Pads::<Sercom3>::new(sda, scl),
            10.mhz()
        );

        let mut dht20 = Self {
            address: 0x38,
            i2c: bsp::i2c_master(clocks, baud, sercom, pm, sda, scl),
            temperature: 0.0,
            humidity: 0.0
        };

        delay.delay_ms(40u8); // delay 40ms for startup
         
        dht20.i2c.write_read(dht20.address, &[0x71], &mut buffer).unwrap(); // get status word

        // led.set_high().unwrap();
        //
        // if (buffer[0] & (1 << 3)) != 0 {
        //     dht20.i2c.write(dht20.address, &[0x08, 0x00]).unwrap();
        //     delay.delay_ms(10u8);
        // } // if status word is not 1 then send initialization and delay 10ms
        // 
        dht20
    }

    fn measure(&mut self, delay: &mut Delay) {
        let mut measurement_data = [0u8; 6];

        self.i2c.write_read(self.address, &[0xAC], &mut measurement_data).unwrap();

        delay.delay_ms(80u8); // wait for measurement

        
        let mut measurement_completed = false;
        while !measurement_completed {
            delay.delay_ms(80u8);
            self.i2c.write_read(self.address, &[0xAC], &mut measurement_data).unwrap();
            measurement_completed = (measurement_data[0] & 0x80) == 0;
        }

        let humidity_raw = u16::from_be_bytes([measurement_data[1], measurement_data[2]]);
        let temperature_raw = u16::from_be_bytes([measurement_data[3], measurement_data[4]]);

        // Calculate humidity and temperature values
        self.humidity =  (humidity_raw as f32) / 10.0;
        self.temperature = (temperature_raw as f32) / 10.0 as f32;
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
        &mut peripherals.NVMCTRL
    );


    // initialize delay
    let mut delay = Delay::new(core_peripherals.SYST, &mut clocks);

    let mut red_led: bsp::RedLed = pin_alias!(pins.red_led).into();
    // initialize DHT20
    let mut dht20 = DHT20::new(&mut clocks, 100.khz(), peripherals.SERCOM3, &mut peripherals.PM, pins.sda, pins.scl, &mut delay, &mut red_led);

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
