#![no_std]
#![no_main]

use bsp::hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin;
use bsp::pin_alias;
#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::ehal;
use bsp::pac;
use feather_m0 as bsp;

use bsp::{entry,I2cSercom,Sda,Scl,I2c};

use hal::delay::Delay;
use hal::time::Hertz;
use hal::clock::GenericClockController;

use ehal::prelude::*;

use pac::{CorePeripherals,Peripherals,PM};

const TEMPERATURE_THRESHOLD: f32 = 25.0;
const BUFFER_SIZE: u8 = 7;

struct DHT20 {
    address: u8,
    i2c: I2c,
    temperature: f32,
    humidity: f32
}

impl DHT20 {
    fn new(clocks: &mut GenericClockController, baud: impl Into<Hertz>, sercom: I2cSercom, pm: &mut PM, sda: impl Into<Sda>, scl: impl Into<Scl>, delay: &mut Delay) -> Self {
        let mut buffer = [0];
        let mut dht20 = Self {
            address: 0x38,
            i2c: bsp::i2c_master(clocks, baud, sercom, pm, sda, scl),
            temperature: 0.0,
            humidity: 0.0
        };

        delay.delay_ms(40u8); // delay 40ms for startup
        dht20.i2c.write_read(dht20.address, &[0x71], &mut buffer).unwrap(); // get status word

        if (buffer[0] & (1 << 3)) != 0 {
            dht20.i2c.write(dht20.address, &[0x08, 0x00]).unwrap();
            delay.delay_ms(10u8);
        } // if status word is not 1 then send initialization and delay 10ms
        
        dht20
    }

    fn measure(&mut self, delay: &mut Delay) {
        let mut measurement_data = [0u8; 7];

        self.i2c.write_read(self.address, &[0xAC], &mut measurement_data).unwrap();

        delay.delay_ms(80u8); // wait for measurement

        
        let mut measurement_completed = false;
        while !measurement_completed {
            delay.delay_ms(80u8);
            self.i2c.write_read(self.address, &[0xAC], &mut measurement_data).unwrap();
            measurement_completed = (measurement_data[0] & 0x80) == 0;
        }

        let humidity_raw = ((measurement_data[1] as u16) << 12) | ((measurement_data[2] as u16) << 4) | ((measurement_data[3] as u16) >> 4);
        let temperature_raw = ((measurement_data[3] as u16 & 0x0F) << 16) | ((measurement_data[4] as u16) << 8) | measurement_data[5] as u16;

        // Calculate humidity and temperature values
        self.humidity =  ((humidity_raw as f32) * 100.0 / 1048576.0) as f32;
        self.temperature = (((temperature_raw as f32) * 200.0 / 1048576.0) - 50.0) as f32;
    }
}

#[entry]
fn main() -> ! {
    // initialize peripherals
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

    let pins = bsp::Pins::new(peripherals.PORT);

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL
    );

    // initialize delay
    let mut delay = Delay::new(core_peripherals.SYST, &mut clocks);

    // initialize DHT20
    let mut dht20 = DHT20::new(&mut clocks, Hertz(100000), peripherals.SERCOM3, &mut peripherals.PM, pins.sda, pins.scl, &mut delay);

    // turn LED on if temperature is above TEMPERATURE_THRESHOLD
    let mut red_led: bsp::RedLed = pin_alias!(pins.red_led).into();

    loop {
        delay.delay_ms(200u8);
        dht20.measure(&mut delay);

        if dht20.temperature >= TEMPERATURE_THRESHOLD {
            red_led.set_high().unwrap();
        } else {
            red_led.set_low().unwrap();
        }
    }
}
