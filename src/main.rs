#![no_std]
#![no_main]

use bme280_rs::Bme280;
use bme280_rs::Configuration;
use bme280_rs::Oversampling;
use bme280_rs::SensorMode;
// use bme280::i2c::BME280;
use bsp::Scl;
use bsp::Sda;
use bsp::hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

// mod dht20;

use bsp::ehal;
use bsp::hal;
use bsp::pac;
use feather_m0 as bsp;
use hal::time::U32Ext;

use bsp::{entry, pin_alias};
// use bsp::{entry, I2c, I2cSercom, Scl, Sda, i2c_master, pin_alias};

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::sercom::{i2c, Sercom3};
// use hal::time::Hertz;

// use ehal::digital::v2::OutputPin;
// use ehal::prelude::*;

use pac::{CorePeripherals, Peripherals};

const TEMPERATURE_THRESHOLD: f32 = 5.0;

#[entry]
fn main() -> ! {
    // initialize peripherals
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

    let mut pins = bsp::Pins::new(peripherals.PORT);

    // let pads = i2c::Pads::<Sercom3>::new(pins.pa)

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    // initialize delay
    let mut delay = Delay::new(core_peripherals.SYST, &mut clocks);


    // initialize i2c and configure i2c
    let pads = i2c::Pads::<Sercom3, Sda, Scl>::new(pins.sda, pins.scl);

    let mut i2c_config = i2c::Config::new(&peripherals.PM, peripherals.SERCOM3, pads, 100.khz());
    i2c_config.set_baud(100.khz());
    let i2c_bus = i2c_config.enable();
    
    let mut bme280 = Bme280::new(i2c_bus, delay);

    bme280.init().unwrap();

    bme280.set_sampling_configuration(
        Configuration::default()
            .with_temperature_oversampling(Oversampling::Oversample1)
            .with_pressure_oversampling(Oversampling::Oversample1)
            .with_humidity_oversampling(Oversampling::Oversample1)
            .with_sensor_mode(SensorMode::Normal)
    ).unwrap();


    // initialize bme280 sensor
    // let mut bme280 = BME280::new_primary(i2c_bus);

    let mut red_led: bsp::RedLed = pin_alias!(pins.red_led).into();

    if let Some(temperature) = bme280.read_temperature().unwrap() {
        if temperature >= TEMPERATURE_THRESHOLD {
            red_led.set_high().unwrap();
        }
    }

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
