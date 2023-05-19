#![no_std]
#![no_main]

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::ehal;
use bsp::pac;
use feather_m0 as bsp;

use bsp::entry;

use hal::delay::Delay;
use hal::clock::GenericClockController;
use ehal::blocking::i2c;

use pac::{CorePeripherals,Peripherals};

const LENGTH: usize = 1;
const DHT_ADDRESS: u8 = 0x38;
const TEMPERATURE_THRESHOLD: f32 = 25.0;

struct TemperatureSensorDriver<I2C> {
    i2c: I2C
}

impl<I2C, E> TemperatureSensorDriver<I2C>
where
    I2C: i2c::WriteRead<Error = E>,
{
    pub fn read_temperature(&mut self) -> Result<u8, E> {
        let mut temp = [0];
        self.i2c
            .write_read(DHT_ADDRESS, &[DHT_REG], &mut temp)
            .and(Ok(temp[0]))
    }
}

#[entry]
fn main() -> ! {
    // initialize peripherals
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

    let mut pins = bsp::Pins::new(peripherals.PORT);

    // intialize clock controller
    let mut clock_controller = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL
     );

    // initialize delay
    let mut delay = Delay::new(core_peripherals.SYST, &mut clock_controller);

    // initialize I2C
    let mut buffer: &mut [u8];
    let mut dht_sen = i2c::WriteRead::write_read(i2c, DHT_ADDRESS, &[0], buffer);

    loop {}
}
