#![no_std]
#![no_main]

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::pac;
use feather_m0 as bsp;

use bsp::I2c;
use bsp::entry;

use hal::sercom::i2c;
use hal::delay::Delay;
use hal::clock::GenericClockController;

use pac::{CorePeripherals,Peripherals};

const LENGTH: usize = 1;
const DHT_ADDRESS: u8 = 0x38;
const TEMPERATURE_THRESHOLD: f32 = 25.0;

#[entry]
fn main() -> ! {
    // initialize peripherals
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut peripherals = Peripherals::take().unwrap();

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
    let mut pins = bsp::Pins::new(peripherals.PORT);

    loop {}
}
