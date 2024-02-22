#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use panic_itm as _; // panic handler
                    // use stm32f1::stm32f103;

use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let mut led2 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();
        led2.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();
        led2.set_low();
    }
}

// use stm32f1xx_hal::prelude::*;
// use stm32f1xx_hal::{
//     delay::Delay
// };
//
// #[entry]
// fn main() -> ! {
//     let peripherals = stm32f103::Peripherals::take().unwrap();
//     let gpioc = &peripherals.GPIOC;
//     gpioc.odr.modify(|_, w| w.odr14().set_bit());
//
//     // let ms = 50_u8;
//     loop {
//         // delay.delay_ms(ms);
//     }
// }
