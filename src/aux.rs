use core::fmt::Write;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use mpu6050::*;
use shared_bus_rtic::{CommonBus, SharedBus};
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    afio::{self, MAPR}, flash, gpio::{self, gpioa, gpiob, gpioc, Alternate, OpenDrain, Output, Pin, PushPull}, i2c::{self, BlockingI2c}, pac::{self, interrupt, AFIO, FLASH, I2C1, RCC, TIM1, TIM2, TIM3}, prelude::*, rcc::{Clocks, Rcc}, timer::{Channel, Counter, CounterMs, Event, Tim2NoRemap, Timer2}
};

type LedPin = gpio::PC13<Output<PushPull>>;
type BreakPin = gpio::PA8<Output<PushPull>>;
type DirPin = gpio::PA2<Output<PushPull>>;
type BScl = Pin<'B', 8, Alternate<OpenDrain>>;
type BSda = Pin<'B', 9, Alternate<OpenDrain>>;
type BlockingI2cPB89 = i2c::BlockingI2c<
    pac::I2C1,
    (
        BScl,
        BSda,
    ),
>;
type I2cDisplay =
    Ssd1306<I2CInterface<SharedBus<BlockingI2cPB89>>, DisplaySize128x64, TerminalMode>;
type I2cMpu6050 = Mpu6050<SharedBus<BlockingI2cPB89>>;

pub fn init_devices() -> (afio::Parts, Clocks, Counter<TIM3, 1000>, gpioa::Parts, gpiob::Parts, gpioc::Parts, I2C1, TIM1, TIM2) {
    // Get access to the core peripherals from the cortex-m crate
    // let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();
    let afio = dp.AFIO.constrain();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

    let timer = dp.TIM3.counter_ms(&clocks);

    // Acquire the GPIO* peripheral
    let gpioc = dp.GPIOC.split();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    

    (afio, clocks, timer, gpioa, gpiob, gpioc, dp.I2C1, dp.TIM1, dp.TIM2)
}

pub fn init_i2c(scl: BScl, sda: BSda, i2c1: I2C1, mapr: &mut MAPR, clocks: Clocks) -> &'static CommonBus<BlockingI2c<I2C1, (BScl, BSda)>> {
    let i2c_2 = i2c::BlockingI2c::i2c1(
        i2c1,
        (scl, sda),
        mapr,
        i2c::Mode::Standard {
            frequency: 100_000.Hz(),
        },
        clocks,
        // below are different timeouts
        1000, // start_timeout_us
        10, // start_retries
        1000, // addr_timeout_us
        1000, // data_timeout_us
    );

    let i2c_sbus = shared_bus_rtic::new!(i2c_2, BlockingI2cPB89);
    i2c_sbus
}

pub fn init_display(i2c_sbus: &CommonBus<BlockingI2c<I2C1, (BScl, BSda)>>) -> Ssd1306<I2CInterface<&CommonBus<BlockingI2c<I2C1, (BScl, BSda)>>>, DisplaySize128x64, TerminalMode> {
    let interface = I2CDisplayInterface::new(i2c_sbus.acquire());

    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();
    display
}