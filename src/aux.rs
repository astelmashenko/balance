// use core::fmt::Write;

use shared_bus_rtic::CommonBus;
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    afio::{self, MAPR},
    gpio::{
        gpioa::{self},
        gpiob, gpioc, Alternate, OpenDrain, Pin,
    },
    i2c::{self, BlockingI2c},
    pac::{self, interrupt, I2C1, TIM1, TIM2, TIM3},
    prelude::*,
    rcc::Clocks,
    timer::{Counter, Event},
};

type BScl = Pin<'B', 8, Alternate<OpenDrain>>;
type BSda = Pin<'B', 9, Alternate<OpenDrain>>;
type BlockingI2cPB89 = i2c::BlockingI2c<pac::I2C1, (BScl, BSda)>;

pub fn init_devices() -> (
    afio::Parts,
    Clocks,
    Counter<TIM3, 1000>,
    gpioa::Parts,
    gpiob::Parts,
    gpioc::Parts,
    I2C1,
    TIM1,
    TIM2,
) {
    // Get access to the core peripherals from the cortex-m crate
    // let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();
    let afio = dp.AFIO.constrain();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies
    // let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(6.MHz())
        .freeze(&mut flash.acr);

    let timer = dp.TIM3.counter_ms(&clocks);

    // Acquire the GPIO* peripheral
    let gpioc = dp.GPIOC.split();
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    (
        afio, clocks, timer, gpioa, gpiob, gpioc, dp.I2C1, dp.TIM1, dp.TIM2,
    )
}

pub fn init_i2c(
    scl: BScl,
    sda: BSda,
    i2c1: I2C1,
    mapr: &mut MAPR,
    clocks: Clocks,
) -> &'static CommonBus<BlockingI2c<I2C1, (BScl, BSda)>> {
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
        10,   // start_retries
        1000, // addr_timeout_us
        1000, // data_timeout_us
    );

    let i2c_sbus = shared_bus_rtic::new!(i2c_2, BlockingI2cPB89);
    i2c_sbus
}

type DisplayIface = I2CInterface<&'static CommonBus<BlockingI2c<I2C1, (BScl, BSda)>>>;

pub fn init_display(
    i2c_sbus: &'static CommonBus<BlockingI2c<I2C1, (BScl, BSda)>>,
) -> Ssd1306<DisplayIface, DisplaySize128x64, TerminalMode> {
    let interface = I2CDisplayInterface::new(i2c_sbus.acquire());

    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();
    display
}

pub fn init_timer_int(timer: &mut Counter<TIM3, 1000>) {
    // ======================= init interrupts of timer ==============================//
    // Configure the syst timer to trigger an update every second
    // let mut sys_timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(100.millis()).unwrap();

    // Set up to generate interrupt when timer expires
    timer.listen(Event::Update);

    // Enable the external interrupt in the NVIC for all peripherals by passing the interrupt numbers
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
    }
}
