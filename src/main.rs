// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::{entry, exception, ExceptionFrame};
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use nb::block;
use panic_halt as _;
// use panic_itm as _;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306}; // panic handler
                                                         // use stm32f1::stm32f103;

use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    timer::Timer,
};

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
    // let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    // let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // let mut led2 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();
    //
    // let device_periphs = stm32::Peripherals::take().unwrap();
    // let mut gpioc = device_periphs.GPIOC.split();

    let mut afio = dp.AFIO.constrain();

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    // let i2c_pins = cortex_m::interrupt::free(move |cs| {
    //     (
    //         gpiob.pb6.into_alternate_af1(cs),
    //         gpiob.pb7.into_alternate_af1(cs),
    //     )
    // });

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        100.kHz(),
        // Mode::Fast {
        //     frequency: 400_000.Hz(),
        //     duty_cycle: DutyCycle::Ratio2to1,
        // },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    // let i2c = i2c::I2c::i2c1(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        // block!(timer.wait()).unwrap();
        // led.set_high();
        // led2.set_low();
        // block!(timer.wait()).unwrap();
        // led.set_low();
        // led2.set_high();
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
