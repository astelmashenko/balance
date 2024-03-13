// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::fmt::Write;

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

use stm32f1xx_hal::{i2c, pac, prelude::*, timer::Timer};

// use embedded_alloc::Heap;
//
// #[global_allocator]
// static HEAP: Heap = Heap::empty();
//
// #[macro_use]
// extern crate alloc;
//
#[entry]
fn main() -> ! {
    // // Initialize the allocator BEFORE you use it
    // {
    //     use core::mem::MaybeUninit;
    //     const HEAP_SIZE: usize = 1024;
    //     static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    //     unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    // }
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

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // Acquire the GPIOC peripheral
    // let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    // let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // let mut led2 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    //
    // let device_periphs = stm32::Peripherals::take().unwrap();
    // let mut gpioc = device_periphs.GPIOC.split();

    let mut afio = dp.AFIO.constrain();

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    // let i2c2_pins = (
    //     gpiob.pb6.into_alternate_open_drain(&mut gpiob.crh),
    //     gpiob.pb7.into_alternate_open_drain(&mut gpiob.crh),
    // );

    let i2c = i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        // 100.kHz(),
        i2c::Mode::Standard {
            frequency: 100_000.Hz(),
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    // let i2c = i2c::I2c::i2c2(dp.I2C2, i2c2_pins, 100.kHz(), &mut rcc);
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    // Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    // Text::with_baseline("Hello rust!", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    // Text::with_baseline("Hello rust2!", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    // display.flush().unwrap();

    let mut i = 0;

    let mut txt = heapless::String::<16>::new();

    // Wait for the timer to trigger an update and change the state of the LED
    #[allow(clippy::empty_loop)]
    loop {
        block!(timer.wait()).unwrap();
        i += 1;
        write!(&mut txt, "Hello: {}", i).unwrap();
        // led.set_high();
        display.clear_buffer();

        // display.reset(, delay)
        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&txt, Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
        // //
        // block!(timer.wait()).unwrap();
        // // led.set_high();
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
