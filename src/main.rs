// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::fmt::Write;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use mpu6050::*;
use nb::block;
use panic_halt as _;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{i2c, pac, prelude::*, timer::Timer};



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

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // let mut led2 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second

    let mut afio = dp.AFIO.constrain();

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

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

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let interface = I2CDisplayInterface::new(bus.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_terminal_mode();
        // .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear().unwrap();

    let mut mpu = Mpu6050::new(bus.acquire_i2c());
    let mut delay = dp.TIM2.delay_us(&clocks);
    mpu.init(&mut delay).unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    #[allow(clippy::empty_loop)]
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();


        let mut txt = heapless::String::<16>::new();
        let mut angle_x = heapless::String::<16>::new();
        let mut angle_y = heapless::String::<16>::new();
        let mut angle_z = heapless::String::<16>::new();

        let temp = mpu.get_temp().unwrap();
        write!(&mut txt, "Temp: {:.2}", temp).unwrap();
        display.set_position(0, 0).unwrap();
        display.write_str(&txt).unwrap();
     
        // gyro: x, y, z
        let gyro = mpu.get_gyro().unwrap();

        write!(&mut angle_x, "Angle X: {:.2}", gyro.x).unwrap();
        display.set_position(0, 1).unwrap();
        display.write_str(&angle_x).unwrap();

        write!(&mut angle_y, "Angle Y: {:.2}", gyro.y).unwrap();
        display.set_position(0, 2).unwrap();
        display.write_str(&angle_y).unwrap();

        write!(&mut angle_z, "Angle Z: {:.2}", gyro.z).unwrap();
        display.set_position(0, 3).unwrap();
        display.write_str(&angle_z).unwrap();

        //
        block!(timer.wait()).unwrap();
        led.set_low();
    }
}

// fn show(
//     display: &Ssd1306<I2CInterface<shared_bus::I2cProxy<'_, shared_bus::NullMutex<i2c::BlockingI2c<pac::I2C1, (stm32f1xx_hal::gpio::Pin<'B', 8, stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::Pin<'B', 9, stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>>>>, DisplaySize128x64, ssd1306::mode::TerminalMode>,
//     pos: u8,
//     txt: &mut heapless::String<16>,
//     temp: f32) {
//     write!(txt, "Temp: {:.2}", temp).unwrap();
//     display.set_position(0, 0).unwrap();
//     display.write_str(&txt).unwrap();
// }

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
