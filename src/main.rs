// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::fmt::Write;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use mpu6050::*;
use panic_halt as _;
use shared_bus_rtic::SharedBus;
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    gpio::{self, Alternate, OpenDrain, Output, Pin, PushPull},
    i2c,
    pac::{self, interrupt, TIM3},
    prelude::*,
    timer::{Channel, CounterMs, Event, Tim2NoRemap, Timer2},
};

type LedPin = gpio::PC13<Output<PushPull>>;
type BlockingI2cPB89 = i2c::BlockingI2c<
    pac::I2C1,
    (
        Pin<'B', 8, Alternate<OpenDrain>>,
        Pin<'B', 9, Alternate<OpenDrain>>,
    ),
>;
type I2cDisplay =
    Ssd1306<I2CInterface<SharedBus<BlockingI2cPB89>>, DisplaySize128x64, TerminalMode>;
type I2cMpu6050 = Mpu6050<SharedBus<BlockingI2cPB89>>;

// Create a Global Variable for the Timer Peripheral that I'm going to pass around.
static G_TIM: Mutex<RefCell<Option<CounterMs<TIM3>>>> = Mutex::new(RefCell::new(None));
// Create a Global Variable for the LED GPIO Peripheral that I'm going to pass around.
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

static G_DISP: Mutex<RefCell<Option<I2cDisplay>>> = Mutex::new(RefCell::new(None));
static G_MPU: Mutex<RefCell<Option<I2cMpu6050>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    // let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();
    let mut afio = dp.AFIO.constrain();

    // ======================= init interrupts of timer ==============================//
    // Take ownership over the raw flash and rcc devices and convert them into the corresponding HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Configure the syst timer to trigger an update every second
    // let mut sys_timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    let dev_timer = dp.TIM3;
    let mut timer = dev_timer.counter_ms(&clocks);
    timer.start(500.millis()).unwrap();

    // Set up to generate interrupt when timer expires
    timer.listen(Event::Update);

    // Enable the external interrupt in the NVIC for all peripherals by passing the interrupt numbers
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
    }

    // ======================= init led pin ========================================//
    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // ======================= init pwm pin ========================================//
    let mut gpioa = dp.GPIOA.split();
    let pina0_pwm = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2 = Timer2::new(dp.TIM2, &clocks).pwm_hz::<Tim2NoRemap, _, _>(pina0_pwm, &mut afio.mapr, 100.Hz());
    let max = pwm2.get_max_duty();
    pwm2.set_duty(Channel::C1, max / 8);
    pwm2.enable(Channel::C1);

    // ======================= init i2c over pb8/pb9 as scl/sda ====================//
    let mut gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c_2 = i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        i2c::Mode::Standard {
            frequency: 100_000.Hz(),
        },
        clocks,
        // below are different timeouts
        1000,
        10,
        1000,
        1000,
    );

    // ======================= init i2c over pb8/pb9 as scl/sda ====================//
    let i2c_sbus = shared_bus_rtic::new!(i2c_2, BlockingI2cPB89);
    let interface = I2CDisplayInterface::new(i2c_sbus.acquire());

    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear().unwrap();

    // ======================= init mpu6050 over i2c ====================//
    let mut mpu = Mpu6050::new(i2c_sbus.acquire());
    let mut delay = dp.TIM1.delay_ms(&clocks);
    mpu.init(&mut delay).unwrap();

    // ======================= init global var to use inside interrupt handler ====================//
    cortex_m::interrupt::free(|cs| {
        // G_I2C2.borrow(cs).replace(Some(i2c_2));
        G_DISP.borrow(cs).replace(Some(display));
        G_MPU.borrow(cs).replace(Some(mpu));
        G_TIM.borrow(cs).replace(Some(timer));
        G_LED.borrow(cs).replace(Some(led));
    });

    #[allow(clippy::empty_loop)]
    loop {
        // Go to sleep
        cortex_m::asm::wfi();
        // delay.delay_ms(2000_u32);
    }
}

#[interrupt]
fn TIM3() {
    // When Timer Interrupt Happens Two Things Need to be Done
    // 1) Toggle the LED
    // 2) Clear Timer Pending Interrupt

    // Start a Critical Section to work with global vars
    cortex_m::interrupt::free(|cs| {
        // Obtain Access to Delay Global Data and Adjust Delay
        let mut led = G_LED.borrow(cs).borrow_mut();
        led.as_mut().unwrap().toggle();

        let mut txt = heapless::String::<16>::new();
        let mut angle_x = heapless::String::<16>::new();
        // let mut angle_y = heapless::String::<16>::new();
        // let mut angle_z = heapless::String::<16>::new();

        let mut mpu_ref = G_MPU.borrow(cs).borrow_mut();
        let mut display = G_DISP.borrow(cs).borrow_mut();
        let mpu = mpu_ref.as_mut().unwrap();

        let temp = mpu.get_temp().unwrap();
        write!(&mut txt, "Temp: {:.2}", temp).unwrap();
        let d = display.as_mut().unwrap();
        d.set_position(0, 0).unwrap();
        d.write_str(&txt).unwrap();

        // gyro: x, y, z
        let gyro = mpu.get_acc_angles().unwrap();

        write!(&mut angle_x, "Angle X: {:.2}", gyro.x).unwrap();
        d.set_position(0, 1).unwrap();
        d.write_str(&angle_x).unwrap();

        // write!(&mut angle_y, "Angle Y: {:.2}", gyro.y).unwrap();
        // d.set_position(0, 2).unwrap();
        // d.write_str(&angle_y).unwrap();

        // write!(&mut angle_z, "Angle Z: {:.2}", gyro.z).unwrap();
        // d.set_position(0, 3).unwrap();
        // d.write_str(&angle_z).unwrap();

        // Obtain access to Global Timer Peripheral and Clear Interrupt Pending Flag
        let mut timer = G_TIM.borrow(cs).borrow_mut();
        timer.as_mut().unwrap().clear_interrupt(Event::Update);
    });
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
