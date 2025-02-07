// #![deny(unsafe_code)]
#![no_main]
#![no_std]

pub mod aux;
pub mod pid;

use aux::{init_devices, init_display, init_i2c, init_timer_int};
use heapless::String;
use pid::{pid1, ANGLE_BIAS};

use core::{fmt::Write, ops::DerefMut};
use cortex_m_rt::{entry, exception, ExceptionFrame};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use mpu6050::*;
use panic_halt as _;
use shared_bus_rtic::SharedBus;
use ssd1306::{mode::TerminalMode, prelude::*, Ssd1306};
use stm32f1xx_hal::{
    gpio::{self, Alternate, OpenDrain, Output, Pin, PushPull},
    i2c::{self},
    pac::{self, interrupt, TIM3},
    prelude::*,
    timer::{Channel, CounterMs, Event, Tim2NoRemap, Timer2},
};

type LedPin = gpio::PC13<Output<PushPull>>;
type BreakPin = gpio::PA8<Output<PushPull>>;
type DirPin = gpio::PA2<Output<PushPull>>;
type BScl = Pin<'B', 8, Alternate<OpenDrain>>;
type BSda = Pin<'B', 9, Alternate<OpenDrain>>;
type BlockingI2cPB89 = i2c::BlockingI2c<pac::I2C1, (BScl, BSda)>;
type I2cDisplay =
    Ssd1306<I2CInterface<SharedBus<BlockingI2cPB89>>, DisplaySize128x64, TerminalMode>;
type I2cMpu6050 = Mpu6050<SharedBus<BlockingI2cPB89>>;

// Create a Global Variable for the Timer Peripheral that I'm going to pass around.
static G_TIM: Mutex<RefCell<Option<CounterMs<TIM3>>>> = Mutex::new(RefCell::new(None));
// Create a Global Variable for the LED GPIO Peripheral that I'm going to pass around.
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static G_BREAK: Mutex<RefCell<Option<BreakPin>>> = Mutex::new(RefCell::new(None));
static G_DIR: Mutex<RefCell<Option<DirPin>>> = Mutex::new(RefCell::new(None));

static G_DISP: Mutex<RefCell<Option<I2cDisplay>>> = Mutex::new(RefCell::new(None));
static G_MPU: Mutex<RefCell<Option<I2cMpu6050>>> = Mutex::new(RefCell::new(None));

static G_PWM: Mutex<RefCell<Option<i32>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let (mut afio, clocks, mut timer, mut gpioa, mut gpiob, mut gpioc, dp_i2c1, dp_tim1, dp_tim2) =
        init_devices();

    // ======================= init led pin ========================================//
    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // ======================= init break/dir pin ========================================//
    let mut p_break = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    m_start(&mut p_break);

    let mut p_dir = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    p_dir.set_low();

    // let mut p_enc_en = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    // p_enc_en.set_low();
    // ======================= init pwm pin ========================================//
    let pina0_pwm = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2 = Timer2::new(dp_tim2, &clocks).pwm_hz::<Tim2NoRemap, _, _>(
        pina0_pwm,
        &mut afio.mapr,
        20.kHz(),
    );
    let max = pwm2.get_max_duty();
    let duty = 400; //585
    pwm2.set_duty(Channel::C1, duty);
    pwm2.enable(Channel::C1);

    // ======================= init i2c over pb8/pb9 as scl/sda ====================//
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c_sbus = init_i2c(scl, sda, dp_i2c1, &mut afio.mapr, clocks);

    // ======================= init i2c display ====================//
    let mut display = init_display(i2c_sbus);

    let mut txt = String::<16>::new();
    display.set_position(0, 7).unwrap();
    write!(&mut txt, "max:{max};d:{duty}").unwrap();
    display.write_str(&txt).unwrap();

    // ======================= init mpu6050 over i2c ====================//
    let mut mpu = Mpu6050::new(i2c_sbus.acquire());
    let mut delay = dp_tim1.delay_ms(&clocks);
    mpu.init(&mut delay).unwrap();
    mpu.set_accel_hpf(device::ACCEL_HPF::_1P25).unwrap();

    init_timer_int(&mut timer, 100);

    // ======================= init global var to use inside interrupt handler ====================//
    cortex_m::interrupt::free(|cs| {
        // G_I2C2.borrow(cs).replace(Some(i2c_2));
        G_DISP.borrow(cs).replace(Some(display));
        G_MPU.borrow(cs).replace(Some(mpu));
        G_TIM.borrow(cs).replace(Some(timer));
        G_LED.borrow(cs).replace(Some(led));
        G_BREAK.borrow(cs).replace(Some(p_break));
        G_DIR.borrow(cs).replace(Some(p_dir));
    });

    let mut m_speed: i32 = 0;
    let mut gz_filt: f32 = 0.0;

    #[allow(clippy::empty_loop)]
    loop {
        // Go to sleep
        // cortex_m::asm::wfi();
        delay.delay_ms(50_u32);

        cortex_m::interrupt::free(|cs| {
            let mut p_break_ref = G_BREAK.borrow(cs).borrow_mut();
            let p_break = p_break_ref.deref_mut().as_mut().unwrap();
            let mut led_ref = G_LED.borrow(cs).borrow_mut();
            let led = led_ref.deref_mut().as_mut().unwrap();

            let mut p_dir_ref = G_DIR.borrow(cs).borrow_mut();
            let p_dir = p_dir_ref.deref_mut().as_mut().unwrap();

            let mut mpu_ref = G_MPU.borrow(cs).borrow_mut();
            let mpu = mpu_ref.deref_mut().as_mut().unwrap();
            let gyro = mpu.get_gyro().unwrap();

            // gyro: x, y  https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
            let acc_ang = mpu.get_acc_angles().unwrap();
            // gyro accelerometer as internal mcu value
            // let acc = mpu.get_acc().unwrap();
            let (pwm, m_speed_o, gz_filt_o) = pid1(acc_ang.x, gz_filt, gyro.z, m_speed);
            m_speed = m_speed_o;
            gz_filt = gz_filt_o;

            G_PWM.borrow(cs).replace(Some(pwm));

            if acc_ang.x < 0.0 {
                m_stop(p_break);
                p_dir.set_high();
                led.set_high();
                m_start(p_break);
            } else {
                m_stop(p_break);
                p_dir.set_low();
                led.set_low();
                m_start(p_break);
            }
        });
    }
}

#[interrupt]
fn TIM3() {
    // Start a Critical Section to work with global vars
    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = G_TIM.borrow(cs).borrow_mut();
        let timer = timer_ref.deref_mut().as_mut().unwrap();

        let mut acc_x = String::<16>::new();
        let mut angle_x = String::<16>::new();
        let mut angle_y = String::<16>::new();
        let mut acc_y = String::<16>::new();
        let mut acc_z = String::<16>::new();

        let mut gyro_x = String::<16>::new();
        let mut gyro_y = String::<16>::new();

        let mut mpu_ref = G_MPU.borrow(cs).borrow_mut();
        let mpu = mpu_ref.deref_mut().as_mut().unwrap();

        let mut display = G_DISP.borrow(cs).borrow_mut();
        let d = display.deref_mut().as_mut().unwrap();

        // gyro accelerometer as internal mcu value
        let acc = mpu.get_acc().unwrap();
        // gyro: x, y  https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
        let acc_ang = mpu.get_acc_angles().unwrap();
        // let acc_ang = angle_accel(acc);
        let gyro = mpu.get_gyro().unwrap();

        let mut pwm_ref = G_PWM.borrow(cs).borrow_mut();
        let pwm = pwm_ref.as_mut().unwrap();

        write!(&mut angle_x, "AngX: {:.3}", acc_ang.x * ANGLE_BIAS).unwrap();
        d.set_position(0, 0).unwrap();
        d.write_str(&angle_x).unwrap();

        write!(&mut angle_y, "AngY: {:.3}", acc_ang.y * ANGLE_BIAS).unwrap();
        d.set_position(0, 1).unwrap();
        d.write_str(&angle_y).unwrap();

        write!(&mut acc_x, "AccX: {:.3}", acc.x).unwrap();
        d.set_position(0, 2).unwrap();
        d.write_str(&acc_x).unwrap();

        write!(&mut acc_y, "AccY: {:.3}", acc.y).unwrap();
        d.set_position(0, 3).unwrap();
        d.write_str(&acc_y).unwrap();

        write!(&mut acc_z, "AccZ: {:.3}", acc.z).unwrap();
        d.set_position(0, 4).unwrap();
        d.write_str(&acc_z).unwrap();

        write!(&mut gyro_x, "Pwm: {pwm}").unwrap();
        d.set_position(0, 5).unwrap();
        d.write_str(&gyro_x).unwrap();

        write!(&mut gyro_y, "Gyro Z: {:.3}", gyro.z).unwrap();
        d.set_position(0, 6).unwrap();
        d.write_str(&gyro_y).unwrap();

        // Obtain access to Global Timer Peripheral and Clear Interrupt Pending Flag
        timer.clear_interrupt(Event::Update);
    });
}

pub fn m_start(p_break: &mut BreakPin) {
    p_break.set_low();
}

pub fn m_stop(p_break: &mut BreakPin) {
    p_break.set_high();
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
