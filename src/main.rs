// #![deny(unsafe_code)]
#![no_main]
#![no_std]

pub mod aux;
pub mod pid;

use aux::{init_control_timer, init_devices, init_display, init_i2c, init_timer_int};
use heapless::String;
use pid::{BalanceController, MAX_SAFE_ANGLE};

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
    pac::{self, interrupt, TIM3, TIM4},
    prelude::*,
    timer::{Channel, CounterMs, Event, PwmChannel, Tim2NoRemap, Timer2},
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
type Pwm2Channel = PwmChannel<pac::TIM2, 0>;

// Display update timer (TIM3 at 100ms)
static G_TIM: Mutex<RefCell<Option<CounterMs<TIM3>>>> = Mutex::new(RefCell::new(None));
// Control loop timer (TIM4 at 10ms = 100Hz)
static G_CONTROL_TIM: Mutex<RefCell<Option<CounterMs<TIM4>>>> = Mutex::new(RefCell::new(None));

// GPIO peripherals
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static G_BREAK: Mutex<RefCell<Option<BreakPin>>> = Mutex::new(RefCell::new(None));
static G_DIR: Mutex<RefCell<Option<DirPin>>> = Mutex::new(RefCell::new(None));

// I2C peripherals
static G_DISP: Mutex<RefCell<Option<I2cDisplay>>> = Mutex::new(RefCell::new(None));
static G_MPU: Mutex<RefCell<Option<I2cMpu6050>>> = Mutex::new(RefCell::new(None));

// PWM channel for motor control
static G_PWM_CH: Mutex<RefCell<Option<Pwm2Channel>>> = Mutex::new(RefCell::new(None));

// PID controller
static G_CONTROLLER: Mutex<RefCell<Option<BalanceController>>> = Mutex::new(RefCell::new(None));

// Telemetry values (set in TIM4, read in TIM3 for display)
static G_PWM_VAL: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_ANGLE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_RAW_ANGLE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_GYRO_RATE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));

#[entry]
fn main() -> ! {
    let (
        mut afio,
        clocks,
        mut timer,
        mut gpioa,
        mut gpiob,
        mut gpioc,
        dp_i2c1,
        dp_tim1,
        dp_tim2,
        dp_tim4,
    ) = init_devices();

    // ======================= init led pin ========================================//
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // ======================= init break/dir pin ========================================//
    let mut p_break = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    m_stop(&mut p_break); // Start with motor stopped

    let p_dir = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

    // ======================= init pwm pin ========================================//
    let pina0_pwm = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2 = Timer2::new(dp_tim2, &clocks).pwm_hz::<Tim2NoRemap, _, _>(
        pina0_pwm,
        &mut afio.mapr,
        20.kHz(),
    );
    let max_duty = pwm2.get_max_duty();
    pwm2.set_duty(Channel::C1, 0); // Start with 0 duty
    pwm2.enable(Channel::C1);

    // Split the PWM into channels to get the channel handle
    let pwm_ch = pwm2.split();

    // ======================= init i2c over pb8/pb9 as scl/sda ====================//
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c_sbus = init_i2c(scl, sda, dp_i2c1, &mut afio.mapr, clocks);

    // ======================= init i2c display ====================//
    let mut display = init_display(i2c_sbus);

    let mut txt = String::<16>::new();
    display.set_position(0, 7).unwrap();
    write!(&mut txt, "PID max:{}", max_duty).unwrap();
    display.write_str(&txt).unwrap();

    // ======================= init mpu6050 over i2c ====================//
    let mut mpu = Mpu6050::new(i2c_sbus.acquire());
    let mut delay = dp_tim1.delay_ms(&clocks);
    mpu.init(&mut delay).unwrap();
    mpu.set_accel_hpf(device::ACCEL_HPF::_1P25).unwrap();

    // ======================= init PID controller ====================//
    let controller = BalanceController::new();

    // ======================= init timers ====================//
    // TIM3: Display update at 100ms
    init_timer_int(&mut timer, 10);
    // TIM4: Control loop at 10ms (100Hz)
    let control_timer = init_control_timer(dp_tim4, &clocks);

    // ======================= store globals ====================//
    cortex_m::interrupt::free(|cs| {
        G_DISP.borrow(cs).replace(Some(display));
        G_MPU.borrow(cs).replace(Some(mpu));
        G_TIM.borrow(cs).replace(Some(timer));
        G_CONTROL_TIM.borrow(cs).replace(Some(control_timer));
        G_LED.borrow(cs).replace(Some(led));
        G_BREAK.borrow(cs).replace(Some(p_break));
        G_DIR.borrow(cs).replace(Some(p_dir));
        G_PWM_CH.borrow(cs).replace(Some(pwm_ch));
        G_CONTROLLER.borrow(cs).replace(Some(controller));
    });

    // Main loop - just sleep, control happens in TIM4 interrupt
    #[allow(clippy::empty_loop)]
    loop {
        cortex_m::asm::wfi();
    }
}

/// TIM4 Interrupt: PID Control Loop at 100Hz
#[interrupt]
fn TIM4() {
    cortex_m::interrupt::free(|cs| {
        // Get timer and clear interrupt
        let mut timer_ref = G_CONTROL_TIM.borrow(cs).borrow_mut();
        let timer = timer_ref.deref_mut().as_mut().unwrap();

        // Get peripherals
        let mut mpu_ref = G_MPU.borrow(cs).borrow_mut();
        let mpu = mpu_ref.deref_mut().as_mut().unwrap();

        let mut controller_ref = G_CONTROLLER.borrow(cs).borrow_mut();
        let controller = controller_ref.deref_mut().as_mut().unwrap();

        let mut pwm_ref = G_PWM_CH.borrow(cs).borrow_mut();
        let pwm = pwm_ref.deref_mut().as_mut().unwrap();

        let mut dir_ref = G_DIR.borrow(cs).borrow_mut();
        let dir = dir_ref.deref_mut().as_mut().unwrap();

        let mut brake_ref = G_BREAK.borrow(cs).borrow_mut();
        let brake = brake_ref.deref_mut().as_mut().unwrap();

        let mut led_ref = G_LED.borrow(cs).borrow_mut();
        let led = led_ref.deref_mut().as_mut().unwrap();

        // Read sensor data
        // Note: Using X axis for balance angle - adjust if your MPU orientation differs
        let acc_ang = mpu.get_acc_angles().unwrap();
        let gyro = mpu.get_gyro().unwrap();

        // Update angle estimate and compute PID output
        // acc_ang.x is in radians from mpu6050 crate, gyro.x is in deg/s
        let accel_angle_deg = acc_ang.x * 57.3; // Convert to degrees
        let (angle, output) = controller.update(accel_angle_deg, gyro.x);

        // Store all values for display (TIM3 reads these, no separate I2C needed)
        *G_PWM_VAL.borrow(cs).borrow_mut() = output;
        *G_ANGLE.borrow(cs).borrow_mut() = angle;
        *G_RAW_ANGLE.borrow(cs).borrow_mut() = accel_angle_deg;
        *G_GYRO_RATE.borrow(cs).borrow_mut() = gyro.x;

        // Safety check - stop if robot has fallen
        if !controller.is_safe(MAX_SAFE_ANGLE) {
            m_stop(brake);
            controller.reset();
            led.set_high(); // LED on = fallen
            timer.clear_interrupt(Event::Update);
            return;
        }

        // Apply motor output
        apply_motor_output(pwm, dir, brake, led, output);

        timer.clear_interrupt(Event::Update);
    });
}

/// TIM3 Interrupt: Display Update at 10Hz
/// Only reads globals set by TIM4 — no I2C access needed here.
#[interrupt]
fn TIM3() {
    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = G_TIM.borrow(cs).borrow_mut();
        let timer = timer_ref.deref_mut().as_mut().unwrap();

        let mut display_ref = G_DISP.borrow(cs).borrow_mut();
        let d = display_ref.deref_mut().as_mut().unwrap();

        // All values come from TIM4's last cycle — consistent snapshot
        let fused_angle = *G_ANGLE.borrow(cs).borrow();
        let raw_angle = *G_RAW_ANGLE.borrow(cs).borrow();
        let gyro_rate = *G_GYRO_RATE.borrow(cs).borrow();
        let pwm_val = *G_PWM_VAL.borrow(cs).borrow();

        let mut line = String::<16>::new();
        write!(&mut line, "Ang: {:.1}", fused_angle).unwrap();
        d.set_position(0, 0).unwrap();
        d.write_str(&line).unwrap();

        line.clear();
        write!(&mut line, "Raw: {:.1}", raw_angle).unwrap();
        d.set_position(0, 1).unwrap();
        d.write_str(&line).unwrap();

        line.clear();
        write!(&mut line, "Gyr: {:.1}", gyro_rate).unwrap();
        d.set_position(0, 2).unwrap();
        d.write_str(&line).unwrap();

        line.clear();
        write!(&mut line, "PWM: {:.0}", pwm_val).unwrap();
        d.set_position(0, 3).unwrap();
        d.write_str(&line).unwrap();

        let controller_ref = G_CONTROLLER.borrow(cs).borrow();
        if let Some(ctrl) = controller_ref.as_ref() {
            line.clear();
            write!(&mut line, "I: {:.1}", ctrl.state.integral).unwrap();
            d.set_position(0, 4).unwrap();
            d.write_str(&line).unwrap();

            line.clear();
            write!(
                &mut line,
                "Kp:{:.0} Ki:{:.1}",
                ctrl.config.kp, ctrl.config.ki
            )
            .unwrap();
            d.set_position(0, 5).unwrap();
            d.write_str(&line).unwrap();
        }

        timer.clear_interrupt(Event::Update);
    });
}

/// Apply PID output to motor hardware
fn apply_motor_output(
    pwm: &mut Pwm2Channel,
    dir: &mut DirPin,
    brake: &mut BreakPin,
    led: &mut LedPin,
    output: f32,
) {
    // Get max duty cycle from PWM
    let max_duty = pwm.get_max_duty();

    // Set direction based on output sign
    if output >= 0.0 {
        dir.set_low();
        led.set_low();
    } else {
        dir.set_high();
        led.set_high();
    }

    // Map output (-255..255) to duty (0..max_duty)
    let duty_fraction = output.abs() / 255.0;
    let mut duty = (duty_fraction * max_duty as f32) as u16;

    // Minimum duty threshold - motor won't move below certain PWM
    const MIN_DUTY_THRESHOLD: u16 = 50;
    if duty < MIN_DUTY_THRESHOLD && duty > 0 {
        duty = MIN_DUTY_THRESHOLD;
    }

    // Apply duty cycle and release brake
    brake.set_low();
    pwm.set_duty(duty);
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
