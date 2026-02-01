// #![deny(unsafe_code)]
#![no_main]
#![no_std]

pub mod aux;
pub mod pid;

use aux::{
    init_control_timer, init_devices, init_display, init_encoder, init_i2c, init_timer_int, QeiType,
};
use heapless::String;
use pid::{Controller, MAX_SAFE_ANGLE};

use core::{fmt::Write, ops::DerefMut};
use cortex_m_rt::{entry, exception, ExceptionFrame};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use mpu6050::{device, Mpu6050};
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
type BreakPin = gpio::PA3<Output<PushPull>>;
type DirPin = gpio::PA2<Output<PushPull>>;
type BScl = Pin<'B', 8, Alternate<OpenDrain>>;
type BSda = Pin<'B', 9, Alternate<OpenDrain>>;
type BlockingI2cPB89 = i2c::BlockingI2c<pac::I2C1, (BScl, BSda)>;
type I2cDisplay =
    Ssd1306<I2CInterface<SharedBus<BlockingI2cPB89>>, DisplaySize128x64, TerminalMode>;
type I2cMpu6050 = Mpu6050<SharedBus<BlockingI2cPB89>>;
type Pwm2Channel = PwmChannel<pac::TIM2, 0>;

// Display update timer (TIM3)
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

// Controller
static G_CTRL: Mutex<RefCell<Option<Controller>>> = Mutex::new(RefCell::new(None));

// Encoder QEI
static G_QEI: Mutex<RefCell<Option<QeiType>>> = Mutex::new(RefCell::new(None));

// Telemetry (set in TIM4, read in TIM3 — all from same control cycle)
static G_ANGLE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_GYRO: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_PWM_OUT: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(0.0));
static G_ENCODER: Mutex<RefCell<i16>> = Mutex::new(RefCell::new(0));

/// Default center of gravity angle in degrees
/// Original default: 88.9 — adjust for your hardware
const CENTER_GRAVITY_DEFAULT: f32 = 89.9;

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

    // ======================= init break/dir pin ==================================//
    let mut p_break = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    m_stop(&mut p_break); // Start with motor stopped

    let p_dir = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

    // ======================= init pwm pin ========================================//
    let pina0_pwm = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2 = Timer2::new(dp_tim2, &clocks).pwm_hz::<Tim2NoRemap, _, _>(
        pina0_pwm,
        &mut afio.mapr,
        10.kHz(), // 10 kHz matching original
    );
    pwm2.set_duty(Channel::C1, 0);
    pwm2.enable(Channel::C1);

    let pwm_ch = pwm2.split();

    // ======================= init i2c ===========================================//
    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
    let i2c_sbus = init_i2c(scl, sda, dp_i2c1, &mut afio.mapr, clocks);

    // ======================= init display =======================================//
    let display = init_display(i2c_sbus);

    // ======================= init MPU6500 =======================================//
    let mut mpu = Mpu6050::new(i2c_sbus.acquire());
    let mut delay = dp_tim1.delay_ms(&clocks);
    mpu.init(&mut delay).unwrap();
    // Configure to match original: ±4g accel, ±2000dps gyro
    mpu.set_accel_range(device::AccelRange::G4).unwrap();
    mpu.set_gyro_range(device::GyroRange::D2000).unwrap();

    // ======================= init encoder (TIM1 QEI on PA8/PA9) ================//
    let mut pina1_enc_ctrl = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    pina1_enc_ctrl.set_high();
    // Release TIM1 from delay use, then configure as quadrature encoder
    let tim1 = delay.release().release();
    let enc_a = gpioa.pa8; // PA8 default mode is Input<Floating>
    let enc_b = gpioa.pa9; // PA9 default mode is Input<Floating>
    let qei = init_encoder(tim1, enc_a, enc_b, &mut afio.mapr, &clocks);

    // ======================= init controller ====================================//
    // Seed filter angle from accelerometer reading
    let acc = mpu.get_acc().unwrap();
    let initial_angle = pid::accel_angle(acc.z, acc.y);

    let mut controller = Controller::new(CENTER_GRAVITY_DEFAULT);
    controller.filter.angle = initial_angle;

    // ======================= init timers ========================================//
    // TIM3: Display update
    init_timer_int(&mut timer, 100);
    // TIM4: Control loop at 10ms (100Hz) matching original
    let control_timer = init_control_timer(dp_tim4, &clocks);

    // ======================= store globals ======================================//
    cortex_m::interrupt::free(|cs| {
        G_DISP.borrow(cs).replace(Some(display));
        G_MPU.borrow(cs).replace(Some(mpu));
        G_TIM.borrow(cs).replace(Some(timer));
        G_CONTROL_TIM.borrow(cs).replace(Some(control_timer));
        G_LED.borrow(cs).replace(Some(led));
        G_BREAK.borrow(cs).replace(Some(p_break));
        G_DIR.borrow(cs).replace(Some(p_dir));
        G_PWM_CH.borrow(cs).replace(Some(pwm_ch));
        G_CTRL.borrow(cs).replace(Some(controller));
        G_QEI.borrow(cs).replace(Some(qei));
    });

    // Main loop — control runs in TIM4 interrupt
    #[allow(clippy::empty_loop)]
    loop {
        cortex_m::asm::wfi();
    }
}

/// TIM4 Interrupt: Control Loop at 100Hz
/// Matches original TIM1_UP_IRQHandler
#[interrupt]
fn TIM4() {
    use core::sync::atomic::{AtomicU16, Ordering};
    // Track previous encoder count to compute delta each cycle
    static LAST_ENC: AtomicU16 = AtomicU16::new(0);

    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = G_CONTROL_TIM.borrow(cs).borrow_mut();
        let timer = timer_ref.deref_mut().as_mut().unwrap();

        let mut mpu_ref = G_MPU.borrow(cs).borrow_mut();
        let mpu = mpu_ref.deref_mut().as_mut().unwrap();

        let mut ctrl_ref = G_CTRL.borrow(cs).borrow_mut();
        let ctrl = ctrl_ref.deref_mut().as_mut().unwrap();

        let mut pwm_ref = G_PWM_CH.borrow(cs).borrow_mut();
        let pwm = pwm_ref.deref_mut().as_mut().unwrap();

        let mut dir_ref = G_DIR.borrow(cs).borrow_mut();
        let dir = dir_ref.deref_mut().as_mut().unwrap();

        let mut brake_ref = G_BREAK.borrow(cs).borrow_mut();
        let brake = brake_ref.deref_mut().as_mut().unwrap();

        let mut led_ref = G_LED.borrow(cs).borrow_mut();
        let led = led_ref.deref_mut().as_mut().unwrap();

        // Read encoder: compute delta since last sample (wrapping subtraction)
        let raw_count = unsafe { (*pac::TIM1::ptr()).cnt.read().cnt().bits() };
        let delta = raw_count.wrapping_sub(LAST_ENC.load(Ordering::Relaxed)) as i16;
        LAST_ENC.store(raw_count, Ordering::Relaxed);
        let encoder = delta; // Negate to match original: Encoder_x = -Read_Encoder(2)

        // Read raw sensor data
        // get_acc() returns g-scaled values, get_gyro() returns deg/s-scaled values
        let acc = mpu.get_acc().unwrap();
        let gyro = mpu.get_gyro().unwrap();

        // Controller update: computes angle, filters, PD + velocity PI
        let (angle, total_pwm) = ctrl.update(acc.z, acc.y, gyro.x, encoder);

        // Store telemetry for display
        *G_ANGLE.borrow(cs).borrow_mut() = angle;
        *G_GYRO.borrow(cs).borrow_mut() = ctrl.gyro_raw;
        *G_PWM_OUT.borrow(cs).borrow_mut() = total_pwm;
        *G_ENCODER.borrow(cs).borrow_mut() = encoder;

        // Safety: stop if fallen
        if angle.abs() - ctrl.balance.center_gravity > MAX_SAFE_ANGLE {
            m_stop(brake);
            pwm.set_duty(0);
            ctrl.reset();
            led.set_high();
            timer.clear_interrupt(Event::Update);
            return;
        }

        // Apply motor output (balance + velocity combined)
        let max_duty = pwm.get_max_duty();
        apply_motor(pwm, dir, brake, led, total_pwm, max_duty);

        timer.clear_interrupt(Event::Update);
    });
}

/// TIM3 Interrupt: Display Update
/// Matches original show.c display layout
#[interrupt]
fn TIM3() {
    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = G_TIM.borrow(cs).borrow_mut();
        let timer = timer_ref.deref_mut().as_mut().unwrap();

        let mut display_ref = G_DISP.borrow(cs).borrow_mut();
        let d = display_ref.deref_mut().as_mut().unwrap();

        let angle = *G_ANGLE.borrow(cs).borrow();
        let gyro = *G_GYRO.borrow(cs).borrow();
        let pwm_out = *G_PWM_OUT.borrow(cs).borrow();
        let encoder = *G_ENCODER.borrow(cs).borrow();

        let mut line = String::<16>::new();

        // Row 0: V_Wheel (encoder velocity)
        write!(&mut line, "V_Wheel: {:0>5}", encoder).unwrap();
        d.set_position(0, 0).unwrap();
        d.write_str(&line).unwrap();

        // Row 1: PWM output (instead of battery voltage)
        line.clear();
        write!(&mut line, "PWM: {:7.0}", pwm_out).unwrap();
        d.set_position(0, 1).unwrap();
        d.write_str(&line).unwrap();

        // Row 2: Gyr_Rol (gyro roll rate)
        line.clear();
        write!(&mut line, "Gyr_Rol: {:3.0}", gyro).unwrap();
        d.set_position(0, 2).unwrap();
        d.write_str(&line).unwrap();

        // Row 3: Rol (filtered roll angle)
        line.clear();
        write!(&mut line, "Rol: {:3.1} deg", angle).unwrap();
        d.set_position(0, 3).unwrap();
        d.write_str(&line).unwrap();

        // // Row 4: State
        // line.clear();
        // write!(&mut line, "State: 0").unwrap();
        // d.set_position(0, 4).unwrap();
        // d.write_str(&line).unwrap();

        // Row 5: Cen_G (center gravity)
        let ctrl_ref = G_CTRL.borrow(cs).borrow();
        if let Some(ctrl) = ctrl_ref.as_ref() {
            line.clear();
            write!(&mut line, "Cen_G: {:3.1}", ctrl.balance.center_gravity).unwrap();
            d.set_position(0, 5).unwrap();
            d.write_str(&line).unwrap();

            // Row 6: Cen_SET
            // line.clear();
            // write!(&mut line, "Cen_SET: {:.1}", CENTER_GRAVITY_DEFAULT).unwrap();
            // d.set_position(0, 6).unwrap();
            // d.write_str(&line).unwrap();

            // Row 7: Kp/Kd gains for tuning reference
            line.clear();
            write!(
                &mut line,
                "Kp:{:.0} i:{:.0} d:{:.0}",
                ctrl.balance.kp, ctrl.balance.ki, ctrl.balance.kd
            )
            .unwrap();
            d.set_position(0, 7).unwrap();
            d.write_str(&line).unwrap();
        }

        timer.clear_interrupt(Event::Update);
    });
}

/// Apply motor output matching original Set_Pwm logic
/// Original: DIR = (motox < 0) ? 0 : 1; PWM = 7199 - |motox|
fn apply_motor(
    pwm: &mut Pwm2Channel,
    dir: &mut DirPin,
    brake: &mut BreakPin,
    led: &mut LedPin,
    output: f32,
    max_duty: u16,
) {
    // Clamp output to max_duty range
    let clamped = pid::clamp(output, -(max_duty as f32), max_duty as f32);

    // Direction: match original (negative → DIR=0/low, positive → DIR=1/high)
    if clamped < 0.0 {
        dir.set_low();
        led.set_low();
    } else {
        dir.set_high();
        led.set_high();
    }

    // PWM duty: map |output| to duty cycle
    // Original uses inverted: PWM = max - |value|
    let duty = clamped.abs() as u16;
    let inverted_duty = max_duty.saturating_sub(duty);

    m_start(brake);
    pwm.set_duty(inverted_duty);
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
