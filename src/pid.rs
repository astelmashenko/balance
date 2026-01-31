//! PID Controller matching original C balance-triangle implementation
//!
//! Architecture:
//! - Complementary filter: 0.1 * accel_angle + 0.9 * (prev_angle + gyro * 0.003)
//! - Balance PD: Kp * (angle - center_gravity) + Kd * gyro_rate
//! - Velocity PI: Kp * filtered_encoder + Ki * integral_encoder (when encoder available)

use libm::atan2f;

/// Complementary filter for angle estimation
/// Matches original filter.c: Yijielvbo_X
pub struct AngleFilter {
    /// Filtered angle in degrees
    pub angle: f32,
}

impl AngleFilter {
    pub fn new() -> Self {
        Self { angle: 0.0 }
    }

    /// Update filter with new sensor readings
    /// Original: Angle = 0.1 * accel_angle + 0.9 * (prev_angle + gyro_rate * 0.003)
    pub fn update(&mut self, accel_angle: f32, gyro_rate: f32) -> f32 {
        self.angle = 0.1 * accel_angle + 0.9 * (self.angle + gyro_rate * 0.003);
        self.angle
    }
}

/// Compute accelerometer angle from raw acceleration values
/// Original: atan2(Accel_Z, Accel_Y) * 180 / PI
/// The mpu6050 crate returns acceleration in g units, which works directly with atan2
#[inline]
pub fn accel_angle(acc_z: f32, acc_y: f32) -> f32 {
    atan2f(acc_z, acc_y) * 57.295_78 // 180 / PI
}

/// Balance PD controller
/// Matches original control.c: balance_x
///
/// Original formula:
///   balance = Balance_KP * (Angle - Center_Gravity) + Balance_KD * Gyro
///
/// This is a PD controller where the derivative term uses the gyroscope
/// reading directly (not numerical differentiation of the error).
pub struct BalancePD {
    pub kp: f32,
    pub kd: f32,
    /// Center of gravity angle (balance setpoint in degrees)
    pub center_gravity: f32,
    /// Integral accumulator (Ki=0 by default, but available)
    pub integral: f32,
    pub ki: f32,
    /// Anti-windup limit for integral
    pub integral_limit: f32,
}

impl BalancePD {
    /// Create with original default gains
    pub fn new(center_gravity: f32) -> Self {
        Self {
            kp: 1100.0,
            ki: 0.5, // 0.0
            kd: 4.0, // 4.0
            center_gravity,
            integral: 0.0,
            integral_limit: 30000.0,
        }
    }

    /// Compute balance PWM output
    /// angle: filtered angle from complementary filter (degrees)
    /// gyro: raw gyroscope rate (deg/s) — used directly as derivative term
    pub fn compute(&mut self, angle: f32, gyro: f32) -> f32 {
        let bias = angle - self.center_gravity;

        // Integral with anti-windup (unused when ki=0)
        self.integral += bias;
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit);

        self.kp * bias + self.ki * self.integral + self.kd * gyro
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}

/// Velocity PI controller (for encoder feedback)
/// Matches original control.c: velocity_x
///
/// Original formula:
///   filtered_encoder = 0.65 * prev + 0.35 * new
///   integral += filtered_encoder (clamped ±10000)
///   velocity_pwm = Position_KP * filtered_encoder + Position_KI * integral
pub struct VelocityPI {
    pub kp: f32,
    pub ki: f32,
    pub filtered_encoder: f32,
    pub integral: f32,
    pub integral_limit: f32,
}

impl VelocityPI {
    pub fn new() -> Self {
        Self {
            kp: -600.0,
            ki: -0.5,
            filtered_encoder: 0.0,
            integral: 0.0,
            integral_limit: 10000.0,
        }
    }

    /// Compute velocity PWM output
    /// encoder: current encoder reading (counts per sample)
    pub fn compute(&mut self, encoder: f32) -> f32 {
        // First-order low-pass filter: 65% old + 35% new
        self.filtered_encoder = 0.65 * self.filtered_encoder + 0.35 * encoder;

        // Integral with anti-windup
        self.integral += self.filtered_encoder;
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit);

        self.kp * self.filtered_encoder + self.ki * self.integral
    }

    pub fn reset(&mut self) {
        self.filtered_encoder = 0.0;
        self.integral = 0.0;
    }
}

/// Complete balance controller combining angle filter + PD + optional velocity PI
pub struct Controller {
    pub filter: AngleFilter,
    pub balance: BalancePD,
    pub velocity: VelocityPI,
    /// Raw gyro value for display (Gyro_Balance_x in original)
    pub gyro_raw: f32,
}

impl Controller {
    pub fn new(center_gravity: f32) -> Self {
        Self {
            filter: AngleFilter::new(),
            balance: BalancePD::new(center_gravity),
            velocity: VelocityPI::new(),
            gyro_raw: 0.0,
        }
    }

    /// Full update cycle: read sensors, filter, compute PD
    /// Returns (filtered_angle, pwm_output)
    ///
    /// acc_z, acc_y: accelerometer values (g units or raw — atan2 works either way)
    /// gyro_x: gyroscope X rate (deg/s)
    pub fn update(&mut self, acc_z: f32, acc_y: f32, gyro_x: f32) -> (f32, f32) {
        // Compute accelerometer angle: atan2(acc_z, acc_y) * 180/PI
        let accel_ang = accel_angle(acc_z, acc_y);

        // Gyro rate (negated to match original: -Gyro_X)
        let gyro_rate = -gyro_x;
        self.gyro_raw = gyro_rate;

        // Complementary filter
        let angle = self.filter.update(accel_ang, gyro_rate);

        // Balance PD controller
        let balance_pwm = self.balance.compute(angle, gyro_rate);

        // Velocity PI (pass 0 encoder if no encoder available)
        // let velocity_pwm = self.velocity.compute(encoder);
        // let pwm = balance_pwm + velocity_pwm;

        (angle, balance_pwm)
    }

    /// Reset all controller state
    pub fn reset(&mut self) {
        self.balance.reset();
        self.velocity.reset();
    }
}

/// PWM output limit
pub const MAX_SAFE_ANGLE: f32 = 45.0;

#[inline]
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value > max {
        max
    } else if value < min {
        min
    } else {
        value
    }
}
