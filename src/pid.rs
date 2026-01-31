//! PID Controller for Self-Balancing Robot
//!
//! Implements:
//! - Complementary filter for angle estimation (gyro + accelerometer fusion)
//! - Standard PID controller with anti-windup
//! - Derivative filtering to reduce noise

/// Configuration parameters for PID controller (tunable)
#[derive(Clone)]
pub struct PidConfig {
    /// Proportional gain - response to current error
    pub kp: f32,
    /// Integral gain - response to accumulated error
    pub ki: f32,
    /// Derivative gain - response to rate of change of error
    pub kd: f32,
    /// Target angle in degrees (balance setpoint, usually near 0)
    pub setpoint: f32,
    /// Maximum integral accumulation (anti-windup)
    pub integral_limit: f32,
    /// Output limit (maps to PWM range)
    pub output_limit: f32,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 30.0,      // Start conservative, tune up
            ki: 0.5,       // Keep low initially
            kd: 0.8,       // Helps dampen oscillation
            setpoint: 0.0, // Degrees from vertical
            integral_limit: 50.0,
            output_limit: 255.0,
        }
    }
}

/// Runtime state for PID controller
#[derive(Default)]
pub struct PidState {
    /// Accumulated integral term
    pub integral: f32,
    /// Previous error (for derivative calculation)
    pub prev_error: f32,
    /// Previous derivative (for filtering)
    pub prev_derivative: f32,
}

impl PidState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_derivative = 0.0;
    }
}

/// Complementary filter for angle estimation from accelerometer and gyroscope
pub struct AngleEstimator {
    /// Current estimated angle in degrees
    pub angle: f32,
    /// Filter coefficient (0.0-1.0, higher = trust gyro more)
    /// Typical value: 0.98 (98% gyro, 2% accelerometer)
    pub alpha: f32,
}

impl Default for AngleEstimator {
    fn default() -> Self {
        Self {
            angle: 0.0,
            alpha: 0.98,
        }
    }
}

impl AngleEstimator {
    pub fn new(alpha: f32) -> Self {
        Self { angle: 0.0, alpha }
    }

    /// Update angle estimate with new sensor readings
    ///
    /// Complementary filter formula:
    /// angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
    ///
    /// - Gyro integration: good for fast changes, drifts over time
    /// - Accel angle: good for steady state, noisy during movement
    ///
    /// # Arguments
    /// * `accel_angle` - Angle calculated from accelerometer (degrees)
    /// * `gyro_rate` - Angular velocity from gyroscope (deg/s)
    /// * `dt` - Time step in seconds
    pub fn update(&mut self, accel_angle: f32, gyro_rate: f32, dt: f32) -> f32 {
        self.angle = self.alpha * (self.angle + gyro_rate * dt) + (1.0 - self.alpha) * accel_angle;
        self.angle
    }
}

/// Complete balance controller combining angle estimation and PID
pub struct BalanceController {
    pub config: PidConfig,
    pub state: PidState,
    pub angle_est: AngleEstimator,
    /// Control loop period in seconds
    pub dt: f32,
}

impl BalanceController {
    /// Create a new balance controller with default settings for 100Hz loop
    pub fn new() -> Self {
        Self {
            config: PidConfig::default(),
            state: PidState::new(),
            angle_est: AngleEstimator::default(),
            dt: 0.01, // 100Hz = 10ms
        }
    }

    /// Create controller with custom configuration
    pub fn with_config(config: PidConfig, alpha: f32, dt: f32) -> Self {
        Self {
            config,
            state: PidState::new(),
            angle_est: AngleEstimator::new(alpha),
            dt,
        }
    }

    /// Update angle estimate from sensor readings
    ///
    /// # Arguments
    /// * `accel_angle` - Angle from accelerometer (degrees)
    /// * `gyro_rate` - Angular velocity from gyroscope (deg/s)
    ///
    /// # Returns
    /// Estimated angle in degrees
    pub fn update_angle(&mut self, accel_angle: f32, gyro_rate: f32) -> f32 {
        self.angle_est.update(accel_angle, gyro_rate, self.dt)
    }

    /// Compute PID output based on current angle
    ///
    /// # Returns
    /// PWM value in range [-output_limit, +output_limit]
    pub fn compute(&mut self, current_angle: f32) -> f32 {
        let config = &self.config;
        let state = &mut self.state;

        // Calculate error (setpoint - measurement)
        let error = config.setpoint - current_angle;

        // === Proportional Term ===
        let p_term = config.kp * error;

        // === Integral Term with Anti-Windup ===
        state.integral += error * self.dt;
        state.integral = clamp(
            state.integral,
            -config.integral_limit,
            config.integral_limit,
        );
        let i_term = config.ki * state.integral;

        // === Derivative Term with filtering ===
        let raw_derivative = (error - state.prev_error) / self.dt;

        // Low-pass filter on derivative to reduce noise
        const DERIVATIVE_FILTER_BETA: f32 = 0.7;
        let filtered_derivative = DERIVATIVE_FILTER_BETA * raw_derivative
            + (1.0 - DERIVATIVE_FILTER_BETA) * state.prev_derivative;

        let d_term = config.kd * filtered_derivative;

        // Store for next iteration
        state.prev_error = error;
        state.prev_derivative = filtered_derivative;

        // === Combine and Limit Output ===
        let output = p_term + i_term + d_term;
        clamp(output, -config.output_limit, config.output_limit)
    }

    /// Combined update: estimate angle and compute PID output
    ///
    /// # Arguments
    /// * `accel_angle` - Angle from accelerometer (degrees)
    /// * `gyro_rate` - Angular velocity from gyroscope (deg/s)
    ///
    /// # Returns
    /// (estimated_angle, pwm_output)
    pub fn update(&mut self, accel_angle: f32, gyro_rate: f32) -> (f32, f32) {
        let angle = self.update_angle(accel_angle, gyro_rate);
        let output = self.compute(angle);
        (angle, output)
    }

    /// Reset controller state (call when robot falls or restarts)
    pub fn reset(&mut self) {
        self.state.reset();
        self.angle_est.angle = 0.0;
    }

    /// Check if angle is within safe operating range
    pub fn is_safe(&self, max_angle: f32) -> bool {
        self.angle_est.angle.abs() < max_angle
    }
}

impl Default for BalanceController {
    fn default() -> Self {
        Self::new()
    }
}

/// Clamp a value to a range
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

/// Maximum safe angle before considering the robot has fallen (degrees)
pub const MAX_SAFE_ANGLE: f32 = 45.0;
