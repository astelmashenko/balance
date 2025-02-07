// use libm::{atan2f, powf, sqrtf};
// use nalgebra::{Vector2, Vector3};

const KP: f32 = 75.0;
const KI: f32 = 5.0;
const KD: f32 = 0.5;
const ALPHA: f32 = 0.4;

pub const ANGLE_BIAS: f32 = 57.3;

// pub fn angle_accel(acc: Vector3<f32>) -> Vector2<f32> {
//     Vector2::<f32>::new(
//         atan2f(acc.y, acc.z) * ANGLE_BIAS,
//         atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.))) * ANGLE_BIAS,
//     )
//     // roll = atan2(y_Buff , z_Buff) * 57.3;
//     // pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
// }

pub fn pid1(ang_x: f32, gz_filt_in: f32, gz: f32, m_speed_in: i32) -> (i32, i32, f32) {
    // gyroZ = GyZ / 131.0; // Convert to deg/s
    // gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
    // pwm_s = -constrain(X1 * robot_angle + X2 * gyroZfilt + X3 * -motor_speed, -255, 255);

    // robot_angle += GyZ * loop_time / 1000 / 65.536;
    // Acc_angle = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values       * 57.2958 (deg/rad)
    // robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
    // gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
    let gz_filt = ALPHA * gz + (1.0 - ALPHA) * gz_filt_in;

    let pwm_val = KP * ang_x + KI * gz_filt + KD * (-m_speed_in as f32);
    let pwm = -constrain(pwm_val as i32, -255, 255);
    let m_speed = m_speed_in + pwm;
    (pwm, m_speed, gz_filt)
}

fn constrain(val: i32, low: i32, high: i32) -> i32 {
    if val > high {
        high
    } else if val < low {
        low
    } else {
        val
    }
}
