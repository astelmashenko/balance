  ---
  PID Controller Architecture

  The system uses a cascaded PID design with two loops:

  Inner Loop: Angle Balance (control.c:199-215)

  Balance_KP = 450;   // Strong proportional response
  Balance_KI = 0;     // Integral disabled (avoids wind-up)
  Balance_KD = 4;     // Derivative from raw gyroscope

  Error is computed as Angle - Center_Gravity, the derivative term uses the gyroscope reading directly (not a numerical difference), and the integral is clamped to ±30000 for
  anti-windup. In practice this is a PD controller since Ki=0.

  Outer Loop: Velocity/Position (control.c:217-233)

  Position_KP = -600;
  Position_KI = -0.5;
  Position_KD = 0;

  The encoder signal is smoothed with a first-order low-pass filter (65% old value + 35% new), and integral is clamped to ±10000. This is a PI controller (Kd=0). The negative gains
  provide negative feedback for stability.

  Spinup Controller: Incremental PI (control.c:248-253)

  Used during the triangle's initial swing-up to vertical:
  Kp = 2, Ki = 0.5
  Pwm += Kp * (Bias - Last_bias) + Ki * Bias   // incremental form

  A state machine (states 0-3 for left-up, 11-13 for right-up) sequences through the spinup maneuver.

  ---
  Angle Filter

  Complementary Filter (filter.c)

  Angle = 0.1 * accel_angle + 0.9 * (prev_angle + gyro_rate * 0.003)

  - 10% weight on accelerometer (accurate long-term, noisy short-term)
  - 90% weight on gyroscope integration (smooth short-term, drifts long-term)
  - Integration timestep: 3 ms (0.003 s)

  This is a first-order complementary filter, not a Kalman filter.

  ---
  Angle Transformation to Degrees

  Accelerometer → Angle (control.c:263-304)

  Raw 16-bit signed values from MPU6050 are read over I2C, then:

  angle_degrees = atan2(acc_side, acc_vertical) * 180 / PI

  For the main balance axis: atan2(acc_z, acc_y) * 180 / 3.14159265

  Gyroscope → Degrees/Second

  gyro_dps = raw_value / 16.4

  The MPU6050 is configured at ±1000 dps range, giving a sensitivity of 16.4 LSB per degree/second.

  MPU6050 Configuration (mpu6050.c)
  ┌─────────────┬────────────────────────────────────────┐
  │   Setting   │                 Value                  │
  ├─────────────┼────────────────────────────────────────┤
  │ Gyro range  │ ±1000 dps (register 0x10)              │
  ├─────────────┼────────────────────────────────────────┤
  │ Accel range │ ±4g (register 0x09)                    │
  ├─────────────┼────────────────────────────────────────┤
  │ Sample rate │ 500 Hz                                 │
  ├─────────────┼────────────────────────────────────────┤
  │ DLPF        │ 20 Hz low-pass                         │
  ├─────────────┼────────────────────────────────────────┤
  │ Calibration │ 200-sample offset averaging at startup │
  └─────────────┴────────────────────────────────────────┘
  ---
  OLED Display (128x64 SSD1306, I2C on PC13/PC14)

  The display in show.c shows 8 rows of telemetry:
  ┌─────┬──────────┬────────────┬───────────────────────────────────────┐
  │ Row │  Label   │   Value    │              Description              │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 0   │ V_Wheel: │ ±NNN       │ Encoder velocity (counts/sample)      │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 1   │ Vol_Bat: │ NNNN mV    │ Battery voltage in millivolts         │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 2   │ Gyr_Rol: │ ±NNNN      │ Gyroscope X rate (raw, deg/s scale)   │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 3   │ Rol:     │ ±NNN.N deg │ Filtered roll angle (1 decimal place) │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 4   │ State:   │ NNN        │ Current state machine state (0-13)    │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 5   │ Cen_G:   │ NN.N deg   │ Dynamic center-of-gravity angle       │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 6   │ Cen_SET: │ NN.N deg   │ Configured center-of-gravity setpoint │
  ├─────┼──────────┼────────────┼───────────────────────────────────────┤
  │ 7   │ Ene_Sto: │ N.NN deg   │ Energy storage coefficient (0.0-0.6)  │
  └─────┴──────────┴────────────┴───────────────────────────────────────┘
  The angle on row 3 is the output of the complementary filter — the fused estimate used by the PID. The sign (±) and decimal point are rendered manually by the display code.

  ---
  Other Hardware Configuration

  - Motor PWM: TIM3 CH2 on PA7, 10 kHz (ARR=7199, PSC=0), direction on PA6, enable on PA4
  - Control loop timer: TIM1, 100 Hz interrupt (ARR=99, PSC=7199)
  - Encoder: TIM2 quadrature mode on PA0/PA1
  - Battery ADC: Channel 8, formula ADC * 6.6 / 4096 (voltage divider scaled)
  - Flash storage: Center_Gra_Sart (default 88.9°) and Energy_Storage (default 0.45) are persisted to STM32 internal flash
