// pid_core/src/lib.rs
#![no_std]

use serde::{Deserialize, Serialize};

pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_error: f32,
    pub integral: f32,
}

impl PidController {
    // 1. Define the universal normalized bounds as associated constants
    pub const LIMIT_MIN: f32 = -1.0;
    pub const LIMIT_MAX: f32 = 1.0;

    // 2. Cleaned up the signature (no need to pass limits in anymore)
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            prev_error: 0.0,
            integral: 0.0,
        }
    }

    pub fn compute(&mut self, error: f32, dt: f32) -> f32 {
        let p_out = self.kp * error;
        let derivative = (error - self.prev_error) / dt;
        let d_out = self.kd * derivative;

        let current_i_out = self.ki * self.integral;
        let theoretical_out = p_out + current_i_out + d_out;

        // 3. Reference the constants cleanly using Self::
        let is_hitting_max = theoretical_out >= Self::LIMIT_MAX && error > 0.0;
        let is_hitting_min = theoretical_out <= Self::LIMIT_MIN && error < 0.0;

        if !is_hitting_max && !is_hitting_min {
            self.integral += error * dt;
        }

        self.prev_error = error;
        let final_out = p_out + (self.ki * self.integral) + d_out;

        // 4. Clamp using the constants
        final_out.clamp(Self::LIMIT_MIN, Self::LIMIT_MAX)
    }
}

pub struct CascadedController {
    pub outer_position_pid: PidController,
    pub inner_angle_pid: PidController,
}

impl CascadedController {
    pub fn new(outer: PidController, inner: PidController) -> Self {
        Self {
            outer_position_pid: outer,
            inner_angle_pid: inner
        }
    }

    pub fn compute(
        &mut self,
        target_pos: f32,
        current_pos: f32,
        current_angle: f32,
        dt: f32,
    ) -> (f32, f32) {
        // --- THE STRATEGIST (Outer Loop) ---
        let pos_error = target_pos - current_pos;

        let target_angle = self.outer_position_pid.compute(
            pos_error,
            dt,
        );

        // --- THE MUSCLE (Inner Loop) ---
        let angle_error = target_angle - current_angle;

        let effort = self.inner_angle_pid.compute(angle_error, dt);
        
        // Return both values
        (effort, target_angle)
    }
}

pub trait PosFilter {
    fn update(&mut self, z: f32) -> f32;
    fn reset(&mut self, initial_z: f32);
}

pub struct EmaFilter {
    pub alpha: f32,
    pub x: f32,
}

impl PosFilter for EmaFilter {
    fn update(&mut self, z: f32) -> f32 {
        self.x = (self.alpha * z) + ((1.0 - self.alpha) * self.x);
        self.x
    }
    fn reset(&mut self, initial_z: f32) {
        self.x = initial_z;
    }
}

pub struct KalmanFilter {
    pub q: f32,
    pub r: f32,
    pub p: f32,
    pub x: f32,
}

impl KalmanFilter {
    pub fn new(q: f32, r: f32, initial_z: f32) -> Self {
        Self {
            q,
            r,
            p: 1.0,
            x: initial_z,
        }
    }
}

impl PosFilter for KalmanFilter {
    fn update(&mut self, z: f32) -> f32 {
        let p_predict = self.p + self.q;
        let k = p_predict / (p_predict + self.r);

        self.x = self.x + k * (z - self.x);
        self.p = (1.0 - k) * p_predict;

        self.x
    }

    fn reset(&mut self, initial_z: f32) {
        self.x = initial_z;
        self.p = 1.0;
    }
}

pub enum ActiveFilter {
    Ema(EmaFilter),
    Kalman(KalmanFilter),
}

impl PosFilter for ActiveFilter {
    fn update(&mut self, z: f32) -> f32 {
        match self {
            ActiveFilter::Ema(ema_filter) => ema_filter.update(z),
            ActiveFilter::Kalman(kalman_filter) => kalman_filter.update(z),
        }
    }

    fn reset(&mut self, z: f32) {
        match self {
            ActiveFilter::Ema(ema_filter) => ema_filter.reset(z),
            ActiveFilter::Kalman(kalman_filter) => kalman_filter.reset(z),
        }
    }
}

pub struct ComplementaryFilter {
    pub alpha: f32, // Trust in the noisy Accel (e.g., 0.02)
    pub angle: f32, // The historical state
}

impl ComplementaryFilter {
    pub fn new(alpha: f32, initial_angle: f32) -> Self {
        Self {
            alpha,
            angle: initial_angle,
        }
    }

    /// gyro_rate: Degrees per second
    /// accel_angle: Absolute angle from gravity
    pub fn update(&mut self, gyro_rate: f32, accel_angle: f32, dt: f32) -> f32 {
        // 1. Predict the new angle using the gyroscope's momentum
        let predicted_history = self.angle + (gyro_rate * dt);

        // 2. Standard EMA: (Trust_New * Noisy_Sensor) + (Trust_Old * History)
        self.angle = (self.alpha * accel_angle) + ((1.0 - self.alpha) * predicted_history);

        self.angle
    }
}

pub struct HardwareConfig {
    pub max_voltage: f32, 
    pub deadband: f32,    
    pub max_angle: f32,   // The physical crash limit of the beam
}

impl HardwareConfig {
    /// Translates the physical IMU reading into a normalized percentage [-1.0, 1.0]
    pub fn normalize_angle(&self, physical_angle: f32) -> f32 {
        (physical_angle / self.max_angle).clamp(-1.0, 1.0)
    }

    /// Translates the normalized effort [-1.0, 1.0] into physical motor voltage
    pub fn map_to_motor(&self, effort: f32) -> f32 {
        if effort == 0.0 {
            return 0.0;
        }

        let mut physical_voltage = effort * self.max_voltage;

        if physical_voltage > 0.0 {
            physical_voltage += self.deadband;
        } else if physical_voltage < 0.0 {
            physical_voltage -= self.deadband;
        }

        physical_voltage.clamp(-self.max_voltage, self.max_voltage)
    }
}

pub struct UniverseConfig {
    pub motor_speed_per_volt: f32, // How many degrees/sec the motor turns per Volt
    pub gravity_acceleration: f32, // How fast the angle converts to ball velocity
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PidParams {
    pub outer_p: f32, pub outer_i: f32, pub outer_d: f32,
    pub inner_p: f32, pub inner_i: f32, pub inner_d: f32,
}