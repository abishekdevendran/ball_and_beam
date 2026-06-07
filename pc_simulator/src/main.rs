mod telemetry;

use std::{fs, path::Path, thread, time::Duration};
use pid_core::{
    ActiveFilter, CascadedController, ComplementaryFilter, HardwareConfig, KalmanFilter, PidController, PidParams, PosFilter, UniverseConfig
};
use rand::{RngExt, SeedableRng, rngs::StdRng};
use telemetry::TerminalChart;

fn load_params(path: &str) -> PidParams {
    if Path::new(path).exists() {
        let data = fs::read_to_string(path).expect("Failed to read params.json");
        serde_json::from_str(&data).expect("Failed to parse JSON")
    } else {
        println!("No params.json found, using safe defaults.");
        PidParams {
            outer_p: 0.0069, outer_i: 0.0005, outer_d: 0.005,
            inner_p: 10.0,   inner_i: 0.1,    inner_d: 0.0,
        }
    }
}

fn main() {
    // 1. Config & Setup
    let params = load_params("params.json");
    let mut rng = StdRng::seed_from_u64(42);
    let mut dashboard = TerminalChart::new(100);

    // 2. Physics & System State
    let mut curr_pos = 10.0;
    let mut curr_velocity = 0.0;
    let mut curr_angle = 0.0;
    let mut true_gyro_rate = 0.0;
    
    let target_pos = 75.0;
    let dt = 0.01;
    let gyro_bias = 1.5;

    // 3. Filters & Controllers
    let mut pos_filter = ActiveFilter::Kalman(KalmanFilter::new(0.01, 1.0, curr_pos));
    let mut angle_filter = ComplementaryFilter::new(0.02, 0.0);
    
    let mut controller = CascadedController::new(
        PidController::new(params.outer_p, params.outer_i, params.outer_d),
        PidController::new(params.inner_p, params.inner_i, params.inner_d),
    );

    let hw_config = HardwareConfig { max_voltage: 12.0, deadband: 1.0, max_angle: 70.0 };
    let universe = UniverseConfig { motor_speed_per_volt: 15.0, gravity_acceleration: 25.0 };

    // 4. Execution Loop
    for step in 1..500 {
        let curr_time = (step as f32) * dt;

        // Phase 1: Sensing (Simulate noise)
        let noisy_pos = curr_pos + rng.random_range(-1.0..1.0);
        let noisy_angle = curr_angle + rng.random_range(-3.0..3.0);
        let noisy_gyro = true_gyro_rate + gyro_bias + rng.random_range(-0.1..0.1);

        // Phase 2: Signal Reconstruction
        let clean_pos = pos_filter.update(noisy_pos);
        let clean_angle = angle_filter.update(noisy_gyro, noisy_angle, dt);

        // Phase 3: Control Logic
        let (effort, _) = controller.compute(
            target_pos, 
            clean_pos, 
            hw_config.normalize_angle(clean_angle), 
            dt
        );

        let voltage = hw_config.map_to_motor(effort);

        // Phase 4: Physics Integration
        true_gyro_rate = voltage * universe.motor_speed_per_volt;
        curr_angle = (curr_angle + true_gyro_rate * dt).clamp(-hw_config.max_angle, hw_config.max_angle);
        curr_velocity += curr_angle * universe.gravity_acceleration * dt;
        curr_pos = (curr_pos + curr_velocity * dt).clamp(0.0, 100.0);

        // Reset velocity on impact
        if curr_pos <= 0.0 || curr_pos >= 100.0 { curr_velocity = 0.0; }

        // Phase 5: Telemetry
        dashboard.draw(curr_time, target_pos, curr_pos, noisy_pos, clean_pos, 0.0, curr_angle, clean_angle);
        thread::sleep(Duration::from_millis(30));
    }
}