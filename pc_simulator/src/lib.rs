use pid_core::{
    ActiveFilter, CascadedController, ComplementaryFilter, HardwareConfig, KalmanFilter,
    PidController, PosFilter, UniverseConfig,
};
use rand::{RngExt, SeedableRng, rngs::StdRng};

pub fn run_simulation_and_get_cost(gains: &[f32; 6]) -> f32 {
    // Use a fixed seed for the evaluator
    let mut rng = StdRng::seed_from_u64(42);
    let mut total_episode_cost = 0.0;

    // We force the AI to survive 3 different randomized scenarios
    // using the exact same PID gains.
    let num_episodes = 4;

    let hw_config = HardwareConfig {
        max_voltage: 12.0,
        deadband: 1.0,
        max_angle: 70.0,
    };

    let universe = UniverseConfig {
        motor_speed_per_volt: 15.0,
        gravity_acceleration: 25.0,
    };

    for _episode in 0..num_episodes {
        // --- DOMAIN RANDOMIZATION ---
        // The AI is dropped into a totally random, unknown state
        let mut curr_pos = rng.random_range(30.0..60.0);
        let mut curr_velocity = rng.random_range(-5.0..5.0); // Might be rolling already!
        let mut curr_angle = rng.random_range(-15.0..15.0); // Beam might be tilted!
        let mut true_gyro_rate: f32 = 0.0;
        let gyro_bias = 1.5;

        let target_pos = rng.random_range(10.0..90.0);
        let dt = 0.01;

        // Instantiate the AI's guesses
        let pos_controller = PidController::new(gains[0], gains[1], gains[2]);
        let muscle_controller = PidController::new(gains[3], gains[4], gains[5]);
        let mut controller = CascadedController::new(pos_controller, muscle_controller);

        let mut episode_cost = 0.0;
        let mut prev_effort = 0.0;

        // Instantiate the Filter INSIDE the tuner
        let mut pos_filter = ActiveFilter::Kalman(KalmanFilter::new(0.01, 1.0, curr_pos));
        let mut angle_filter = ComplementaryFilter::new(0.02, curr_angle);

        // Simulate 10 seconds (1000 ticks)
        for _step in 0..1000 {
            // --- 1. SIMULATE NOISY SENSOR ---
            let pos_noise: f32 = rng.random_range(-1.0..1.0);
            let noisy_sensor_pos = curr_pos + pos_noise;

            // --- 2. RUN FILTER PIPELINE ---
            // The AI is now effectively controlling the *filtered* signal,
            // which includes the phase lag you were battling.
            let clean_pos = pos_filter.update(noisy_sensor_pos);

            let gyro_noise: f32 = rng.random_range(-0.1..0.1);
            let noisy_gyro_rate = true_gyro_rate + gyro_bias + gyro_noise;

            let accel_noise: f32 = rng.random_range(-3.0..3.0);
            let noisy_accel_angle = curr_angle + accel_noise;

            // The Complementary Filter magically fuses the drifting gyro and noisy accel
            let clean_angle = angle_filter.update(noisy_gyro_rate, noisy_accel_angle, dt);

            // --- 3. CONTROL ---
            let normalized_current_angle = hw_config.normalize_angle(clean_angle);
            let (effort, _) =
                controller.compute(target_pos, clean_pos, normalized_current_angle, dt);

            let physical_voltage = hw_config.map_to_motor(effort);

            // --- KINEMATIC PHYSICS (Using your working model) ---
            true_gyro_rate = physical_voltage * universe.motor_speed_per_volt;
            curr_angle += true_gyro_rate * dt;
            curr_angle = curr_angle.clamp(-hw_config.max_angle, hw_config.max_angle);

            curr_velocity += curr_angle * universe.gravity_acceleration * dt;
            curr_pos += curr_velocity * dt;

            // --- THE PHYSICAL WALLS & EARLY EXIT ---
            if curr_pos <= 0.0 || curr_pos >= 100.0 {
                // OLD: episode_cost += 10_000.0 + (remaining_ticks * 100.0);

                // NEW: Penalty scaled by distance from target at failure.
                // This tells the AI: "It's okay to crash, as long as you were close to the target."
                let distance_at_failure = (target_pos - curr_pos).abs();
                episode_cost += 1000.0 + (distance_at_failure * 10.0);
                break;
            }

            // --- LQR COST ACCUMULATION ---
            let pos_error = target_pos - curr_pos;
            episode_cost += pos_error * pos_error * 3.0; // Accuracy
            episode_cost += curr_velocity * curr_velocity * 3.0; // Stability

            // This punishes the AI for making the beam move violently
            episode_cost += (true_gyro_rate * true_gyro_rate) * 0.2;

            let effort_delta = effort - prev_effort;
            episode_cost += effort_delta * effort_delta * 0.5; // Smoothness

            prev_effort = effort;
        }

        total_episode_cost += episode_cost;
    }

    // Return the average cost across all randomized scenarios
    total_episode_cost / (num_episodes as f32)
}
