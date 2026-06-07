use pc_simulator::run_simulation_and_get_cost;
use pid_core::PidParams;
// tuner.rs
use rand::{Rng, RngExt};
use rayon::prelude::*;

const DIMENSIONS: usize = 6; // [Outer P, Outer I, Outer D, Inner P, Inner I, Inner D]

#[derive(Clone, Debug)]
pub struct Particle {
    pub position: [f32; DIMENSIONS],
    pub velocity: [f32; DIMENSIONS],
    pub personal_best_pos: [f32; DIMENSIONS],
    pub personal_best_cost: f32, // Lower is better (Zero error is the goal)
}

impl Particle {
    pub fn new(bounds: &[(f32, f32); DIMENSIONS], rng: &mut impl Rng) -> Self {
        let mut pos = [0.0; DIMENSIONS];
        let mut vel = [0.0; DIMENSIONS];

        for i in 0..DIMENSIONS {
            let (min, max) = bounds[i];
            pos[i] = rng.random_range(min..max);
            // Initial velocity is a random fraction of the search space width
            vel[i] = rng.random_range(-(max - min) * 0.1..(max - min) * 0.1);
        }

        Self {
            position: pos,
            velocity: vel,
            personal_best_pos: pos,
            personal_best_cost: f32::MAX, // Start with worst possible cost
        }
    }
}

pub struct SwarmOptimizer {
    pub particles: Vec<Particle>,
    pub global_best_pos: [f32; DIMENSIONS],
    pub global_best_cost: f32,
    pub bounds: [(f32, f32); DIMENSIONS],
}

impl SwarmOptimizer {
    pub fn optimize(&mut self, epochs: usize) {
        let w = 0.9; // Inertia weight
        let c1 = 1.5; // Cognitive weight (Self-trust)
        let c2 = 1.5; // Social weight (Swarm-trust)

        let mut epochs_since_improvement = 0;
        let mut last_best_cost = f32::MAX;

        for epoch in 0..epochs {
            // 1. EVALUATE FITNESS (IN PARALLEL)
            // This is where Rayon shines. It splits the swarm across your CPU cores.
            self.particles.par_iter_mut().for_each(|particle| {
                // RUN YOUR PHYSICS SIMULATION HERE FOR 10 SECONDS
                let cost = run_simulation_and_get_cost(&particle.position);

                if cost < particle.personal_best_cost {
                    particle.personal_best_cost = cost;
                    particle.personal_best_pos = particle.position;
                }
            });

            // 2. FIND GLOBAL BEST (SEQUENTIAL)
            for particle in &self.particles {
                if particle.personal_best_cost < self.global_best_cost {
                    self.global_best_cost = particle.personal_best_cost;
                    self.global_best_pos = particle.personal_best_pos;
                }
            }

            if self.global_best_cost < last_best_cost {
                last_best_cost = self.global_best_cost;
                epochs_since_improvement = 0;
            } else {
                epochs_since_improvement += 1;
            }

            // 3. UPDATE VELOCITIES AND POSITIONS (IN PARALLEL)
            let g_best = self.global_best_pos; // Copy for the closure

            self.particles.par_iter_mut().for_each(|particle| {
                let mut rng = rand::rng(); // Thread-local RNG

                for (i, (vel, (pos, g_best_val))) in particle
                    .velocity
                    .iter_mut()
                    .zip(particle.position.iter_mut().zip(g_best.iter()))
                    .enumerate()
                {
                    let r1: f32 = rng.random();
                    let r2: f32 = rng.random();

                    *vel = w * *vel
                        + c1 * r1 * (particle.personal_best_pos[i] - *pos)
                        + c2 * r2 * (*g_best_val - *pos);

                    *pos += *vel;
                    *pos = pos.clamp(self.bounds[i].0, self.bounds[i].1);
                }
            });

            println!("Epoch {}: Best Cost = {}", epoch, self.global_best_cost);

            // The Reseeding Trigger
            if epochs_since_improvement >= 15 {
                println!(
                    "Stagnation detected at Epoch {}. Re-seeding swarm...",
                    epoch
                );

                // Sort particles by personal best cost (worst to best)
                self.particles.sort_by(|a, b| {
                    a.personal_best_cost
                        .partial_cmp(&b.personal_best_cost)
                        .unwrap()
                });

                // Teleport the bottom 70% to new random positions
                let bottom = (self.particles.len() as f32) * 0.7;
                for particle in self.particles.iter_mut().take(bottom.round() as usize) {
                    *particle = Particle::new(&self.bounds, &mut rand::rng());
                }

                // Reset the counter
                epochs_since_improvement = 0;
            }
        }
    }
}

fn main() {
    let bounds = [
        (0.001, 0.1),  // Outer P
        (0.000, 0.01), // Outer I
        (0.001, 0.1),  // Outer D
        (0.5, 20.0),   // Inner P
        (0.0, 0.1),    // Inner I
        (0.0, 0.5),    // Inner D
    ];

    let mut optimizer = SwarmOptimizer {
        particles: (0..10000)
            .map(|_| Particle::new(&bounds, &mut rand::rng()))
            .collect(),
        global_best_pos: [0.0; DIMENSIONS],
        global_best_cost: f32::MAX,
        bounds,
    };

    println!("Starting swarm optimization...");
    optimizer.optimize(100); // Run for 100 epochs

    println!("Optimization complete!");
    let params = PidParams {
        outer_p: optimizer.global_best_pos[0],
        outer_i: optimizer.global_best_pos[1],
        outer_d: optimizer.global_best_pos[2],
        inner_p: optimizer.global_best_pos[3],
        inner_i: optimizer.global_best_pos[4],
        inner_d: optimizer.global_best_pos[5],
    };

    let json = serde_json::to_string_pretty(&params).unwrap();
    std::fs::write("params.json", json).expect("Unable to write file");
    println!("Saved gains to params.json");
    println!("Best Gains Found: {:?}", optimizer.global_best_pos);
}
