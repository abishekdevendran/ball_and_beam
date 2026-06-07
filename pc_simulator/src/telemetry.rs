use std::collections::VecDeque;
use textplots::{Chart, ColorPlot, Shape};
use rgb::RGB8;

pub struct TerminalChart {
    // Position Data
    true_pos: VecDeque<(f32, f32)>,
    noisy_pos: VecDeque<(f32, f32)>,
    clean_pos: VecDeque<(f32, f32)>,
    target_pos: VecDeque<(f32, f32)>,
    
    // Angle Data
    target_angle: VecDeque<(f32, f32)>,
    true_angle: VecDeque<(f32, f32)>,
    clean_angle: VecDeque<(f32, f32)>,
    
    max_points: usize,
}

impl TerminalChart {
    pub fn new(max_points: usize) -> Self {
        Self {
            true_pos: VecDeque::with_capacity(max_points + 1),
            noisy_pos: VecDeque::with_capacity(max_points + 1),
            clean_pos: VecDeque::with_capacity(max_points + 1),
            target_pos: VecDeque::with_capacity(max_points + 1),
            target_angle: VecDeque::with_capacity(max_points + 1),
            true_angle: VecDeque::with_capacity(max_points + 1),
            clean_angle: VecDeque::with_capacity(max_points + 1),
            max_points,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &mut self, 
        time: f32, 
        target_p: f32,
        true_p: f32, 
        noisy_p: f32, 
        clean_p: f32, 
        target_a: f32,
        true_a: f32, 
        clean_a: f32,
    ) {
        // 1. Update sliding windows
        self.target_pos.push_back((time, target_p));
        self.true_pos.push_back((time, true_p));
        self.noisy_pos.push_back((time, noisy_p));
        self.clean_pos.push_back((time, clean_p));
        
        self.target_angle.push_back((time, target_a));
        self.true_angle.push_back((time, true_a));
        self.clean_angle.push_back((time, clean_a));

        if self.true_pos.len() > self.max_points {
            self.target_pos.pop_front();
            self.true_pos.pop_front();
            self.noisy_pos.pop_front();
            self.clean_pos.pop_front();
            self.target_angle.pop_front();
            self.true_angle.pop_front();
            self.clean_angle.pop_front();
        }

        let min_x = self.true_pos.front().unwrap().0;
        let max_x = time;
        
        let pos_error = target_p - clean_p;
        let angle_error = target_a - clean_a;

        // 2. Clear terminal
        print!("{esc}c", esc = 27 as char);
        
        // --- CHART 1: POSITION DASHBOARD ---
        println!("=== THE STRATEGIST (OUTER LOOP - POSITION) ===");
        println!("Time: {:<6.2} | Target: {:<6.2} | Current: {:<6.2} | Error: \x1b[38;2;255;165;0m{:<6.2}\x1b[0m", time, target_p, clean_p, pos_error);
        println!("--- \x1b[38;2;255;255;255mTarget (White)\x1b[0m | \x1b[38;2;255;50;50mNoisy (Red)\x1b[0m | \x1b[38;2;100;100;255mTrue (Blue)\x1b[0m | \x1b[38;2;0;255;0mFiltered (Green)\x1b[0m ---");
        
        Chart::new(160, 20, min_x, max_x)
            .linecolorplot(&Shape::Lines(self.target_pos.make_contiguous()), RGB8 { r: 255, g: 255, b: 255 })
            .linecolorplot(&Shape::Points(self.noisy_pos.make_contiguous()), RGB8 { r: 255, g: 50, b: 50 })
            .linecolorplot(&Shape::Lines(self.true_pos.make_contiguous()), RGB8 { r: 100, g: 100, b: 255 })
            .linecolorplot(&Shape::Lines(self.clean_pos.make_contiguous()), RGB8 { r: 0, g: 255, b: 0 })
            .display();

        // --- CHART 2: ANGLE DASHBOARD ---
        println!("\n=== THE MUSCLE (INNER LOOP - ANGLE) ===");
        println!("Target: {:<6.2} | Current: {:<6.2} | Error: \x1b[38;2;255;165;0m{:<6.2}\x1b[0m", target_a, clean_a, angle_error);
        println!("--- \x1b[38;2;255;255;255mTarget (White)\x1b[0m | \x1b[38;2;100;100;255mTrue (Blue)\x1b[0m | \x1b[38;2;0;255;0mFiltered (Green)\x1b[0m ---");
        
        Chart::new(160, 20, min_x, max_x)
            .linecolorplot(&Shape::Lines(self.target_angle.make_contiguous()), RGB8 { r: 255, g: 255, b: 255 })
            .linecolorplot(&Shape::Lines(self.true_angle.make_contiguous()), RGB8 { r: 100, g: 100, b: 255 })
            .linecolorplot(&Shape::Lines(self.clean_angle.make_contiguous()), RGB8 { r: 0, g: 255, b: 0 })
            .display();
    }
}