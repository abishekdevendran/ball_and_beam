use pc_simulator::run_simulation_and_get_cost;
use pid_core::PidParams;
// Import your cost function from your library here!
// use pc_simulator::run_simulation_and_get_cost; 

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let file_path = args.get(1).map(|s| s.as_str()).unwrap_or("params.json");
    
    let data = std::fs::read_to_string(file_path).expect("File not found");
    let params: PidParams = serde_json::from_str(&data).expect("Bad JSON");
    
    // Evaluate the cost
    let gains = [params.outer_p, params.outer_i, params.outer_d, 
                 params.inner_p, params.inner_i, params.inner_d];
    
    let cost = run_simulation_and_get_cost(&gains);
    
    println!("Evaluating parameters from {}:", file_path);
    println!("Cost Result: {}", cost);
}