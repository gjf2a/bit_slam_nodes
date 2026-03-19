use std::env;

use bit_slam_nodes::nodes::run_bump_obstacle_node;

fn main() -> anyhow::Result<()> {
    let args = env::args().collect::<Vec<_>>();
    let robot_name = if args.len() == 2 {
        format!("/{}", args[1])
    } else {
        "".to_string()
    };
    run_bump_obstacle_node(&robot_name)
}
