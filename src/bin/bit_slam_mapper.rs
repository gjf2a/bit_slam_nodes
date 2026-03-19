use arg_vals::ArgVals;
use bit_slam_nodes::nodes::run_bit_slam_node;

fn main() -> anyhow::Result<()> {
    let args = ArgVals::env();
    if args.len() < 1 {
        println!(
            "Usage: particle_filter_node robot_name [-num_particles=n] [-spin_time=millseconds] [-meters_per_cell=mps]"
        );
    } else {
        run_bit_slam_node(&args)?;
    }
    Ok(())
}

