use arg_vals::ArgVals;
use bit_slam_nodes::nodes::bump_obstacle_node;

fn main() -> anyhow::Result<()> {
    let mut args = ArgVals::env();
    if args.len() == 0 {
        args.add_simple("");
    }
    let spec = bump_obstacle_node(&args)?;
    spec.run()
}
