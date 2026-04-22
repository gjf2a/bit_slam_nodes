use arg_vals::ArgVals;
use arg_vals::merged_arg_docs;
use bit_slam_nodes::{
    bit_slam_nodes::{BitSlamNode, BumpObstacleNode},
    node_struct::{RunnableNode, run_nodes},
    unguided_nodes::BumpTurnNode,
};

fn main() -> anyhow::Result<()> {
    let nodes: Vec<Box<dyn RunnableNode>> = vec![
        Box::new(BitSlamNode::default()),
        Box::new(BumpObstacleNode::default()),
        Box::new(BumpTurnNode::default()),
    ];
    let actual_args = ArgVals::env();
    if actual_args.len() == 0 {
        let merged_docs = merged_arg_docs(nodes.iter().map(|n| n.arg_docs()));
        eprintln!("{merged_docs}");
    } else {
        run_nodes(nodes)?;
    }
    Ok(())
}
