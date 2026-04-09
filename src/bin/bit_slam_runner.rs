use std::{sync::Arc, time::Duration};

use arg_vals::{ArgVals, merged_arg_docs};
use bit_slam_nodes::{
    bit_slam_nodes::{BitSlamExplorerNode, BitSlamNode, BumpObstacleNode},
    fuzzy_nodes::{DefuzzifyingErrorCorrectingNode, GoalFuzzifierNode},
    node_struct::RunnableNode,
};
use crossbeam::atomic::AtomicCell;

fn main() -> anyhow::Result<()> {
    let explorer = BitSlamExplorerNode::default();
    let mut goal_fuzzifier = GoalFuzzifierNode::default();
    goal_fuzzifier.add_default(
        "--fuzzy-goal-topic",
        &explorer.goal_publish_topic(&explorer.arg_docs().get_args_with_defaults())?,
    )?;
    goal_fuzzifier.add_default(
        "--reset-topic",
        &explorer.stop_publish_topic(&explorer.arg_docs().get_args_with_defaults())?,
    )?;
    let nodes: Vec<Box<dyn RunnableNode>> = vec![
        Box::new(BitSlamNode::default()),
        Box::new(explorer),
        Box::new(BumpObstacleNode::default()),
        Box::new(goal_fuzzifier),
        Box::new(DefuzzifyingErrorCorrectingNode::default()),
    ];
    let actual_args = ArgVals::env();
    if actual_args.len() == 0 {
        let merged_docs = merged_arg_docs(nodes.iter().map(|n| n.arg_docs()));
        eprintln!("{merged_docs}");
    } else {
        let running = Arc::new(AtomicCell::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false))?;
        for node in nodes {
            let args = node.arg_docs().get_args_with_defaults();
            let spec = node.spec(&args)?;
            let running = running.clone();
            std::thread::spawn(move || {
                if let Err(e) = spec.run(running) {
                    eprintln!("Error: {e}");
                }
            });
        }
        while running.load() {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
    Ok(())
}
