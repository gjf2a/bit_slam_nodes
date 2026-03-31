use bit_slam_nodes::{
    bit_slam_nodes::{BitSlamExplorerNode, BitSlamNode, BumpObstacleNode},
    fuzzy_nodes::{DefuzzifyingErrorCorrectingNode, GoalFuzzifierNode},
    node_struct::RunnableNode,
};

fn main() -> anyhow::Result<()> {
    let explorer = BitSlamExplorerNode::default();
    let mut goal_fuzzifier = GoalFuzzifierNode::default();
    goal_fuzzifier.add_default("--fuzzy-goal-topic", &explorer.goal_publish_topic(&explorer.args().get_args_with_defaults())?)?;
    let nodes: Vec<Box<dyn RunnableNode>> = vec![
        Box::new(BitSlamNode::default()),
        Box::new(explorer),
        Box::new(BumpObstacleNode::default()),
        Box::new(goal_fuzzifier),
        Box::new(DefuzzifyingErrorCorrectingNode::default()),
    ];

    Ok(())
}
