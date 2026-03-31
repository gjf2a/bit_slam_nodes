use bit_slam_nodes::{fuzzy_nodes::GoalFuzzifierNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    GoalFuzzifierNode::default().run()
}
