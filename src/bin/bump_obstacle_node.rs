use bit_slam_nodes::{bit_slam_nodes::BumpIrObstacleNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    BumpIrObstacleNode::default().run()
}
