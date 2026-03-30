use bit_slam_nodes::{bit_slam_nodes::BumpObstacleNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    BumpObstacleNode::default().run()
}
