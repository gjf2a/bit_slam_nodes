use bit_slam_nodes::{bit_slam_nodes::ScanObstacleNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    ScanObstacleNode::default().run()
}
