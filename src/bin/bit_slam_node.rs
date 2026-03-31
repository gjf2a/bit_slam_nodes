use bit_slam_nodes::{bit_slam_nodes::BitSlamNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    BitSlamNode::default().run()
}
