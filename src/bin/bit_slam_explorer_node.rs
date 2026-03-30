use bit_slam_nodes::{bit_slam_nodes::BitSlamExplorerNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    BitSlamExplorerNode::default().run()
}
