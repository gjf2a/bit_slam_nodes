use bit_slam_nodes::{bump_avoid_nodes::IrHazardDataNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    IrHazardDataNode::default().run()
}
