use bit_slam_nodes::{node_struct::RunnableNode, bump_avoid_nodes::IrHazardDataNode};

fn main() -> anyhow::Result<()> {
    IrHazardDataNode::default().run()
}
