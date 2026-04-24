use bit_slam_nodes::{node_struct::RunnableNode, unguided_nodes::IrHazardDataNode};

fn main() -> anyhow::Result<()> {
    IrHazardDataNode::default().run()
}
