use bit_slam_nodes::{unguided_nodes::IrHazardDataNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    IrHazardDataNode::default().run()
}
