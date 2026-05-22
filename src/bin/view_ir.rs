use bit_slam_nodes::{information_only_nodes::ShowIrNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    ShowIrNode::default().run()
}