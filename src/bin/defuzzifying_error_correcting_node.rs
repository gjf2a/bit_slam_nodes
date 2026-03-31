use bit_slam_nodes::{fuzzy_nodes::DefuzzifyingErrorCorrectingNode, node_struct::RunnableNode};

fn main() -> anyhow::Result<()> {
    DefuzzifyingErrorCorrectingNode::default().run()
}
