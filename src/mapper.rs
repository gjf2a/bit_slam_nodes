use r2r::Node;

use crate::{NodeHandler, NodeSpec};

pub struct BitSlamNode {

}

pub struct BitSlamHandlers {

}

impl NodeHandler for BitSlamHandlers {
    fn run(&self) -> impl Future<Output = ()> + Send + 'static {
        async {
            todo!();
        }
    }
}

impl NodeSpec for BitSlamNode {
    type Handler = BitSlamHandlers;

    fn customize(&self, node: &mut Node) {
        todo!()
    }

    fn handlers(&self) -> impl Iterator<Item=&Self::Handler> {
        todo!()
    }
}