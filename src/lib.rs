pub mod create3_bumper;
pub mod mapper;

use crossbeam::atomic::AtomicCell;
use r2r::{Context, Node};
use std::sync::Arc;

pub trait NodeHandler {
    fn run(&self) -> impl Future<Output = ()> + Send + 'static;
}

pub trait NodeSpec {
    type Handler : NodeHandler;

    fn customize(&self, node: &mut Node);
    fn handlers(&self) -> impl Iterator<Item=&Self::Handler>;

    fn runner(&self, node_name: &str, period: u64) -> anyhow::Result<()> {
        let context = Context::create()?;
        let mut node = Node::create(context, node_name, "")?;
        self.customize(&mut node);

        let running = Arc::new(AtomicCell::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false))?;
        smol::block_on(async {
            for handler in self.handlers() {
                smol::spawn(handler.run()).detach();
            }
            while running.load() {
                node.spin_once(std::time::Duration::from_millis(period));
            }
        });

        Ok(())
    }
}

