use crossbeam::atomic::AtomicCell;
use futures::StreamExt;
use smol::lock::Mutex;
use std::{pin::Pin, sync::Arc};

use r2r::{Context, Node, Publisher, QosProfile, WrappedTypesupport};

pub struct NodeSpec {
    node: Arc<Mutex<Node>>,
    futures: Vec<Pin<Box<dyn Future<Output = ()> + Send + 'static>>>,
    period: u64,
}

impl NodeSpec {
    pub fn new(node_name: &str, period: u64) -> anyhow::Result<Self> {
        let context = Context::create()?;
        let node = Arc::new(Mutex::new(Node::create(context, node_name, "")?));
        Ok(Self {
            node,
            futures: vec![],
            period,
        })
    }

    pub fn publisher<P: WrappedTypesupport>(&self, topic: &str) -> anyhow::Result<Publisher<P>> {
        let mut node = smol::block_on(self.node.lock());
        Ok(node.create_publisher::<P>(topic, QosProfile::sensor_data())?)
    }

    pub fn subscribe<P: WrappedTypesupport + 'static + Send, F: FnMut(P) + Send + 'static>(
        &mut self,
        topic: &str,
        handler: F,
    ) -> anyhow::Result<()> {
        let mut subscriber = {
            let mut node = smol::block_on(self.node.lock());
            node.subscribe::<P>(topic, QosProfile::sensor_data())
                .map_err(|e| anyhow::anyhow!("Subscribe failed: {}", e))?
        };
        self.futures.push(Box::pin(async move {
            let mut handler = handler;
            loop {
                if let Some(msg) = subscriber.next().await {
                    handler(msg);
                }
            }
        }));
        Ok(())
    }

    pub fn run(self) -> anyhow::Result<()> {
        let running = Arc::new(AtomicCell::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false))?;
        smol::block_on(async {
            for future in self.futures {
                smol::spawn(future).detach();
            }
            while running.load() {
                let mut node = smol::block_on(self.node.lock());
                node.spin_once(std::time::Duration::from_millis(self.period));
            }
        });
        Ok(())
    }
}
