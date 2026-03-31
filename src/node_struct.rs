use arg_vals::{ArgDocs, ArgVals};
use crossbeam::atomic::AtomicCell;
use futures::StreamExt;
use smol::lock::Mutex;
use std::{pin::Pin, sync::Arc};

use r2r::{Context, Node, Publisher, QosProfile, WrappedTypesupport};

pub trait RunnableNode {
    fn arg_docs(&self) -> &ArgDocs;
    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()>;
    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>>;
    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>>;
    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec>;
    fn run(&self) -> anyhow::Result<()> {
        let arg_docs = self.arg_docs();
        let args = arg_docs.get_args_with_defaults();
        eprintln!("Publishing on: {:?}", self.publishing_topics(&args));
        eprintln!("Subscribing to: {:?}", self.subscribing_topics(&args));
        let actual_args = ArgVals::env();
        if actual_args.len() == 0 {
            eprintln!("{arg_docs}");
            Ok(())
        } else {
            let spec = self.spec(&args)?;
            spec.run()
        }
    }
}

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

    pub fn node(&self) -> Arc<Mutex<Node>> {
        self.node.clone()
    }

    pub fn publisher<P: WrappedTypesupport>(&self, topic: &str) -> anyhow::Result<Publisher<P>> {
        let mut node = smol::block_on(self.node.lock());
        Ok(node.create_publisher::<P>(topic, QosProfile::sensor_data())?)
    }

    pub fn subscribe<
        P: WrappedTypesupport + 'static + Send,
        F: FnMut(P, &Node) + Send + 'static,
    >(
        &mut self,
        topic: &str,
        handler: F,
    ) -> anyhow::Result<()> {
        let mut subscriber = {
            let mut node = smol::block_on(self.node.lock());
            node.subscribe::<P>(topic, QosProfile::sensor_data())
                .map_err(|e| anyhow::anyhow!("Subscribe failed: {}", e))?
        };
        let node = self.node.clone();
        self.futures.push(Box::pin(async move {
            let mut handler = handler;
            loop {
                if let Some(msg) = subscriber.next().await {
                    let node = smol::block_on(node.lock());
                    handler(msg, &node);
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
