pub mod fuzzy;
pub mod fuzzy_nodes;
pub mod nodes;
pub mod util;

const PERIOD: u64 = 100;

use crossbeam::atomic::AtomicCell;
use futures::StreamExt;
use smol::lock::Mutex;
use std::{pin::Pin, sync::Arc};

use r2r::{
    Context, Node, Publisher, QosProfile, WrappedTypesupport,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Twist, TwistStamped, Vector3},
    std_msgs::msg::Header,
};

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

    pub fn subscribe<P: WrappedTypesupport + 'static + Send, F: FnMut(P, &Node) + Send + 'static>(
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

pub fn twist_stamped(node: &Node, x: f64, z: f64) -> anyhow::Result<TwistStamped> {
    let clock = node.get_ros_clock();
    let mut clock = clock.lock().unwrap();
    let now = clock.get_now()?;
    let stamp = Time {
        sec: now.as_secs() as i32,
        nanosec: now.subsec_nanos(),
    };
    Ok(TwistStamped {
        header: Header {
            stamp,
            frame_id: "base_link".to_string(),
        },
        twist: Twist {
            linear: Vector3 { x, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z },
        },
    })
}
