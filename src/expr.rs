use crossbeam::atomic::AtomicCell;
use futures::StreamExt;
use smol::lock::Mutex;
use std::{pin::Pin, sync::Arc};

use r2r::{
    Context, Node, QosProfile, WrappedTypesupport, irobot_create_msgs::msg::HazardDetectionVector,
    nav_msgs::msg::Odometry,
};

pub fn subscribe<P: WrappedTypesupport + 'static + Send, F: FnMut(P) + Send + 'static>(
    node: Arc<Mutex<Node>>,
    topic: &str,
    handler: F,
) -> anyhow::Result<Pin<Box<dyn Future<Output = ()> + Send + 'static>>> {
    let mut subscriber = {
        let mut guard = smol::block_on(node.lock());
        guard
            .subscribe::<P>(topic, QosProfile::sensor_data())
            .map_err(|e| anyhow::anyhow!("Subscribe failed: {}", e))?
    };

    Ok(Box::pin(async move {
        let mut handler = handler;
        loop {
            if let Some(msg) = subscriber.next().await {
                handler(msg);
            }
        }
    }))
}

pub fn example(period: u64) -> anyhow::Result<()> {
    let odom_topic = "/archangel/odom";
    let hazard_topic = "/archangel/hazard_detection";
    let context = Context::create()?;
    let node = Arc::new(Mutex::new(Node::create(context, "dummy", "")?));
    let mut futures = vec![];
    futures.push(subscribe(node.clone(), odom_topic, |odom: Odometry| {
        println!("{odom:?}");
    })?);
    futures.push(subscribe(
        node.clone(),
        hazard_topic,
        |haz: HazardDetectionVector| {
            println!("{haz:?}");
        },
    )?);
    let running = Arc::new(AtomicCell::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false))?;
    smol::block_on(async {
        for future in futures {
            smol::spawn(future).detach();
        }
        while running.load() {
            let mut guard = smol::block_on(node.lock());
            guard.spin_once(std::time::Duration::from_millis(period));
        }
    });
    Ok(())
}
