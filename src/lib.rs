pub mod create3_bumper;
pub mod mapper;

#[macro_export]
macro_rules! ros2_node {
    ($node:ident, $period:expr, $($kw:ident $($args:tt)+),+) => {
        let context = r2r::Context::create()?;
        let mut node = r2r::Node::create(context, $node, "")?;
        let mut closures = vec![];

        $(
            $crate::__ros2_dispatch! {@$kw $( $args )*}
        )*

        let running = std::sync::Arc::new(crossbeam::atomic::AtomicCell::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false))?;

        smol::block_on(async {
            for handler in closures {
                smol::spawn(handler()).detach();
            }
            while running.load() {
                node.spin_once(std::time::Duration::from_millis($period));
            }
        });
    }
}

#[macro_export]
macro_rules! __ros2_dispatch {
    (@publisher $pub_name:ident; $ros_type: ident; $topic:expr) => {
        let $pub_name = node.create_publisher::<$ros_type>($topic, r2r::QosProfile::sensor_data())?;
    };

    (@subscriber $sub_name:ident; $ros_type: ident; $topic:expr; $handler:block) => {
        let $sub_name = node.subscribe::<$ros_type>($topic, r2r::QosProfile::sensor_data())?;
        closures.push(($sub_name, Box::new(async || {
            loop {
                if let Some(msg) = $sub_name.next().await {
                    $handler
                }
            }
        })));
    }
}

/*
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

*/