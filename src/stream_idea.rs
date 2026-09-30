use futures::{Stream, StreamExt, future::Fuse};
use r2r::{Context, Node, QosProfile, WrappedTypesupport, irobot_create_msgs::msg::{HazardDetectionVector, IrIntensityVector}, nav_msgs::msg::Odometry, std_msgs::msg::String as Ros2String};

use crate::{bit_slam_nodes::{on_path_topic_name, stop_topic_name}, odom_topic_name};

// First compiling version
pub fn test1(robot: &str) -> anyhow::Result<()> {
    let context = Context::create()?;
    let mut node = Node::create(context, "test_node", robot)?;
    let mut sub_hazard = node.subscribe::<HazardDetectionVector>(&format!("{robot}/hazard_detection"), QosProfile::sensor_data())?.fuse();
    let mut sub_ir = node.subscribe::<IrIntensityVector>(&format!("{robot}/ir_intensity"), QosProfile::sensor_data())?.fuse();
    let mut sub_odom = node.subscribe::<Odometry>(&odom_topic_name(robot), QosProfile::sensor_data())?.fuse();
    let mut sub_stop = node.subscribe::<Ros2String>(&stop_topic_name(robot), QosProfile::sensor_data())?.fuse();
    let mut sub_on_path = node.subscribe::<Ros2String>(&on_path_topic_name(robot), QosProfile::sensor_data())?.fuse();
    smol::block_on(async {
        loop {
            futures::select! {
                hazard = sub_hazard.select_next_some() => println!("hazard"),
                ir = sub_ir.select_next_some() => println!("ir"),
                odom = sub_odom.select_next_some() => println!("odom"),
                stop = sub_stop.select_next_some() => println!("stop"),
                on_path = sub_on_path.select_next_some() => println!("on_path"),
                complete => break
            }
        }
    });
    
    Ok(())
}

/* 
// I am preserving this attempt so I remember why I had to define a macro.
// In the run() method, it flags that self has been mutably borrowed more than once.

pub struct TestNode {
    node: Node,
    robot: String,
}

impl TestNode {
    pub fn new(robot: &str) -> anyhow::Result<Self> {
        let context = Context::create()?;
        Ok(Self {node: Node::create(context, "test_node", robot)?, robot: robot.to_string()})    
    }

    pub fn topic(&self, topic: &str) -> String {
        format!("{}/{topic}", self.robot)
    }

    pub fn subscribe<T: WrappedTypesupport + 'static>(&mut self, topic: &str) -> r2r::Result<impl Stream<Item=T>> {
        let topic = format!("{}/{topic}", self.robot);
        self.node.subscribe::<T>(&topic, QosProfile::sensor_data())
    }

    pub fn run(&mut self) -> anyhow::Result<()> {
        let mut sub_hazard = self.subscribe::<HazardDetectionVector>("hazard_detection")?;
        let mut sub_ir = self.subscribe::<IrIntensityVector>("ir_intensity")?;
        let mut sub_odom = self.subscribe::<Odometry>("odom")?;

        Ok(())
    }
}
*/

// Second compiling version
macro_rules! subscribe {
    ($node:expr, $sub_type:ty, $topic:expr) => {
        $node.subscribe::<$sub_type>($topic, QosProfile::sensor_data())?.fuse()
    };
}

pub fn test2(robot: &str) -> anyhow::Result<()> {
    let context = Context::create()?;
    let mut node = Node::create(context, "test_node", robot)?;
    let mut sub_hazard = subscribe!(node, HazardDetectionVector, &format!("{robot}/hazard_detection"));
    let mut sub_ir = subscribe!(node, IrIntensityVector, &format!("{robot}/ir_intensity"));
    let mut sub_odom = subscribe!(node, Odometry, &odom_topic_name(robot));
    let mut sub_stop = subscribe!(node, Ros2String, &stop_topic_name(robot));
    let mut sub_on_path = subscribe!(node, Ros2String, &on_path_topic_name(robot));
    smol::block_on(async {
        loop {
            futures::select! {
                hazard = sub_hazard.select_next_some() => println!("hazard"),
                ir = sub_ir.select_next_some() => println!("ir"),
                odom = sub_odom.select_next_some() => println!("odom"),
                stop = sub_stop.select_next_some() => println!("stop"),
                on_path = sub_on_path.select_next_some() => println!("on_path"),
                complete => break
            }
        }
    });
    
    Ok(())
}

// Third compiling version

#[derive(Default)]
pub struct AllMessages {
    hazards: Vec<HazardDetectionVector>,
    irs: Vec<IrIntensityVector>,
    odoms: Vec<Odometry>,
    stops: Vec<Ros2String>,
    on_paths: Vec<Ros2String>,
}

// I'm not actually spinning the node - this is bad.
pub fn test3(robot: &str) -> anyhow::Result<()> {
    let context = Context::create()?;
    let mut node = Node::create(context, "test_node", robot)?;
    let mut sub_hazard = subscribe!(node, HazardDetectionVector, &format!("{robot}/hazard_detection"));
    let mut sub_ir = subscribe!(node, IrIntensityVector, &format!("{robot}/ir_intensity"));
    let mut sub_odom = subscribe!(node, Odometry, &odom_topic_name(robot));
    let mut sub_stop = subscribe!(node, Ros2String, &stop_topic_name(robot));
    let mut sub_on_path = subscribe!(node, Ros2String, &on_path_topic_name(robot));
    let mut messages = AllMessages::default();
    smol::block_on(async {
        loop {
            futures::select! {
                hazard = sub_hazard.select_next_some() => messages.hazards.push(hazard),
                ir = sub_ir.select_next_some() => messages.irs.push(ir),
                odom = sub_odom.select_next_some() => messages.odoms.push(odom),
                stop = sub_stop.select_next_some() => messages.stops.push(stop),
                on_path = sub_on_path.select_next_some() => messages.on_paths.push(on_path),
                complete => break
            }
        }
    });
    
    Ok(())
}


// Vision for where I ideally would like to go
//
// A programmer creates a struct with a handler for each subscription.
// The programmer annotates each handler with the topic name and
// the data type.
// A macro of some kind generates all of the node setup and subscription
// code.