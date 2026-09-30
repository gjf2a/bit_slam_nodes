use futures::StreamExt;
use r2r::{Context, Node, QosProfile, irobot_create_msgs::msg::{HazardDetectionVector, IrIntensityVector}, nav_msgs::msg::Odometry, std_msgs::msg::String as Ros2String};

use crate::{bit_slam_nodes::{on_path_topic_name, stop_topic_name}, odom_topic_name};

pub fn test(robot: &str) -> anyhow::Result<()> {
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