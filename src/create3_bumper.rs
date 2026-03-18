use r2r::{Node, QosProfile, irobot_create_msgs::msg::HazardDetectionVector, std_msgs::msg::String as Ros2String};

use crate::ros2_node;

fn run_create3_bumper(robot_name: &str) -> anyhow::Result<()> {
    ros2_node!(robot_name, 100, 
        publisher outgoing; Ros2String; "outgoing",
    );
    Ok(())
}

pub struct Create3BumperSpec {
    robot_name: String
}

impl NodeSpec for Create3BumperSpec {
    type Handler = Create3BumperHandlers;

    fn customize(&self, node: &mut Node) {
        let hazard_topic = format!("/{}/hazard_detection", self.robot_name);
        let hazard_subscriber = node.subscribe::<HazardDetectionVector>(hazard_topic.as_str(), QosProfile::sensor_data())?;
    }

    fn handlers(&self) -> impl Iterator<Item=&Self::Handler> {
        todo!()
    }
}