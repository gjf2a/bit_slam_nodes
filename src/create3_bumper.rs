use crate::{NodeHandler, NodeSpec};
use r2r::{Node, QosProfile, irobot_create_msgs::msg::HazardDetectionVector};

pub struct Create3BumperHandlers {

}

impl NodeHandler for Create3BumperHandlers {
    fn run(&self) -> impl Future<Output = ()> + Send + 'static {
        async {
            todo!();
        }
    }
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