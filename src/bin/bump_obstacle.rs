use bit_slam_nodes::NodeSpec;
use particle_filter_create3::Bump;
use r2r::{irobot_create_msgs::msg::HazardDetectionVector, std_msgs::msg::String as Ros2String};
use std::env;

fn main() -> anyhow::Result<()> {
    let args = env::args().collect::<Vec<_>>();
    let robot_name = if args.len() == 2 {
        format!("/{}", args[1])
    } else {
        "".to_string()
    };
    let hazard_topic = format!("{robot_name}/hazard_detection");
    let mut spec = NodeSpec::new("BumpObstacle", 100)?;
    let publish_topic = format!("{robot_name}_obstacles");
    let publisher = spec.publisher::<Ros2String>(&publish_topic)?;
    println!("Publishing on {publish_topic}");
    spec.subscribe(&hazard_topic, move |hazards: HazardDetectionVector| {
        for detection in hazards.detections {
            let name = detection.header.frame_id.as_str();
            if let Ok(bump) = name.parse::<Bump>() {
                let (distance, heading) = bump.obstacle_at();
                let heading: f64 = heading.into();
                let msg = Ros2String {
                    data: format!("({distance},{heading})"),
                };
                if let Err(e) = publisher.publish(&msg) {
                    eprintln!("Error publishing {msg:?}: {e}");
                }
            }
        }
    })?;
    spec.run()?;
    Ok(())
}
