pub mod bit_slam_nodes;
pub mod fuzzy;
pub mod fuzzy_nodes;
pub mod node_struct;
pub mod util;

const PERIOD: u64 = 100;

pub fn odom_topic_name(robot: &str) -> String {
    format!("{robot}/odom")
}
