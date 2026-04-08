pub mod bit_slam_nodes;
pub mod fuzzy;
pub mod fuzzy_nodes;
pub mod node_struct;
pub mod util;

const PERIOD: u64 = 100;

pub fn odom_topic_name(robot: &str) -> String {
    format!("{robot}/odom")
}

#[macro_export]
macro_rules! robot_name {
    ($args:ident) => {
        $args
            .get_str_value("--robot")
            .map_or("robot_name", |rn| rn.as_str())
    };
}
