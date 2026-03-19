use bit_grid::{angle::Radians, point::FloatPoint, pose::RobotPose};
use r2r::nav_msgs::msg::Odometry;

pub fn obstacle_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_obstacles")
}

pub fn parse_obstacle_distance_heading(msg: &str) -> anyhow::Result<(f64, Radians)> {
    let fp = msg.parse::<FloatPoint>()?;
    Ok((fp[0], Radians::new(fp[1])))
}

pub fn find_roll_pitch_yaw(value: &Odometry) -> (Radians, Radians, Radians) {
    let (q1, q2, q3, q0) = (
        value.pose.pose.orientation.x,
        value.pose.pose.orientation.y,
        value.pose.pose.orientation.z,
        value.pose.pose.orientation.w,
    );
    (
        Radians::new(2.0 * (q0 * q1 + q2 * q3)
            .atan2(q0.powf(2.0) - q1.powf(2.0) - q2.powf(2.0) + q3.powf(2.0))),
        Radians::new(2.0 * (q0 * q2 - q1 * q3).asin()),
        Radians::new((q0 * q3 + q1 * q2).atan2(q0.powf(2.0) + q1.powf(2.0) - q2.powf(2.0) - q3.powf(2.0))),
    )
}

pub fn pose_from_odometry(value: &Odometry) -> RobotPose<Radians> {
    let mut result = RobotPose::default();
    result.pos[0] = value.pose.pose.position.x;
    result.pos[1] = value.pose.pose.position.y;
    let (_, _, theta) = find_roll_pitch_yaw(value);
    result.theta = theta;
    result
}