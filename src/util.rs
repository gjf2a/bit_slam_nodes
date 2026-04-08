use particle_filter::Particle;
use particle_filter::{angle::Radians, point::FloatPoint, pose::RobotPose};
use r2r::geometry_msgs::msg::{Point as Ros2Point, Pose, Quaternion};
use r2r::nav_msgs::msg::{MapMetaData, OccupancyGrid, Odometry};
use r2r::{
    Node,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Twist, TwistStamped, Vector3},
    std_msgs::msg::Header,
};

pub fn parse_obstacle_distance_heading(msg: &str) -> anyhow::Result<(f64, Radians)> {
    let fp = msg.parse::<FloatPoint>()?;
    Ok((fp[0], Radians::new(fp[1])))
}

pub fn find_yaw(value: &Odometry) -> Radians {
    find_roll_pitch_yaw(value).2
}

pub fn find_roll_pitch_yaw(value: &Odometry) -> (Radians, Radians, Radians) {
    let (q1, q2, q3, q0) = (
        value.pose.pose.orientation.x,
        value.pose.pose.orientation.y,
        value.pose.pose.orientation.z,
        value.pose.pose.orientation.w,
    );
    (
        Radians::new(
            2.0 * (q0 * q1 + q2 * q3)
                .atan2(q0.powf(2.0) - q1.powf(2.0) - q2.powf(2.0) + q3.powf(2.0)),
        ),
        Radians::new(2.0 * (q0 * q2 - q1 * q3).asin()),
        Radians::new(
            (q0 * q3 + q1 * q2).atan2(q0.powf(2.0) + q1.powf(2.0) - q2.powf(2.0) - q3.powf(2.0)),
        ),
    )
}

pub fn pose_from_odometry(value: &Odometry) -> RobotPose<Radians> {
    let mut result = RobotPose::default();
    result.pos[0] = value.pose.pose.position.x;
    result.pos[1] = value.pose.pose.position.y;
    let theta = find_yaw(value);
    result.theta = theta;
    result
}

pub fn stamped_header(node: &Node) -> anyhow::Result<Header> {
    let clock = node.get_ros_clock();
    let mut clock = clock.lock().unwrap();
    let now = clock.get_now()?;
    let stamp = Time {
        sec: now.as_secs() as i32,
        nanosec: now.subsec_nanos(),
    };
    Ok(Header {
        stamp,
        frame_id: "base_link".to_string(),
    })
}

pub fn twist_stamped(node: &Node, x: f64, z: f64) -> anyhow::Result<TwistStamped> {
    Ok(TwistStamped {
        header: stamped_header(node)?,
        twist: Twist {
            linear: Vector3 { x, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z },
        },
    })
}

pub fn particle2rosgrid(node: &Node, particle: &Particle) -> anyhow::Result<OccupancyGrid> {
    let map = particle.map();
    let bounds = map.bounding_box();
    let header = stamped_header(node)?;
    let origin_corner = map.to_meters(bounds.min());
    let info = MapMetaData {
        map_load_time: header.stamp.clone(),
        width: map.width() as u32,
        height: map.height() as u32,
        resolution: map.square_size_m() as f32,
        origin: Pose {
            position: Ros2Point {
                x: origin_corner[0],
                y: origin_corner[1],
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        },
    };
    Ok(OccupancyGrid {
        header,
        info,
        data: bounds
            .col_major_coord_iter()
            .map(|p| {
                if map.all_obstacles().contains(&p) {
                    1
                } else if map.all_spaces().contains(&p) {
                    0
                } else {
                    -1
                }
            })
            .collect(),
    })
}
