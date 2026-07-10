use std::sync::Arc;

use chrono::Local;
use particle_filter::angle::Angle;
use particle_filter::{BitGridMap, MapInput, Particle};
use particle_filter::{angle::Radians, pose::RobotPose};
use r2r::Publisher;
use r2r::geometry_msgs::msg::{Point as Ros2Point, Pose, Quaternion};
use r2r::nav_msgs::msg::{MapMetaData, OccupancyGrid, Odometry};
use r2r::{
    Node,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Twist, TwistStamped, Vector3},
    std_msgs::msg::{Header, String as Ros2String},
};
use smol::lock::Mutex;

pub fn ros2_node_name(robot: &str, concept: &str) -> String {
    if robot.len() == 0 {
        concept.to_string()
    } else {
        format!("{robot}_{concept}")
    }
}

pub fn ros2_topic_name(robot: &str, concept: &str) -> String {
    if robot.len() == 0 {
        concept.to_string()
    } else {
        format!("{robot}/{concept}")
    }
}

pub fn publish_str(publisher: &Publisher<Ros2String>, s: String) -> anyhow::Result<()> {
    let msg = Ros2String { data: s };
    publisher.publish(&msg)?;
    Ok(())
}

pub fn timestamped_filename(prefix: &str) -> String {
    let now = Local::now();
    format!("{prefix}_{}.json", now.format("%Y_%m_%d_%H_%M_%S"))
}

pub fn find_yaw(value: &Odometry) -> Radians {
    find_roll_pitch_yaw(value).2
}

pub fn find_roll_pitch_yaw(value: &Odometry) -> (Radians, Radians, Radians) {
    quaternion2roll_pitch_yaw(&value.pose.pose.orientation)
}

fn quaternion2roll_pitch_yaw(q: &Quaternion) -> (Radians, Radians, Radians) {
    let (q1, q2, q3, q0) = (q.x, q.y, q.z, q.w);
    (
        Radians::new(
            2.0 * (q0 * q1 + q2 * q3)
                .atan2(q0.powf(2.0) - q1.powf(2.0) - q2.powf(2.0) + q3.powf(2.0)),
        ),
        Radians::new(2.0 * (q0 * q2 - q1 * q3).asin()),
        Radians::new(
            (2.0 * (q0 * q3 + q1 * q2))
                .atan2(q0.powf(2.0) + q1.powf(2.0) - q2.powf(2.0) - q3.powf(2.0)),
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

pub fn stamped_header(node: Arc<Mutex<Node>>) -> anyhow::Result<Header> {
    let node = smol::block_on(node.lock());
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

pub fn publish_map_input(
    map_input: &MapInput,
    publisher: &Publisher<Ros2String>,
    node: Arc<Mutex<Node>>,
) -> anyhow::Result<()> {
    let stamped_str = StampedString {
        header: stamped_header(node)?,
        data: format!("{map_input}"),
    };
    serde_json::to_string(&stamped_str)
        .map_err(anyhow::Error::from)
        .and_then(|json| publish_str(&publisher, json))
}

pub fn twist_stamped(
    node: Arc<Mutex<Node>>,
    forward: f64,
    leftward_angular: f64,
) -> anyhow::Result<TwistStamped> {
    Ok(TwistStamped {
        header: stamped_header(node)?,
        twist: Twist {
            linear: Vector3 {
                x: forward,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: leftward_angular,
            },
        },
    })
}

pub fn particle2rosgrid(
    node: Arc<Mutex<Node>>,
    particle: &Particle,
) -> anyhow::Result<OccupancyGrid> {
    let header = stamped_header(node)?;
    let info = map_meta_data(&header, particle.map());
    Ok(OccupancyGrid {
        header,
        info,
        data: occupancy_grid_vec(particle.map()),
    })
}

fn map_meta_data(header: &Header, map: &BitGridMap) -> MapMetaData {
    let origin_corner = map.to_meters(map.bounding_box().min());
    MapMetaData {
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
                w: 1.0,
            },
        },
    }
}

// Guidelines: https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html
fn occupancy_grid_vec(map: &BitGridMap) -> Vec<i8> {
    map.bounding_box()
        .row_major_coord_iter()
        .map(|p| {
            if map.all_obstacles().contains(&p) {
                1
            } else if map.all_spaces().contains(&p) {
                0
            } else {
                -1
            }
        })
        .collect()
}

pub fn pose2ros2pose(pose: &RobotPose<Radians>) -> Pose {
    Pose {
        position: Ros2Point {
            x: pose.pos[0],
            y: pose.pos[1],
            z: 0.0,
        },
        orientation: yaw2quaternion(pose.theta),
    }
}

pub fn yaw2quaternion(yaw: Radians) -> Quaternion {
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: (yaw / 2.0).sin(),
        w: (yaw / 2.0).cos(),
    }
}

pub fn time_less_than(t1: &Time, t2: &Time) -> bool {
    t1.sec < t2.sec || (t1.sec == t2.sec && t1.nanosec < t2.nanosec)
}

pub fn add_time_ns(time: &Time, ns: u32) -> Time {
    let ns = ns + time.nanosec;
    let bonus_sec = ns / 1_000_000_000;
    let remainder = ns % 1_000_000_000;
    Time {
        sec: time.sec + bonus_sec as i32,
        nanosec: remainder,
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StampedString {
    pub header: r2r::std_msgs::msg::Header,
    pub data: String,
}

#[cfg(test)]
mod tests {
    use crate::util::{
        add_time_ns, occupancy_grid_vec, quaternion2roll_pitch_yaw, time_less_than, yaw2quaternion,
    };
    use assert_eq_float::*;
    use particle_filter::{
        BitGridMap,
        angle::{Angle, Degrees},
        point::GridPoint,
    };
    use r2r::builtin_interfaces::msg::Time;

    #[test]
    fn test_occupancy_grid() {
        let map =
            BitGridMap::from_map_inputs(0.1, 0.2, "odometry_staircase_1000_steps.mi").unwrap();
        let occupancy_grid = occupancy_grid_vec(&map);
        assert_eq!(occupancy_grid.len(), map.bounding_box().area() as usize);
        for y in 0..map.bounding_box().height() {
            for x in 0..map.bounding_box().width() {
                let i = (y * map.bounding_box().width() + x) as usize;
                let start = map.bounding_box().min();
                let mapped = GridPoint::new([x as i64, y as i64]) + start;
                let expected = match map.cell_for(&mapped) {
                    particle_filter::Cell::Obstacle => 1,
                    particle_filter::Cell::Space => 0,
                    particle_filter::Cell::Unvisited => -1,
                    particle_filter::Cell::Inconsistent => 1,
                };
                assert_eq!(occupancy_grid[i], expected);
            }
        }
    }

    #[test]
    fn test_yaw_quaternion() {
        for yaw in 0..360 {
            let yaw = Degrees::new(yaw as f64).radians();
            let q = yaw2quaternion(yaw);
            let back = quaternion2roll_pitch_yaw(&q);
            assert_eq_float!(f64::from(yaw), f64::from(back.2));
        }
    }

    #[test]
    fn test_time() {
        let t1 = Time {
            sec: 10,
            nanosec: 20,
        };
        let t2 = Time {
            sec: 10,
            nanosec: 30,
        };
        let t3 = Time {
            sec: 11,
            nanosec: 10,
        };
        assert!(time_less_than(&t1, &t2));
        assert!(time_less_than(&t1, &t3));
        assert!(time_less_than(&t2, &t3));
        assert!(!time_less_than(&t2, &t1));
        assert!(!time_less_than(&t3, &t1));
        assert!(!time_less_than(&t3, &t2));

        let t4 = add_time_ns(&t1, 10);
        assert_eq!(t2, t4);

        let t5 = add_time_ns(&t1, 1_000_000_000 - 10);
        assert_eq!(t3, t5);

        let t6 = Time {
            sec: 14,
            nanosec: 30,
        };
        let t7 = add_time_ns(&t1, 4_000_000_010);
        assert_eq!(t6, t7);
    }
}
