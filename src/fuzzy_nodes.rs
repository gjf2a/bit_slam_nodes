use std::sync::Arc;

use crate::{
    NodeSpec, PERIOD, fuzzy::{FuzzySet, FuzzyVar}, twist_stamped, util::find_yaw
};
use arg_vals::ArgVals;
use bit_grid::{
    angle::Radians,
    point::{FloatPoint, Point},
    pt,
};
use r2r::{
    Node,
    geometry_msgs::msg::{Point as Ros2Point, TwistStamped},
    nav_msgs::msg::Odometry,
    std_msgs::msg::String as Ros2String,
};
use serde::{Deserialize, Serialize};
use smol::lock::Mutex;

const DISTANCE_LIMIT: f64 = 0.25;
const ANGLE_LIMIT: f64 = 0.2;
const X_LIMIT: f64 = 0.5;
const Z_LIMIT: f64 = 1.0;

pub fn fuzzified_goal_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_goal_error")
}

pub fn goal_fuzzifier_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let robot_name = args.get_symbol(0);
    let mut spec = NodeSpec::new(&format!("{robot_name}_goal_fuzzifier_node"), PERIOD)?;
    let goal_topic = args.get_str_value("-fuzzy_goal_topic").unwrap();
    let publisher = spec.publisher::<Ros2String>(&fuzzified_goal_topic_name(robot_name))?;
    let goal = Arc::new(Mutex::new(None));
    let set_goal = goal.clone();
    spec.subscribe(goal_topic, move |msg: Ros2Point| {
        let mut goal = smol::block_on(set_goal.lock());
        *goal = Some(pt!(msg.x, msg.y));
    })?;
    spec.subscribe(&format!("{robot_name}/odom"), move |odom: Odometry| {
        let odom_point = pt!(odom.pose.pose.position.x, odom.pose.pose.position.y);
        let yaw = find_yaw(&odom);
        if let Some(goal) = *smol::block_on(goal.lock()) {
            let error = FuzzyError::new(&odom_point, &yaw, &goal);
            match serde_json::to_string(&error) {
                Ok(data) => {
                    if let Err(e) = publisher.publish(&Ros2String { data }) {
                        eprintln!("Error {e} publishing message {error:?}");
                    }
                }
                Err(e) => eprintln!("Error {e} sending {error:?}"),
            }
        }
    })?;
    Ok(spec)
}

pub fn defuzzifying_error_correcting_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let robot_name = args.get_symbol(0);
    let mut spec = NodeSpec::new(&format!("{robot_name}_defuzz_error_node"), PERIOD)?;
    let fuzzy_topic = fuzzified_goal_topic_name(robot_name);
    let motor_topic = format!("{robot_name}/cmd_vel_stamped");
    let publisher = spec.publisher::<TwistStamped>(&motor_topic)?;
    let node = Arc::downgrade(&spec.node());
    spec.subscribe(
        &fuzzy_topic,
        move |msg: Ros2String| match serde_json::from_str::<FuzzyError>(&msg.data) {
            Ok(fuzzy_error) => {
                if let Some(node) = node.upgrade() {
                    let node = smol::block_on(node.lock());
                    match fuzzy_error.defuzzify_twist_stamped(&node) {
                        Ok(msg) => if let Err(e) = publisher.publish(&msg) {
                            eprintln!("Error {e} when publishing {msg:?}");
                        }
                        Err(e) => eprintln!("Error {e} when defuzzifying {fuzzy_error:?}")
                    }
                }
            }
            Err(e) => eprintln!("Error {e}: could not parse {}", msg.data),
        },
    )?;
    Ok(spec)
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct FuzzyError {
    left: FuzzyVar,
    right: FuzzyVar,
    distance: FuzzyVar,
}

impl FuzzyError {
    pub fn new(odom_point: &FloatPoint, yaw: &Radians, goal: &FloatPoint) -> Self {
        let diff = *goal - *odom_point;
        let (distance, goal_direction): (f64, Radians) = diff.into();
        let angle_diff = f64::from(goal_direction - *yaw);

        let fuzzifier = FuzzySet::Rising(0.0, ANGLE_LIMIT);
        let (left, right) = if angle_diff > 0.0 {
            (fuzzifier.fuzzify(angle_diff), FuzzyVar::new(0.0))
        } else {
            (FuzzyVar::new(0.0), fuzzifier.fuzzify(-angle_diff))
        };
        let distance = FuzzySet::Rising(0.0, DISTANCE_LIMIT / 2.0).fuzzify(distance);
        Self {
            left,
            right,
            distance: distance & !(left | right),
        }
    }

    pub fn defuzzify_twist_stamped(&self, node: &Node) -> anyhow::Result<TwistStamped> {
        let x = self.distance.defuzzify(0.0, X_LIMIT);
        let turn_limit = Z_LIMIT * (if self.left > self.right {1.0} else {-1.0});
        let z = (self.left | self.right).defuzzify(0.0, turn_limit);
        twist_stamped(node, x, z)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use bit_grid::{angle::Radians, point::Point, pt};

    use crate::{fuzzy::FuzzyVar, fuzzy_nodes::FuzzyError};

    #[test]
    fn test_distance_goal_offsets() {
        for (
            (odom_x, odom_y),
            yaw,
            (goal_x, goal_y),
            distance,
            goal_direction,
            normalized_angle,
            (f_left, f_right, f_distance),
        ) in [
            (
                (-1.0, 1.0),
                PI / 3.0,
                (2.0, 3.0),
                3.605551275463989,
                0.5880026035475675,
                -0.4591949476490301,
                (0.0, 1.0, 0.0),
            ),
            (
                (-1.0, 1.0),
                PI / 6.0,
                (2.0, 3.0),
                3.605551275463989,
                0.5880026035475675,
                0.06440382794926869,
                (0.32201913974634344, 0.0, 0.6779808602536566),
            ),
        ] {
            let odom = pt!(odom_x, odom_y);
            let goal = pt!(goal_x, goal_y);
            let diff = goal - odom;
            assert_eq!(distance, f64::from(diff));
            let angle_diff = Radians::from(diff);
            assert_eq!(Radians::new(goal_direction), angle_diff);
            let yaw = Radians::new(yaw);
            assert_eq!(Radians::new(normalized_angle), angle_diff - yaw);
            let expected_error = FuzzyError {
                left: FuzzyVar::new(f_left),
                right: FuzzyVar::new(f_right),
                distance: FuzzyVar::new(f_distance),
            };
            assert_eq!(expected_error, FuzzyError::new(&odom, &yaw, &goal));
        }
    }
}
