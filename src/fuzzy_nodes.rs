use std::sync::Arc;

use crate::{
    PERIOD,
    fuzzy::{FuzzySet, FuzzyVar},
    node_struct::{NodeSpec, RunnableNode},
    odom_topic_name,
    util::{find_yaw, twist_stamped},
};
use arg_vals::{ArgDocs, ArgVals};
use bit_grid::{
    angle::Radians,
    point::{FloatPoint, Point},
    pt,
};
use r2r::{
    Node, Publisher,
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

pub struct GoalFuzzifierNode {
    docs: ArgDocs,
}

impl Default for GoalFuzzifierNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "goal_fuzzifier_node",
                &vec![
                    ("--robot", "str", ""),
                    ("--fuzzy-goal-topic", "str", ""),
                    ("--reset-topic", "str", ""),
                ],
            ),
        }
    }
}

impl RunnableNode for GoalFuzzifierNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&format!("{robot}_goal_fuzzifier_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let publisher = spec.publisher::<Ros2String>(&pubs[0])?;
        let goal = Arc::new(Mutex::new(None));
        let set_goal = goal.clone();
        spec.subscribe(&subs[0], move |msg: Ros2Point, _| {
            let mut goal = smol::block_on(set_goal.lock());
            *goal = Some(pt!(msg.x, msg.y));
        })?;
        let goal_goal = goal.clone();
        spec.subscribe(&subs[1], move |odom: Odometry, _| {
            if let Some(goal) = *smol::block_on(goal_goal.lock()) {
                if let Err(e) = publish_odom_fuzzy(&odom, &goal, &publisher) {
                    eprintln!("Error {e} when fuzzifying odometry {odom:?} to {goal}");
                }
            }
        })?;
        spec.subscribe(&subs[2], move |msg: Ros2String, _| {
            if msg.data.to_lowercase() == "stop" {
                let mut goal = smol::block_on(goal.lock());
                *goal = None;
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = args.get_str_value("--robot")?;
        Ok(vec![fuzzified_goal_topic_name(robot)])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = args.get_str_value("--robot")?;
        Ok(vec![
            args.get_str_value("--fuzzy_goal_topic")?.clone(),
            odom_topic_name(robot),
            args.get_str_value("--reset-topic")?.clone(),
        ])
    }

    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()> {
        self.docs.set_default(param, param_default)
    }
}

fn publish_odom_fuzzy(
    odom: &Odometry,
    goal: &FloatPoint,
    publisher: &Publisher<Ros2String>,
) -> anyhow::Result<()> {
    let odom_point = pt!(odom.pose.pose.position.x, odom.pose.pose.position.y);
    let yaw = find_yaw(&odom);
    let error = FuzzyError::new(&odom_point, &yaw, &goal);
    let data = serde_json::to_string(&error)?;
    publisher.publish(&Ros2String { data })?;
    Ok(())
}

pub struct DefuzzifyingErrorCorrectingNode {
    docs: ArgDocs,
}

impl Default for DefuzzifyingErrorCorrectingNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new("goal_fuzzifier_node", &vec![("--robot", "str", "")]),
        }
    }
}

impl RunnableNode for DefuzzifyingErrorCorrectingNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&format!("{robot}_defuzz_error_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let publisher = spec.publisher::<TwistStamped>(&pubs[0])?;
        spec.subscribe(&subs[0], move |msg: Ros2String, node| {
            if let Err(e) = publish_fuzzy_twist(&msg, &node, &publisher) {
                eprintln!("Error {e} when publishing {}", msg.data);
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = args.get_str_value("--robot")?;
        Ok(vec![format!("{robot}/cmd_vel_stamped")])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = args.get_str_value("--robot")?;
        Ok(vec![fuzzified_goal_topic_name(robot)])
    }

    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()> {
        self.docs.set_default(param, param_default)
    }
}

fn publish_fuzzy_twist(
    msg: &Ros2String,
    node: &Node,
    publisher: &Publisher<TwistStamped>,
) -> anyhow::Result<()> {
    let fuzzy_error = serde_json::from_str::<FuzzyError>(&msg.data)?;
    let msg = fuzzy_error.defuzzify_twist_stamped(node)?;
    publisher.publish(&msg)?;
    Ok(())
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
        let turn_limit = Z_LIMIT * (if self.left > self.right { 1.0 } else { -1.0 });
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
