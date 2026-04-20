use std::{f64::consts::PI, sync::Arc};

use arg_vals::ArgDocs;

use crossbeam::atomic::AtomicCell;
use particle_filter::angle::Radians;
use r2r::{
    Node, Publisher,
    geometry_msgs::msg::TwistStamped,
    nav_msgs::msg::Odometry,
    std_msgs::msg::String as Ros2String,
};

use crate::{PERIOD, bit_slam_nodes::obstacle_topic_name, node_struct::{NodeSpec, RunnableNode}, odom_topic_name, robot_name, util::{pose_from_odometry, twist_stamped}};

pub struct BumpTurnNode {
    docs: ArgDocs,
}

impl Default for BumpTurnNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new("bump_turn_node", &vec![("--robot", "str", "")]),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum BumpTurnMode {
    Forward,
    Turn,
}

impl RunnableNode for BumpTurnNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn publishing_topics(&self, args: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![format!("{robot}/cmd_vel_stamped")])
    }

    fn subscribing_topics(&self, args: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![obstacle_topic_name(robot), odom_topic_name(robot)])
    }

    fn spec(&self, args: &arg_vals::ArgVals) -> anyhow::Result<crate::node_struct::NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(format!("{robot}_bump_turn_node").as_str(), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let mode = Arc::new(AtomicCell::new(BumpTurnMode::Forward));
        let turn_remaining = Arc::new(AtomicCell::new(None));
        let last_angle = Arc::new(AtomicCell::new(None));
        let publisher = spec.publisher::<TwistStamped>(&pubs[0])?;

        let ob_mode = mode.clone();
        let tr = turn_remaining.clone();
        spec.subscribe(&subs[0], move |_obstacle: Ros2String, _| {
            ob_mode.store(BumpTurnMode::Turn);
            tr.store(Some(Radians::new(PI * 1.0 / 8.0)));
        })?;

        let odom_mode = mode.clone();
        let turn_remaining = turn_remaining.clone();
        let last_angle = last_angle.clone();
        spec.subscribe(&subs[1], move |odom: Odometry, node| {
            let pose = pose_from_odometry(&odom);
            eprintln!("At {pose}; last angle was {:?}", last_angle.load());
            if let Some(last_angle) = last_angle.load() {
                if let Err(e) = bump_turn_move(pose.theta, last_angle, turn_remaining.clone(), node, odom_mode.clone(), &publisher) {
                    eprintln!("Error {e} from bump_turn_move()");
                }
            }
            last_angle.store(Some(pose.theta))
        })?;
        Ok(spec)
    }
}

fn bump_turn_move(msg_angle: Radians, last_angle: Radians, turn_remaining: Arc<AtomicCell<Option<Radians>>>, node: &Node, odom_mode: Arc<AtomicCell<BumpTurnMode>>, publisher: &Publisher<TwistStamped>) -> anyhow::Result<()> {
    let (x, z) = match odom_mode.load() {
        BumpTurnMode::Forward => (0.5, 0.0),
        BumpTurnMode::Turn => match turn_remaining.load() {
            Some(mut turn_remaining_radians) => {
                let last_diff = last_angle - msg_angle;
                turn_remaining_radians -= last_diff.abs();
                if f64::from(turn_remaining_radians) < 0.0 {
                    turn_remaining.store(None);
                } else {
                    turn_remaining.store(Some(turn_remaining_radians));
                }
                (0.0, 1.0)
            }
            None => {
                odom_mode.store(BumpTurnMode::Forward);
                (0.5, 0.0)
            }
        }
    };
    publisher.publish(&twist_stamped(node, x, z)?)?;
    eprintln!("published x: {x} z: {z}");
    Ok(())
}
