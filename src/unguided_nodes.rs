use std::{cmp::max, f64::consts::PI, sync::Arc};

use arg_vals::{ArgDocs, ArgVals};

use particle_filter::{
    MapInput,
    angle::{Angle, Radians},
};
use r2r::{
    Node, Publisher,
    geometry_msgs::msg::TwistStamped,
    irobot_create_msgs::msg::{HazardDetectionVector, IrIntensityVector},
    nav_msgs::msg::Odometry,
    std_msgs::msg::String as Ros2String,
};
use rand::RngExt;
use ringbuffer::{AllocRingBuffer, RingBuffer};
use smol::lock::Mutex;

use crate::{
    PERIOD, bit_slam_nodes::{decode_ir, hazards_from, obstacle_topic_name, stop_topic_name}, node_struct::{NodeSpec, RunnableNode}, odom_topic_name, robot_name, util::{StampedString, pose_from_odometry, publish_map_input, publish_str, ros2_node_name, stamped_header, twist_stamped},
};

pub struct BumpTurnNode {
    docs: ArgDocs,
}

impl Default for BumpTurnNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "bump_turn_node",
                &vec![("--robot", "str", ""), ("--turn-distance", "f64", "0.5")],
            ),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
enum AvoidMode {
    #[default]
    Forward,
    Turn,
    Stop,
}

#[derive(Copy, Clone, Default, Debug)]
struct BumpTurnStatus {
    mode: AvoidMode,
    turn_remaining: Option<Radians>,
    last_angle: Option<Radians>,
}

impl BumpTurnStatus {
    fn bump_turn_move(
        &mut self,
        msg_angle: Radians,
        node: Arc<Mutex<Node>>,
        publisher: &Publisher<TwistStamped>,
    ) -> anyhow::Result<()> {
        if let Some((x, z)) = match self.mode {
            AvoidMode::Forward => Some((0.5, 0.0)),
            AvoidMode::Stop => Some((0.0, 0.0)),
            AvoidMode::Turn => self.turn(msg_angle),
        } {
            publisher.publish(&twist_stamped(node, x, z)?)?;
        }
        self.last_angle = Some(msg_angle);
        Ok(())
    }

    fn turn(&mut self, msg_angle: Radians) -> Option<(f64, f64)> {
        if let Some(last_angle) = self.last_angle {
            match self.turn_remaining.as_mut() {
                Some(turn_remaining_radians) => {
                    let last_diff = last_angle - msg_angle;
                    *turn_remaining_radians -= last_diff.abs();
                    self.turn_remaining = if f64::from(*turn_remaining_radians) < 0.0 {
                        None
                    } else {
                        Some(*turn_remaining_radians)
                    };
                    Some((0.0, 1.0))
                }
                None => {
                    self.mode = AvoidMode::Forward;
                    Some((0.5, 0.0))
                }
            }
        } else {
            None
        }
    }
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
        Ok(vec![
            obstacle_topic_name(robot),
            odom_topic_name(robot),
            stop_topic_name(robot),
        ])
    }

    fn spec(&self, args: &arg_vals::ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let avoid_distance: f64 = args.get_value("--turn-distance")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "bump_turn_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;

        let status = Arc::new(Mutex::new(BumpTurnStatus::default()));
        let motor_publisher = spec.publisher::<TwistStamped>(&pubs[0])?;

        let obstacle_status = status.clone();
        spec.subscribe(&subs[0], move |obstacle: Ros2String, _| {
            if let Ok(map_input) = obstacle.data.parse::<MapInput>() {
                let turn = match map_input {
                    MapInput::Pose(_) => false,
                    MapInput::Collision(_) => true,
                    MapInput::RangeObject(obstacle) => obstacle.distance() < avoid_distance,
                    MapInput::FreeSpace(_, _, _) => false,
                };

                if turn {
                    let mut obstacle_status = smol::block_on(obstacle_status.lock());
                    obstacle_status.mode = AvoidMode::Turn;
                    obstacle_status.turn_remaining = Some(Radians::new(PI * 1.0 / 8.0));
                }
            }
        })?;

        let odom_status = status.clone();
        spec.subscribe(&subs[1], move |odom: Odometry, node| {
            let pose = pose_from_odometry(&odom);
            if let Some(mut status) = odom_status.try_lock() {
                if let Err(e) = status.bump_turn_move(pose.theta, node, &motor_publisher) {
                    eprintln!("Error {e} from bump_turn_move()");
                }
            }
        })?;

        spec.subscribe(&subs[2], move |msg: Ros2String, _| {
            let mut status = smol::block_on(status.lock());
            match msg.data.as_str() {
                "stop" => status.mode = AvoidMode::Stop,
                "start" => status.mode = AvoidMode::Forward,
                _ => {
                    eprintln!("Unknown stop message: {}", msg.data);
                }
            }
        })?;
        Ok(spec)
    }
}

pub struct SimpleBumpIrNode {
    docs: ArgDocs,
}

impl Default for SimpleBumpIrNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "simple_bump_ir_node",
                &vec![
                    ("--robot", "str", ""),
                    ("--max-ir", "i16", "40"),
                    ("--min-turn-count", "usize", "5"),
                    ("--max-turn-count", "usize", "20"),
                ],
            ),
        }
    }
}

pub struct SimpleBumpIrStatus {
    min_timeout: usize,
    max_timeout: usize,
    timeout_counts_left: usize,
    max_ir: i16,
    inputs: Vec<MapInput>,
    running: bool,
    publisher: Publisher<Ros2String>,
}

impl SimpleBumpIrStatus {
    pub fn new(min_timeout: usize, max_timeout: usize, max_ir: i16, publisher: Publisher<Ros2String>) -> Self {
        Self {
            min_timeout,
            max_timeout,
            timeout_counts_left: 0,
            max_ir,
            inputs: vec![],
            running: true,
            publisher,
        }
    }

    pub fn handle_bump(&mut self, hazards: HazardDetectionVector, node: Arc<Mutex<Node>>) -> anyhow::Result<()> {
        for (_, bump) in hazards_from(&hazards) {
            self.avoid_obstacle();
            let map_input = bump.obstacle_at();
            publish_map_input(&map_input, &self.publisher, node.clone())?;
            self.inputs.push(map_input);
        }
        Ok(())
    }

    pub fn handle_irs(&mut self, irs: IrIntensityVector, node: Arc<Mutex<Node>>) -> anyhow::Result<()> {
        for ir in irs.readings.iter() {
            let reading = decode_ir(ir, self.max_ir)?;
            self.avoid_obstacle();
            let map_input = reading.reading_at();
            publish_map_input(&map_input, &self.publisher, node.clone())?;
            self.inputs.push(map_input);
        }
        Ok(())
    }

    pub fn handle_odom(&mut self, odom: &Odometry, node: Arc<Mutex<Node>>) -> anyhow::Result<()> {
        let pose = pose_from_odometry(&odom);
        let map_input = MapInput::Pose(pose);
        publish_map_input(&map_input, &self.publisher, node.clone())?;
        self.inputs.push(map_input);
        Ok(())
    }

    pub fn get_forward_left(&mut self) -> (f64, f64) {
        if !self.running {
            (0.0, 0.0)
        } else if self.timeout_counts_left > 0 {
            self.timeout_counts_left -= 1;
            (0.0, 0.5)
        } else {
            (0.5, 0.0)
        }
    }

    fn avoid_obstacle(&mut self) {
        let mut rnd = rand::rng();
        self.timeout_counts_left = rnd.random_range(self.min_timeout..=self.max_timeout);
    }
}

impl RunnableNode for SimpleBumpIrNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn publishing_topics(&self, args: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![format!("{robot}/cmd_vel_stamped"), obstacle_topic_name(robot)])
    }

    fn subscribing_topics(&self, args: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![
            format!("{robot}/hazard_detection"),
            format!("{robot}/ir_intensity"),
            odom_topic_name(robot),
            stop_topic_name(robot),
        ])
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "simple_bump_ir_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;

        let status = Arc::new(Mutex::new(SimpleBumpIrStatus::new(
            args.get_value("--min-turn-count")?,
            args.get_value("--max-turn-count")?,
            args.get_value("--max-ir")?,
            spec.publisher::<Ros2String>(&pubs[1])?,
        )));

        let hazard_status = status.clone();
        spec.subscribe(&subs[0], move |hazards: HazardDetectionVector, node| {
            let mut status = smol::block_on(hazard_status.lock());
            if let Err(e) = status.handle_bump(hazards, node.clone()) {
                eprintln!("Error {e} when handling bump");
            }
        })?;

        let ir_status = status.clone();
        spec.subscribe(&subs[1], move |irs: IrIntensityVector, node| {
            let mut status = smol::block_on(ir_status.lock());
            if let Err(e) = status.handle_irs(irs, node.clone()) {
                eprintln!("Error {e} when handling IR");
            }
        })?;

        let odom_status = status.clone();
        let motor_publisher = spec.publisher::<TwistStamped>(&pubs[0])?;
        spec.subscribe(&subs[2], move |odom: Odometry, node| {
            let mut status = smol::block_on(odom_status.lock());
            if let Err(e) = status.handle_odom(&odom, node.clone()) {
                eprintln!("Error {e} when handling odometry");
            }
            let (forward, leftward_angular) = status.get_forward_left();
            match twist_stamped(node, forward, leftward_angular) {
                Ok(twist) => {
                    if let Err(e) = motor_publisher.publish(&twist) {
                        eprintln!("Couldn't publish twist: {e}");
                    }
                }
                Err(err) => {
                    eprintln!("Couldn't build twist: {err}");
                }
            }
        })?;

        spec.subscribe(&subs[3], move |msg: Ros2String, _| {
            let mut status = smol::block_on(status.lock());
            match msg.data.as_str() {
                "stop" => status.running = false,
                "start" => status.running = true,
                _ => {
                    eprintln!("Unknown stop message: {}", msg.data);
                }
            }
        })?;

        Ok(spec)
    }
}

pub struct IrHazardDataNode {
    docs: ArgDocs,
}

impl Default for IrHazardDataNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "bump_turn_node",
                &vec![
                    ("--robot", "str", ""),
                    ("--history-window", "usize", "5"),
                    ("--starting-ir-max", "i16", "200"),
                    ("--ir-max-min", "i16", "10"),
                ],
            ),
        }
    }
}

struct IrHazardStatus {
    ir_max: i16,
    ir_max_min: i16,
    max_ir_history: AllocRingBuffer<i16>,
    mode: AvoidMode,
    pending_turn: f64,
    turn_coefficient: f64,
}

impl IrHazardStatus {
    fn new(args: &ArgVals) -> anyhow::Result<Self> {
        let ir_max_min = args.get_value("--ir-max-min")?;
        let ir_max = max(ir_max_min, args.get_value("--starting-ir-max")?);
        Ok(Self {
            ir_max,
            ir_max_min,
            max_ir_history: AllocRingBuffer::new(args.get_value("--history-window")?),
            mode: AvoidMode::Forward,
            pending_turn: 1.0,
            turn_coefficient: 1.0,
        })
    }

    fn handle_bump(&mut self, hazards: HazardDetectionVector) {
        for (frame_id, _) in hazards_from(&hazards) {
            if frame_id.starts_with("bump") {
                if let Some(min_max_ir) = self.max_ir_history.iter().min().copied() {
                    self.ir_max = max(self.ir_max_min, min_max_ir);
                    eprintln!("ir_max is now: {}", self.ir_max);
                }
            }
            self.mode = AvoidMode::Turn;
            self.turn_coefficient = self.pending_turn;
        }
    }

    fn handle_ir(
        &mut self,
        ir: IrIntensityVector,
        node: Arc<Mutex<Node>>,
        publisher: &Publisher<TwistStamped>,
    ) -> anyhow::Result<()> {
        let current_max = ir.readings.iter().map(|i| i.value).max().unwrap();
        self.pending_turn = turn_coefficient(&ir);
        self.mode = if current_max >= self.ir_max {
            if self.mode == AvoidMode::Forward {
                self.turn_coefficient = self.pending_turn;
            }
            AvoidMode::Turn
        } else {
            AvoidMode::Forward
        };
        let (x, z) = match self.mode {
            AvoidMode::Forward => (0.5, 0.0),
            AvoidMode::Turn => (0.0, self.turn_coefficient),
            AvoidMode::Stop => (0.0, 0.0),
        };
        self.max_ir_history.enqueue(current_max);
        publisher.publish(&twist_stamped(node, x, z)?)?;
        Ok(())
    }
}

impl RunnableNode for IrHazardDataNode {
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
        Ok(vec![
            format!("{robot}/ir_intensity"),
            format!("{robot}/hazard_detection"),
        ])
    }

    fn spec(&self, args: &arg_vals::ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let ir_status = Arc::new(Mutex::new(IrHazardStatus::new(args)?));
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "ir_hazard_data_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let ir_status_bump = ir_status.clone();
        spec.subscribe(&subs[1], move |hazards: HazardDetectionVector, _| {
            let mut ir_status_bump = smol::block_on(ir_status_bump.lock());
            ir_status_bump.handle_bump(hazards);
        })?;
        let publisher = spec.publisher::<TwistStamped>(&pubs[0])?;
        spec.subscribe(&subs[0], move |ir: IrIntensityVector, node| {
            if let Some(mut ir_status) = ir_status.try_lock() {
                if let Err(e) = ir_status.handle_ir(ir, node, &publisher) {
                    eprintln!("Error {e} when handling ir");
                }
            }
        })?;
        Ok(spec)
    }
}

fn turn_coefficient(ir: &IrIntensityVector) -> f64 {
    let halfway = ir.readings.len() / 2;
    let left_max = (0..halfway).map(|i| ir.readings[i].value).max().unwrap();
    let right_max = ((halfway + 1)..ir.readings.len())
        .map(|i| ir.readings[i].value)
        .max()
        .unwrap();
    if left_max > right_max { -1.0 } else { 1.0 }
}
