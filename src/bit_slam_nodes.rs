use crate::{
    PERIOD,
    node_struct::{NodeSpec, RunnableNode},
    odom_topic_name, robot_name,
    util::{
        StampedString, add_time_ns, particle2rosgrid, pose_from_odometry, pose2ros2pose,
        publish_str, ros2_node_name, ros2_topic_name, stamped_header, time_less_than,
        timestamped_filename,
    },
};
use arg_vals::{ArgDocs, ArgVals};
use particle_filter::{
    MapInput, Particle, ParticleFilter, ParticleFilterSettings, ParticleType,
    irobot_create3::{Bump, IrHeading, IrReading},
    path_plan::PathsBackTo,
    point::FloatPoint,
};
use r2r::{
    Node, Publisher,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Point as Ros2Point, PoseStamped},
    irobot_create_msgs::msg::{HazardDetectionVector, IrIntensity, IrIntensityVector},
    nav_msgs::msg::{OccupancyGrid, Odometry},
    sensor_msgs::msg::LaserScan,
    std_msgs::msg::{Header, String as Ros2String},
};
use smol::lock::Mutex;
use std::{collections::VecDeque, sync::Arc};

pub struct BumpObstacleNode {
    docs: ArgDocs,
}

impl Default for BumpObstacleNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new("bump_obstacle_node", &vec![("--robot", "str", "")]),
        }
    }
}

impl RunnableNode for BumpObstacleNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "obstacle_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let publisher = spec.publisher::<Ros2String>(&pubs[0])?;
        spec.subscribe(&subs[0], move |hazards: HazardDetectionVector, _| {
            for (frame_id, bump) in hazards_from(&hazards) {
                let str = format!("{}", bump.obstacle_at());
                let stamped_str = StampedString {
                    header: hazards.header.clone(),
                    data: str,
                };
                if let Err(e) = serde_json::to_string(&stamped_str)
                    .map_err(anyhow::Error::from)
                    .and_then(|json| publish_str(&publisher, json))
                {
                    eprintln!("Error {e} when trying to publish hazard {frame_id}.");
                }
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![obstacle_topic_name(robot_name!(args))])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![format!("{robot}/hazard_detection")])
    }
}

pub struct BumpIrObstacleNode {
    docs: ArgDocs,
}

impl Default for BumpIrObstacleNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "bump_obstacle_node",
                &vec![("--robot", "str", ""), ("--min-obstacle-ir", "i16", "40")],
            ),
        }
    }
}

impl RunnableNode for BumpIrObstacleNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let min_ir_obstacle_present = args.get_value::<i16>("--min-obstacle-ir")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "obstacle_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let pending_hazards = Arc::new(Mutex::new(VecDeque::new()));
        let ir_hazard_check = pending_hazards.clone();
        spec.subscribe(&subs[0], move |hazards: HazardDetectionVector, _| {
            for (_, bump) in hazards_from(&hazards) {
                let mut pending_hazards = smol::block_on(pending_hazards.lock());
                pending_hazards.push_back((hazards.header.clone(), bump));
                eprintln!("Caught bump: {bump:?}; {} pending", pending_hazards.len());
            }
        })?;
        let publisher = spec.publisher::<Ros2String>(&pubs[0])?;
        spec.subscribe(&subs[1], move |ir: IrIntensityVector, _| {
            if let Some(mut pending_hazards) = ir_hazard_check.try_lock() {
                Self::handle_ir_intensity(
                    &mut pending_hazards,
                    ir,
                    &publisher,
                    min_ir_obstacle_present,
                );
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![obstacle_topic_name(robot_name!(args))])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![
            format!("{robot}/hazard_detection"),
            format!("{robot}/ir_intensity"),
        ])
    }
}

impl BumpIrObstacleNode {
    fn handle_ir_intensity(
        pending_hazards: &mut VecDeque<(Header, Bump)>,
        ir: IrIntensityVector,
        publisher: &Publisher<Ros2String>,
        min_ir_obstacle_present: i16,
    ) {
        while let Some((bump_header, bump)) = pending_hazards.pop_front() {
            eprintln!("Publishing {bump:?}; {} pending", pending_hazards.len());
            let str = format!("{}", bump.obstacle_at());
            let stamped_str = StampedString {
                header: bump_header,
                data: str,
            };
            if let Err(e) = serde_json::to_string(&stamped_str)
                .map_err(anyhow::Error::from)
                .and_then(|json| publish_str(&publisher, json))
            {
                eprintln!("Error {e} when trying to publish {}", bump.obstacle_at());
            }
        }
        Self::process_ir_readings(&ir, min_ir_obstacle_present, &publisher);
    }

    fn process_ir_readings(
        ir: &IrIntensityVector,
        min_ir_obstacle_present: i16,
        publisher: &Publisher<Ros2String>,
    ) {
        for sensor in ir.readings.iter() {
            match decode_ir(sensor, min_ir_obstacle_present) {
                Ok(ir_reading) => {
                    let str = format!("{}", ir_reading.reading_at());
                    let stamped_str = StampedString {
                        header: ir.header.clone(),
                        data: str,
                    };
                    if let Err(e) = serde_json::to_string(&stamped_str)
                        .map_err(anyhow::Error::from)
                        .and_then(|json| publish_str(&publisher, json))
                    {
                        eprintln!("Error {e} when trying to publish IR reading.");
                    }
                }
                Err(e) => {
                    eprintln!(
                        "Error {e} when trying to parse IR header {}",
                        sensor.header.frame_id
                    );
                }
            }
        }
    }
}

pub fn hazards_from(hazards: &HazardDetectionVector) -> impl Iterator<Item = (String, Bump)> {
    hazards.detections.iter().filter_map(|h| {
        h.header
            .frame_id
            .parse::<Bump>()
            .ok()
            .map(|b| (h.header.frame_id.clone(), b))
    })
}

pub fn decode_ir(ir: &IrIntensity, min_obstacle_ir_present: i16) -> anyhow::Result<IrReading> {
    ir.header
        .frame_id
        .parse::<IrHeading>()
        .map(|heading| IrReading::new(ir.value, heading, min_obstacle_ir_present))
}

pub struct ScanObstacleNode {
    docs: ArgDocs,
}

impl Default for ScanObstacleNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new("scan_obstacle_node", &vec![("--robot", "str", "")]),
        }
    }
}

impl RunnableNode for ScanObstacleNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "obstacle_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let publisher = spec.publisher::<Ros2String>(&pubs[0])?;
        let obstacle_threshold = 12.0;
        spec.subscribe(&subs[0], move |scan: LaserScan, _| {
            for (i, &range) in scan.ranges.iter().enumerate() {
                if range > scan.range_min {
                    if let Err(e) = publish_scan_obstacle_location(
                        &publisher,
                        &scan,
                        i,
                        range,
                        obstacle_threshold,
                    ) {
                        eprintln!("Error {e} when trying to publish scan obstacle");
                    }
                }
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![obstacle_topic_name(robot_name!(args))])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![format!("{robot}/scan")])
    }
}

pub const SCAN_DISTANCE_NOISE: f64 = 0.05;
pub const SCAN_HEADING_NOISE: f64 = 0.005;

fn publish_scan_obstacle_location(
    publisher: &Publisher<Ros2String>,
    scan: &LaserScan,
    scan_index: usize,
    range: f32,
    obstacle_threshold: f32,
) -> anyhow::Result<()> {
    let tag = if range < obstacle_threshold {
        "object"
    } else {
        "freespace"
    };
    let heading = scan.angle_min + (scan_index as f32 * scan.angle_increment);
    let distance = if range < obstacle_threshold {
        range
    } else {
        obstacle_threshold
    };
    let str = format!("({tag},{distance},{heading},{SCAN_DISTANCE_NOISE},{SCAN_HEADING_NOISE})");
    let stamped_str = StampedString {
        header: scan.header.clone(),
        data: str,
    };
    publish_str(publisher, serde_json::to_string(&stamped_str)?)
}

pub struct BitSlamNode {
    docs: ArgDocs,
}

impl Default for BitSlamNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "bit_slam_node",
                &vec![
                    ("--robot", "str", ""),
                    ("--num-particles", "usize", "1000"),
                    ("--meters-per-cell", "f64", "0.1"),
                    ("--save-map", "bool", "true"),
                    ("--weight-strategy", "WeightStrategy", "MinPose"),
                    (
                        "--selection-strategy",
                        "SelectionStrategy",
                        "RankProportion",
                    ),
                    ("--range-sensor-interval-ms", "Option<u32>", "None"),
                ],
            ),
        }
    }
}

impl RunnableNode for BitSlamNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let setup = BitSlamSetup::new(args)?;
        let mut spec = NodeSpec::new(&setup.node_name, setup.period)?;
        let particle_filter = setup.create_particle_filter();
        let particle_publisher = spec.publisher::<Ros2String>(&setup.particle_topic)?;
        let occupancy_grid_publisher =
            spec.publisher::<OccupancyGrid>(&setup.occupancy_grid_topic)?;
        let pose_publisher = spec.publisher::<PoseStamped>(&setup.pose_topic)?;
        let status_publisher = spec.publisher::<Ros2String>(&setup.status_topic)?;
        let particle_data = Arc::new(Mutex::new(ParticleData {
            particle_filter,
            last_sensor_time: None,
            range_reading_interval_ns: args
                .get_optional_value::<u32>("--range-sensor-interval-ms")?
                .map(|ms| {
                    assert!(ms < 4_000);
                    ms * 1_000_000
                }),
            next_range_reading: None,
            particle_publisher,
            occupancy_grid_publisher,
            pose_publisher,
            status_publisher,
            map_saved: false,
        }));
        setup.subscribe_obstacle(&mut spec, particle_data.clone())?;
        setup.subscribe_odometry(&mut spec, particle_data.clone())?;
        setup.subscribe_save(&mut spec, particle_data)?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![
            particle_topic_name(robot),
            occupancy_grid_topic_name(robot),
            pose_topic_name(robot),
            status_topic_name(robot),
        ])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![
            obstacle_topic_name(robot),
            odom_topic_name(robot),
            save_topic_name(robot),
        ])
    }
}

struct ParticleData {
    particle_filter: ParticleFilter,
    last_sensor_time: Option<Time>,
    range_reading_interval_ns: Option<u32>,
    next_range_reading: Option<Time>,
    particle_publisher: Publisher<Ros2String>,
    occupancy_grid_publisher: Publisher<OccupancyGrid>,
    pose_publisher: Publisher<PoseStamped>,
    status_publisher: Publisher<Ros2String>,
    map_saved: bool,
}

impl ParticleData {
    fn save(&self) -> anyhow::Result<()> {
        let json = serde_json::to_string(&self.particle_filter)?;
        let output_filename = timestamped_filename("bit_slam");
        std::fs::write(output_filename, json)?;
        Ok(())
    }

    fn is_timely(&mut self, time: &Time) -> bool {
        let mut prev = Some(time.clone());
        std::mem::swap(&mut prev, &mut self.last_sensor_time);
        prev.map_or(true, |prev| time_less_than(&prev, time))
    }

    fn usable_range_reading(&mut self, time: &Time) -> bool {
        match self.range_reading_interval_ns {
            None => true,
            Some(range_reading_interval_ns) => {
                let mut usable_reading = true;
                if let Some(next_range_reading) = self.next_range_reading.clone() {
                    if time_less_than(time, &next_range_reading) {
                        usable_reading = false;
                    }
                }
                if usable_reading {
                    self.next_range_reading = Some(add_time_ns(time, range_reading_interval_ns));
                }
                usable_reading
            }
        }
    }
}

struct BitSlamSetup {
    node_name: String,
    occupancy_grid_topic: String,
    particle_topic: String,
    obstacle_topic: String,
    status_topic: String,
    odom_topic: String,
    pose_topic: String,
    save_topic: String,
    settings: ParticleFilterSettings,
    period: u64,
}

impl BitSlamSetup {
    fn new(args: &ArgVals) -> anyhow::Result<Self> {
        let robot = format!("{}", args.get_str_value("--robot")?);
        let mut settings = ParticleFilterSettings::default();
        settings.num_particles = args.get_value("--num-particles")?;
        settings.square_size_m = args.get_value("--meters-per-cell")?;
        settings.save_inputs = args.get_value("--save-map")?;
        settings.selection_strategy = args.get_value("--selection-strategy")?;
        settings.weight_strategy = args.get_value("--weight-strategy")?;
        let node_name = ros2_node_name(&robot, "bitslam_node");
        Ok(Self {
            node_name,
            occupancy_grid_topic: occupancy_grid_topic_name(&robot),
            particle_topic: particle_topic_name(&robot),
            obstacle_topic: obstacle_topic_name(&robot),
            status_topic: status_topic_name(&robot),
            odom_topic: odom_topic_name(&robot),
            pose_topic: pose_topic_name(&robot),
            save_topic: save_topic_name(&robot),
            settings,
            period: args.get_value("-spin_time").unwrap_or(PERIOD),
        })
    }

    fn create_particle_filter(&self) -> ParticleFilter {
        ParticleFilter::new(self.settings.clone())
    }

    fn subscribe_obstacle(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.obstacle_topic, move |raw_msg: Ros2String, node| {
            let mut particle_data = smol::block_on(particle_data.lock());
            if let Ok(obst) = serde_json::from_str::<StampedString>(&raw_msg.data) {
                let sim_time = &obst.header.stamp;
                if particle_data.is_timely(sim_time) {
                    let ros2_string_wrapper = Ros2String {
                        data: obst.data.clone(),
                    };
                    if let Err(e) = publish_particle_obstacle(
                        node,
                        &ros2_string_wrapper,
                        &mut particle_data,
                        sim_time,
                    ) {
                        eprintln!("Error {e} when updating particle filter");
                    }
                }
            } else {
                eprintln!("Error: Received corrupted or invalid JSON on obstacle topic!");
            }
        })?;
        Ok(())
    }

    fn subscribe_odometry(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.odom_topic, move |odom: Odometry, node| {
            let mut particle_data = smol::block_on(particle_data.lock());
            let sim_time = &odom.header.stamp;
            if particle_data.is_timely(sim_time) {
                if let Err(e) = publish_particle_odom(node, &odom, &mut particle_data, sim_time) {
                    eprintln!("Error {e} when updating particle filter with {odom:?}");
                }
            }
        })
    }

    fn subscribe_save(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.save_topic, move |msg: Ros2String, _| {
            let mut particle_data = smol::block_on(particle_data.lock());
            eprintln!("Received save message: \"{}\"", msg.data);
            let parts = msg.data.split_whitespace().collect::<Vec<_>>();
            if parts[0] == "at" {
                match parts[1].parse::<FloatPoint>() {
                    Ok(point) => {
                        particle_data.particle_filter.set_actual_ending_point(point);
                        eprintln!("distance from start: {point}");
                    }
                    Err(e) => {
                        eprintln!("Error {e} when parsing ground truth offset.")
                    }
                }
            }
            if let Err(e) = particle_data.save() {
                eprintln!("Error {e} when saving particle filter.");
            }
        })
    }
}

fn publish_particle_obstacle(
    node: Arc<Mutex<Node>>,
    obst: &Ros2String,
    particle_data: &mut ParticleData,
    sim_time: &r2r::builtin_interfaces::msg::Time,
) -> anyhow::Result<()> {
    let map_input = obst.data.parse::<MapInput>()?;
    if !map_input.is_range_obstacle() || particle_data.usable_range_reading(sim_time) {
        particle_data.particle_filter.iterate(map_input);
        publish_particle(node, particle_data, sim_time)?;
    } else {
        eprintln!(
            "Ignoring {sim_time:?} range object; awaiting {:?}",
            particle_data.next_range_reading
        );
    }
    Ok(())
}

fn publish_particle_odom(
    node: Arc<Mutex<Node>>,
    odom: &Odometry,
    particle_data: &mut ParticleData,
    sim_time: &r2r::builtin_interfaces::msg::Time,
) -> anyhow::Result<()> {
    let pose = pose_from_odometry(&odom);
    particle_data.particle_filter.iterate(MapInput::Pose(pose));
    publish_particle(node, particle_data, sim_time)?;
    Ok(())
}

fn publish_particle(
    node: Arc<Mutex<Node>>,
    particle_data: &mut ParticleData,
    sim_time: &r2r::builtin_interfaces::msg::Time,
) -> anyhow::Result<()> {
    let particle = particle_data.particle_filter.representative_particle();
    if !particle_data.map_saved && PathsBackTo::done(&particle.particle) {
        particle_data.save()?;
        particle_data.map_saved = true;
    }

    publish_str(
        &particle_data.particle_publisher,
        serde_json::to_string(&particle.particle)?,
    )?;

    let grid = particle2rosgrid(node.clone(), &particle.particle)?;
    particle_data.occupancy_grid_publisher.publish(&grid)?;

    let mut pose_stamped = PoseStamped {
        header: stamped_header(node)?,
        pose: pose2ros2pose(&particle.particle.estimated_pose()),
    };
    pose_stamped.header.stamp = sim_time.clone();
    //eprintln!("estimated: {:?} raw: {:?}", particle.particle.estimated_pose(), particle.particle.estimate().last_raw_pose());
    particle_data.pose_publisher.publish(&pose_stamped)?;

    let msg = (if particle.particle_type == ParticleType::Failure {
        "Failed"
    } else if particle_data.map_saved {
        "Finished"
    } else {
        "Mapping"
    })
    .to_string();
    publish_str(&particle_data.status_publisher, msg)
}

pub struct BitSlamExplorerNode {
    docs: ArgDocs,
}

impl BitSlamExplorerNode {
    pub fn goal_publish_topic(&self, args: &ArgVals) -> anyhow::Result<String> {
        Ok(ros2_topic_name(robot_name!(args), "bitslam_explorer_goal"))
    }

    pub fn stop_publish_topic(&self, args: &ArgVals) -> anyhow::Result<String> {
        Ok(stop_topic_name(robot_name!(args)))
    }
}

impl Default for BitSlamExplorerNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new("bit_slam_explorer_node", &vec![("--robot", "str", "")]),
        }
    }
}

impl RunnableNode for BitSlamExplorerNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "explorer_node"), PERIOD)?;
        let pubs = self.publishing_topics(args)?;
        let subs = self.subscribing_topics(args)?;
        let point_publisher = spec.publisher::<Ros2Point>(&pubs[0])?;
        let stop_publisher = spec.publisher::<Ros2String>(&pubs[1])?;
        spec.subscribe(
            &subs[0],
            move |particle_str: Ros2String, _| match serde_json::from_str::<Particle>(
                &particle_str.data,
            ) {
                Ok(particle) => {
                    publish_goal_from_particle(&particle, &point_publisher, &stop_publisher)
                }
                Err(e) => {
                    eprintln!("Error {e} when deserializing particle");
                }
            },
        )?;
        spec.subscribe(&subs[1], move |obstacle: Ros2String, _| {
            if let Ok(map_input) = obstacle.data.parse::<MapInput>() {
                if let MapInput::Collision(_) = map_input {
                    // Maybe do something here to go the other way.
                }
            }
        })?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![
            self.goal_publish_topic(args)?,
            self.stop_publish_topic(args)?,
        ])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![particle_topic_name(robot), obstacle_topic_name(robot)])
    }
}

fn publish_goal_from_particle(
    particle: &Particle,
    point_publisher: &Publisher<Ros2Point>,
    stop_publisher: &Publisher<Ros2String>,
) {
    if let Some(target) = particle.map().exploration_target(particle.estimated_pose()) {
        let meters = particle.map().to_meters(target);
        let target = particle.estimate().convert_to_raw_space(&meters);
        publish_target(target, point_publisher);
    } else {
        if particle.map().is_consistent() {
            eprintln!("There is not a path but map is consistent");
        } else {
            eprintln!("The map is inconsistent");
        }
        if let Err(e) = publish_str(stop_publisher, "stop".to_string()) {
            eprintln!("Error {e} when trying to publish stop message");
        }
    }
}

fn publish_target(target: FloatPoint, point_publisher: &Publisher<Ros2Point>) {
    let msg = Ros2Point {
        x: target[0],
        y: target[1],
        z: 0.0,
    };
    if let Err(e) = point_publisher.publish(&msg) {
        eprintln!("Error {e} when trying to publish {msg:?}");
    }
}

pub fn obstacle_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_obstacles")
}

pub fn particle_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_maps")
}

pub fn occupancy_grid_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_occupancy_grid")
}

pub fn pose_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_pose")
}

pub fn status_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_status")
}

pub fn save_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_save")
}

pub fn stop_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_explorer_stop")
}
