use crate::{
    PERIOD,
    node_struct::{NodeSpec, RunnableNode},
    odom_topic_name, robot_name,
    util::{particle2rosgrid, pose_from_odometry, ros2_node_name, ros2_topic_name, timestamped_filename},
};
use arg_vals::{ArgDocs, ArgVals};
use particle_filter::{
    MapInput, Particle, ParticleFilter, ParticleFilterSettings, irobot_create3::Bump,
    path_plan::{PathsBackTo, necessary_turns_from}, point::FloatPoint,
};
use r2r::{
    Node, Publisher,
    geometry_msgs::msg::Point as Ros2Point,
    irobot_create_msgs::msg::HazardDetectionVector,
    nav_msgs::msg::{OccupancyGrid, Odometry},
    sensor_msgs::msg::LaserScan,
    std_msgs::msg::String as Ros2String,
};
use smol::lock::Mutex;
use std::sync::Arc;

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
                if let Err(e) = publish_obstacle_location(&publisher, &bump) {
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

pub fn hazards_from(hazards: &HazardDetectionVector) -> impl Iterator<Item = (String, Bump)> {
    hazards.detections.iter().filter_map(|h| {
        h.header
            .frame_id
            .parse::<Bump>()
            .ok()
            .map(|b| (h.header.frame_id.clone(), b))
    })
}

fn publish_obstacle_location(publisher: &Publisher<Ros2String>, bump: &Bump) -> anyhow::Result<()> {
    let obstacle = bump.obstacle_at();
    let msg = Ros2String {
        data: format!("{obstacle}"),
    };
    publisher.publish(&msg)?;
    Ok(())
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
        let obstacle_threshold = 1.0;
        spec.subscribe(&subs[0], move |scan: LaserScan, _| {
            for (i, &range) in scan.ranges.iter().enumerate() {
                if range > scan.range_min {
                    let tag = if range < obstacle_threshold {
                        "object"
                    } else {
                        "freespace"
                    };
                    let heading = scan.angle_min + (i as f32 * scan.angle_increment);
                    let dist = if range < obstacle_threshold {
                        range
                    } else {
                        obstacle_threshold
                    };
                    if let Err(e) = publish_scan_obstacle_location(&publisher, tag, dist, heading) {
                        eprintln!("Error {e} when trying to publish scan obstacle");
                    }
                    break;
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
    tag: &str,
    distance: f32,
    heading: f32,
) -> anyhow::Result<()> {
    let msg = Ros2String {
        data: format!("({tag},{distance},{heading},{SCAN_DISTANCE_NOISE},{SCAN_HEADING_NOISE})"),
    };
    publisher.publish(&msg)?;
    Ok(())
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
        let status_publisher = spec.publisher::<Ros2String>(&setup.status_topic)?;
        let particle_data = Arc::new(Mutex::new(ParticleData {
            particle_filter,
            particle_publisher,
            occupancy_grid_publisher,
            status_publisher,
            distance_from_start: None,
            map_saved: false,
        }));
        setup.subscribe_obstacle(&mut spec, particle_data.clone())?;
        setup.subscribe_odometry(&mut spec, particle_data.clone())?;
        setup.subscribe_save(&mut spec, particle_data)?;
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![
            particle_topic_name(robot_name!(args)),
            occupancy_grid_topic_name(robot_name!(args)),
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
    particle_publisher: Publisher<Ros2String>,
    occupancy_grid_publisher: Publisher<OccupancyGrid>,
    status_publisher: Publisher<Ros2String>,
    distance_from_start: Option<FloatPoint>,
    map_saved: bool,
}

impl ParticleData {
    fn save(&self) -> anyhow::Result<()> {
        let json = serde_json::to_string(&self.particle_filter)?;
        let output_filename = timestamped_filename("bit_slam");
        std::fs::write(output_filename, json)?;
        Ok(())
    }
}

struct BitSlamSetup {
    node_name: String,
    occupancy_grid_topic: String,
    particle_topic: String,
    obstacle_topic: String,
    status_topic: String,
    odom_topic: String,
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
        let node_name = ros2_node_name(&robot, "bitslam_node");
        Ok(Self {
            node_name,
            occupancy_grid_topic: occupancy_grid_topic_name(&robot),
            particle_topic: particle_topic_name(&robot),
            obstacle_topic: obstacle_topic_name(&robot),
            status_topic: status_topic_name(&robot),
            odom_topic: odom_topic_name(&robot),
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
        spec.subscribe(&self.obstacle_topic, move |obst: Ros2String, node| {
            let mut particle_data = smol::block_on(particle_data.lock());
            if let Err(e) = publish_particle_obstacle(node, &obst, &mut particle_data) {
                eprintln!("Error {e} when updating particle filter with {}", obst.data);
            }
        })
    }

    fn subscribe_odometry(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.odom_topic, move |odom: Odometry, node| {
            let mut particle_data = smol::block_on(particle_data.lock());
            if let Err(e) = publish_particle_odom(node, &odom, &mut particle_data) {
                eprintln!("Error {e} when updating particle filter with {odom:?}");
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
                        particle_data.distance_from_start = Some(point);
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
) -> anyhow::Result<()> {
    let map_input = obst.data.parse::<MapInput>()?;
    particle_data.particle_filter.iterate(map_input);
    publish_particle(node, particle_data)?;
    Ok(())
}

fn publish_particle_odom(
    node: Arc<Mutex<Node>>,
    odom: &Odometry,
    particle_data: &mut ParticleData,
) -> anyhow::Result<()> {
    let pose = pose_from_odometry(&odom);
    particle_data.particle_filter.iterate(MapInput::Pose(pose));
    publish_particle(node, particle_data)?;
    Ok(())
}

fn publish_particle(node: Arc<Mutex<Node>>, particle_data: &mut ParticleData) -> anyhow::Result<()> {
    let failure = particle_data.particle_filter.example_failure();
    let particle = match failure.as_ref() {
        None => particle_data.particle_filter.particles().next().unwrap(),
        Some(failure) => failure,
    };
    if !particle_data.map_saved && PathsBackTo::done(&particle) {
        particle_data.save()?;
        particle_data.map_saved = true;
    }
    let data = serde_json::to_string(particle)?;
    let msg = Ros2String { data };
    particle_data.particle_publisher.publish(&msg)?;

    let grid = particle2rosgrid(node, particle)?;
    particle_data.occupancy_grid_publisher.publish(&grid)?;

    let msg = Ros2String {
        data: (if failure.is_some() {
            "Failed"
        } else if particle_data.map_saved {
            "Finished"
        } else {
            "Mapping"
        })
        .to_string(),
    };
    particle_data.status_publisher.publish(&msg)?;
    Ok(())
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
        let point_publisher = spec.publisher::<Ros2Point>(&pubs[0])?;
        let stop_publisher = spec.publisher::<Ros2String>(&pubs[1])?;
        spec.subscribe(
            &particle_topic_name(robot),
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
        Ok(spec)
    }

    fn publishing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![
            self.goal_publish_topic(args)?,
            self.stop_publish_topic(args)?,
        ])
    }

    fn subscribing_topics(&self, args: &ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![particle_topic_name(robot_name!(args))])
    }
}

fn publish_goal_from_particle(
    particle: &Particle,
    point_publisher: &Publisher<Ros2Point>,
    stop_publisher: &Publisher<Ros2String>,
) {
    let paths = PathsBackTo::any(particle.map(), particle.estimated_pose());
    let path = paths.shortest_path().map(|p| necessary_turns_from(p.iter().copied(), particle.map()));
    if let Some(path) = path {
        if let Some(next_step) = path.get(1).copied() {
            let meters = particle.map().to_meters(next_step);
            let target = particle.estimate().convert_to_raw_space(&meters);
            publish_target(target, point_publisher);
        } else {
            if let Some(last_raw_pose) = particle.estimate().last_raw_pose() {
                let target = last_raw_pose + (1.0, last_raw_pose.theta);
                publish_target(target, point_publisher);
            } else {
                eprintln!("Just starting - no last raw pose");
            }            
        }
    } else {
        eprintln!("There is not a path");
        let data = "stop".to_string();
        if let Err(e) = stop_publisher.publish(&Ros2String { data }) {
            eprintln!("Error {e} when trying to publish stop message");
        }
    }
}

fn publish_target(target: FloatPoint, point_publisher: &Publisher<Ros2Point>) {
    eprintln!("Publishing target {target}");
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

pub fn status_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_status")
}

pub fn save_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_save")
}

pub fn stop_topic_name(robot: &str) -> String {
    ros2_topic_name(robot, "bitslam_explorer_stop")
}