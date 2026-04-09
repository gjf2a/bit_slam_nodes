use crate::{
    PERIOD,
    node_struct::{NodeSpec, RunnableNode},
    odom_topic_name, robot_name,
    util::{parse_obstacle_distance_heading, particle2rosgrid, pose_from_odometry},
};
use arg_vals::{ArgDocs, ArgVals};
use chrono::Local;
use particle_filter::{
    MapInput, Particle, ParticleFilter, ParticleFilterSettings, irobot_create3::Bump,
    path_plan::PathsBackTo,
};
use r2r::{
    Node, Publisher,
    geometry_msgs::msg::Point as Ros2Point,
    irobot_create_msgs::msg::HazardDetectionVector,
    nav_msgs::msg::{OccupancyGrid, Odometry},
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

    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()> {
        self.docs.set_default(param, param_default)
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(format!("{robot}_obstacle_node").as_str(), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        let pubs = self.publishing_topics(args)?;
        let publisher = spec.publisher::<Ros2String>(&pubs[0])?;
        spec.subscribe(&subs[0], move |hazards: HazardDetectionVector, _| {
            for detection in hazards.detections {
                if let Err(e) = publish_obstacle_location(&publisher, &detection.header.frame_id) {
                    eprintln!(
                        "Error {e} when trying to publish hazard {}",
                        detection.header.frame_id
                    );
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

fn publish_obstacle_location(
    publisher: &Publisher<Ros2String>,
    frame_id: &str,
) -> anyhow::Result<()> {
    if let Ok(bump) = frame_id.parse::<Bump>() {
        let (distance, heading) = bump.obstacle_at();
        let heading: f64 = heading.into();
        let msg = Ros2String {
            data: format!("({distance},{heading})"),
        };
        publisher.publish(&msg)?;
    }
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

    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()> {
        self.docs.set_default(param, param_default)
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let setup = BitSlamSetup::new(args)?;
        let mut spec = NodeSpec::new(&setup.node_name, setup.period)?;
        let particle_filter = setup.create_particle_filter();
        let particle_publisher = spec.publisher::<Ros2String>(&setup.particle_topic)?;
        let occupancy_grid_publisher =
            spec.publisher::<OccupancyGrid>(&setup.occupancy_grid_topic)?;
        let particle_data = Arc::new(Mutex::new(ParticleData {
            particle_filter,
            particle_publisher,
            occupancy_grid_publisher,
            map_saved: false,
        }));
        setup.subscribe_obstacle(&mut spec, particle_data.clone())?;
        setup.subscribe_odometry(&mut spec, particle_data.clone())?;
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
        Ok(vec![obstacle_topic_name(robot), odom_topic_name(robot)])
    }
}

struct ParticleData {
    particle_filter: ParticleFilter,
    particle_publisher: Publisher<Ros2String>,
    occupancy_grid_publisher: Publisher<OccupancyGrid>,
    map_saved: bool,
}

struct BitSlamSetup {
    node_name: String,
    occupancy_grid_topic: String,
    particle_topic: String,
    obstacle_topic: String,
    odom_topic: String,
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
        Ok(Self {
            node_name: format!("{robot}_bitslam_node"),
            occupancy_grid_topic: occupancy_grid_topic_name(&robot),
            particle_topic: particle_topic_name(&robot),
            obstacle_topic: obstacle_topic_name(&robot),
            odom_topic: odom_topic_name(&robot),
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
            if let Some(mut particle_data) = particle_data.try_lock() {
                eprintln!("Inside odometry critical section");
                if let Err(e) = publish_particle_odom(node, &odom, &mut particle_data) {
                    eprintln!("Error {e} when updating particle filter with {odom:?}");
                }
            } else {
                eprintln!("Skipped odometry critical section");
            }
        })
    }
}

fn publish_particle_obstacle(
    node: &Node,
    obst: &Ros2String,
    particle_data: &mut ParticleData,
) -> anyhow::Result<()> {
    let (distance, heading) = parse_obstacle_distance_heading(&obst.data)?;
    particle_data
        .particle_filter
        .iterate(MapInput::Obstacle(distance, heading));
    publish_particle(node, particle_data)?;
    Ok(())
}

fn publish_particle_odom(
    node: &Node,
    odom: &Odometry,
    particle_data: &mut ParticleData,
) -> anyhow::Result<()> {
    let pose = pose_from_odometry(&odom);
    particle_data.particle_filter.iterate(MapInput::Pose(pose));
    publish_particle(node, particle_data)?;
    Ok(())
}

fn publish_particle(node: &Node, particle_data: &mut ParticleData) -> anyhow::Result<()> {
    let failure = particle_data.particle_filter.example_failure();
    let particle = match failure.as_ref() {
        None => particle_data.particle_filter.particles().next().unwrap(),
        Some(failure) => failure,
    };
    if !particle_data.map_saved && PathsBackTo::done(&particle) {
        let json = serde_json::to_string(&particle_data.particle_filter)?;
        let now = Local::now();
        let output_filename = format!("bit_slam_{}.json", now.format("%Y_%m_%d_%H_%M_%S"));
        std::fs::write(output_filename, json)?;
        particle_data.map_saved = true;
    }
    let data = serde_json::to_string(particle)?;
    let msg = Ros2String { data };
    particle_data.particle_publisher.publish(&msg)?;

    let grid = particle2rosgrid(node, particle)?;
    particle_data.occupancy_grid_publisher.publish(&grid)?;
    Ok(())
}

pub struct BitSlamExplorerNode {
    docs: ArgDocs,
}

impl BitSlamExplorerNode {
    pub fn goal_publish_topic(&self, args: &ArgVals) -> anyhow::Result<String> {
        Ok(format!("{}_bitslam_explorer_goal", robot_name!(args)))
    }

    pub fn stop_publish_topic(&self, args: &ArgVals) -> anyhow::Result<String> {
        Ok(format!("{}_bitslam_explorer_stop", robot_name!(args)))
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

    fn add_default(&mut self, param: &str, param_default: &str) -> anyhow::Result<()> {
        self.docs.set_default(param, param_default)
    }

    fn spec(&self, args: &ArgVals) -> anyhow::Result<NodeSpec> {
        let robot = args.get_str_value("--robot")?;
        let mut spec = NodeSpec::new(&format!("{robot}_explorer_node"), PERIOD)?;
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
    if let Some(next_step) = paths.shortest_path().and_then(|p| p.get(1).copied()) {
        eprintln!("There is a path");
        let meters = particle.map().to_meters(next_step);
        let target = particle.estimate().convert_to_raw_space(&meters);
        let msg = Ros2Point {
            x: target[0],
            y: target[1],
            z: 0.0,
        };
        if let Err(e) = point_publisher.publish(&msg) {
            eprintln!("Error {e} when trying to publish {msg:?}");
        }
    } else {
        eprintln!("There is not a path");
        let data = "stop".to_string();
        if let Err(e) = stop_publisher.publish(&Ros2String { data }) {
            eprintln!("Error {e} when trying to publish stop message");
        }
    }
}

pub fn obstacle_topic_name(robot: &str) -> String {
    format!("{robot}_obstacles")
}

pub fn particle_topic_name(robot: &str) -> String {
    format!("{robot}_maps")
}

pub fn occupancy_grid_topic_name(robot: &str) -> String {
    format!("{robot}_occupancy_grid")
}
