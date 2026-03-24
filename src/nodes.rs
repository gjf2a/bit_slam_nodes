use crate::{
    NodeSpec, PERIOD,
    util::{parse_obstacle_distance_heading, pose_from_odometry},
};
use arg_vals::ArgVals;
use particle_filter::{Particle, ParticleFilter, ParticleFilterSettings, path_plan::paths_from};
use particle_filter_create3::Bump;
use r2r::{
    Publisher, geometry_msgs::msg::Point as Ros2Point, irobot_create_msgs::msg::HazardDetectionVector, nav_msgs::msg::Odometry,
    std_msgs::msg::String as Ros2String,
};
use smol::lock::Mutex;
use std::sync::Arc;

pub fn bump_obstacle_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let robot_name = args.get_symbol(0);
    let hazard_topic = format!("{robot_name}/hazard_detection");
    let mut spec = NodeSpec::new(format!("{robot_name}_obstacle_node").as_str(), PERIOD)?;
    let publish_topic = obstacle_topic_name(robot_name);
    let publisher = spec.publisher::<Ros2String>(&publish_topic)?;
    println!("Publishing on {publish_topic}");
    spec.subscribe(&hazard_topic, move |hazards: HazardDetectionVector, _| {
        for detection in hazards.detections {
            if let Err(e) = publish_obstacle_location(&publisher, &detection.header.frame_id) {
                eprintln!("Error {e} publishing hazard {}", detection.header.frame_id);
            }
        }
    })?;
    Ok(spec)
}

fn publish_obstacle_location(
    publisher: &Publisher<Ros2String>,
    frame_id: &str,
) -> anyhow::Result<()> {
    let bump = frame_id.parse::<Bump>()?;
    let (distance, heading) = bump.obstacle_at();
    let heading: f64 = heading.into();
    let msg = Ros2String {
        data: format!("({distance},{heading})"),
    };
    publisher.publish(&msg)?;
    Ok(())
}

pub fn bit_slam_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let setup = BitSlamSetup::new(args);
    let mut spec = NodeSpec::new(&setup.node_name, setup.period)?;
    let particle_filter = setup.create_particle_filter();
    let publisher = spec.publisher::<Ros2String>(&setup.map_topic)?;
    let particle_data = Arc::new(Mutex::new(ParticleData {particle_filter, publisher}));
    setup.subscribe_obstacle(&mut spec, particle_data.clone())?;
    setup.subscribe_odometry(&mut spec, particle_data.clone())?;
    Ok(spec)
}

pub fn bit_slam_explorer_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let robot_name = args.get_symbol(0);
    let mut spec = NodeSpec::new(&format!("{robot_name}_explorer_node"), PERIOD)?;
    let target_topic = format!("{robot_name}_bitslam_explorer_goal");
    let publisher = spec.publisher::<Ros2Point>(&target_topic)?;
    spec.subscribe(&map_topic_name(robot_name), move |particle_str: Ros2String, _| {
        match serde_json::from_str::<Particle>(&particle_str.data) {
            Ok(particle) => {
                let paths = paths_from(particle.map(), particle.estimated_pose());
                if let Some(next_step) = paths.shortest_path().and_then(|p| p.get(1).copied()) {
                    let meters = particle.map().to_meters(next_step);
                    let target = particle.estimate().convert_to_raw_space(&meters);
                    let msg = Ros2Point {x: target[0], y: target[1], z: 0.0};
                    if let Err(e) = publisher.publish(&msg) {
                        eprintln!("Error {e} when trying to publish {msg:?} to {target_topic}");
                    }
                }
            }
            Err(e) => {
                eprintln!("Error {e} when deserializing particle");
            }
        }
    })?;
    Ok(spec)
}

pub fn obstacle_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_obstacles")
}

pub fn map_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_maps")
}

struct ParticleData {
    particle_filter: ParticleFilter,
    publisher: Publisher<Ros2String>,
}

struct BitSlamSetup {
    node_name: String,
    map_topic: String,
    obstacle_topic: String,
    odom_topic: String,
    settings: ParticleFilterSettings,
    period: u64,
}

impl BitSlamSetup {
    fn new(args: &ArgVals) -> Self {
        let robot_name = format!("/{}", args.get_symbol(0));
        let mut settings = ParticleFilterSettings::default();
        if let Some(num_particles) = args.get_value("-num_particles") {
            settings.num_particles = num_particles;
        }
        if let Some(meters_per_cell) = args.get_value("-meters_per_cell") {
            settings.square_size_m = meters_per_cell;
        }
        Self {
            node_name: format!("{robot_name}_bitslam_node"),
            map_topic: map_topic_name(&robot_name),
            obstacle_topic: obstacle_topic_name(&robot_name),
            odom_topic: format!("{robot_name}/odom"),
            settings,
            period: args.get_value("-spin_time").unwrap_or(PERIOD),
        }
    }

    fn create_particle_filter(&self) -> ParticleFilter {
        ParticleFilter::new(self.settings.clone())
    }

    fn subscribe_obstacle(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.obstacle_topic, move |obst: Ros2String, _| {
            let mut particle_data = smol::block_on(particle_data.lock());
            if let Err(e) = publish_particle_obstacle(&obst, &mut particle_data) {
                eprintln!("Error {e} when updating particle filter with {}", obst.data);
            }
        })
    }

    fn subscribe_odometry(
        &self,
        spec: &mut NodeSpec,
        particle_data: Arc<Mutex<ParticleData>>,
    ) -> anyhow::Result<()> {
        spec.subscribe(&self.odom_topic, move |odom: Odometry, _| {
            let mut particle_data = smol::block_on(particle_data.lock());
            if let Err(e) = publish_particle_odom(&odom, &mut particle_data) {
                eprintln!("Error {e} when updating particle filter with {odom:?}");
            }
        })
    }
}

fn publish_particle_obstacle(
    obst: &Ros2String,
    particle_data: &mut ParticleData,
) -> anyhow::Result<()> {
    let obstacle = parse_obstacle_distance_heading(&obst.data)?;
    particle_data.particle_filter.iterate(None, Some(obstacle));
    publish_particle(particle_data)?;
    Ok(())
}

fn publish_particle_odom(
    odom: &Odometry, particle_data: &mut ParticleData
) -> anyhow::Result<()> {
    let pose = pose_from_odometry(&odom);
    particle_data.particle_filter.iterate(Some(pose), None);
    publish_particle(particle_data)?;
    Ok(())
}

fn publish_particle(particle_data: &ParticleData) -> anyhow::Result<()> {
    let failure = particle_data.particle_filter.example_failure();
    let particle = match failure.as_ref() {
        None => particle_data.particle_filter.particles().next().unwrap(),
        Some(failure) => failure,
    };
    let data = serde_json::to_string(particle)?;
    let msg = Ros2String { data };
    particle_data.publisher.publish(&msg)?;
    Ok(())
}
