use crate::{
    NodeSpec, PERIOD,
    util::{parse_obstacle_distance_heading, pose_from_odometry},
};
use arg_vals::ArgVals;
use particle_filter::{ParticleFilter, ParticleFilterSettings};
use particle_filter_create3::Bump;
use r2r::{
    Publisher, irobot_create_msgs::msg::HazardDetectionVector, nav_msgs::msg::Odometry,
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
    spec.subscribe(&hazard_topic, move |hazards: HazardDetectionVector| {
        for detection in hazards.detections {
            let name = detection.header.frame_id.as_str();
            match name.parse::<Bump>() {
                Ok(bump) => {
                    let (distance, heading) = bump.obstacle_at();
                    let heading: f64 = heading.into();
                    let msg = Ros2String {
                        data: format!("({distance},{heading})"),
                    };
                    if let Err(e) = publisher.publish(&msg) {
                        eprintln!("Error publishing {msg:?}: {e}");
                    }
                }
                Err(e) => {
                    eprintln!("Error {e} parsing hazard info '{name}'");
                }
            }
        }
    })?;
    Ok(spec)
}

pub fn bit_slam_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let setup = BitSlamSetup::new(args);
    let mut spec = NodeSpec::new(&setup.node_name, setup.period)?;
    let publisher = Arc::new(Mutex::new(spec.publisher::<Ros2String>(&setup.map_topic)?));
    setup.subscribe_obstacle(&mut spec, publisher.clone())?;
    setup.subscribe_odometry(&mut spec, publisher.clone())?;
    Ok(spec)
}

pub fn bit_slam_explorer_node(args: &ArgVals) -> anyhow::Result<NodeSpec> {
    let robot_name = args.get_symbol(0);
    let mut spec = NodeSpec::new(&format!("{robot_name}_explorer"), PERIOD)?;

    Ok(spec)
}

pub fn obstacle_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_obstacles")
}

pub fn map_topic_name(robot_name: &str) -> String {
    format!("{robot_name}_maps")
}

fn publish_particle(
    publisher: Arc<Mutex<Publisher<Ros2String>>>,
    particle_filter: &ParticleFilter,
) {
    let failure = particle_filter.example_failure();
    let particle = match failure.as_ref() {
        None => particle_filter.particles().next().unwrap(),
        Some(failure) => failure,
    };
    match serde_json::to_string(particle) {
        Ok(data) => {
            let msg = Ros2String { data };
            let publisher = smol::block_on(publisher.lock());
            if let Err(e) = publisher.publish(&msg) {
                eprintln!("Error {e} when publishing {}", msg.data);
            }
        }
        Err(e) => {
            eprintln!("Error {e} when serializing particle");
        }
    };
}

struct BitSlamSetup {
    node_name: String,
    map_topic: String,
    obstacle_topic: String,
    odom_topic: String,
    particle_filter: Arc<Mutex<ParticleFilter>>,
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
            particle_filter: Arc::new(Mutex::new(ParticleFilter::new(settings))),
            period: args.get_value("-spin_time").unwrap_or(PERIOD),
        }
    }

    fn subscribe_obstacle(
        &self,
        spec: &mut NodeSpec,
        publisher: Arc<Mutex<Publisher<Ros2String>>>,
    ) -> anyhow::Result<()> {
        let particle_filter = self.particle_filter.clone();
        spec.subscribe(&self.obstacle_topic, move |obst: Ros2String| {
            match parse_obstacle_distance_heading(&obst.data) {
                Ok(obstacle) => {
                    let mut particle_filter = smol::block_on(particle_filter.lock());
                    particle_filter.iterate(None, Some(obstacle));
                    publish_particle(publisher.clone(), &particle_filter);
                }
                Err(e) => {
                    eprintln!("Error {e} parsing '{}'", obst.data);
                }
            }
        })
    }

    fn subscribe_odometry(
        &self,
        spec: &mut NodeSpec,
        publisher: Arc<Mutex<Publisher<Ros2String>>>,
    ) -> anyhow::Result<()> {
        let particle_filter = self.particle_filter.clone();
        spec.subscribe(&self.odom_topic, move |odom: Odometry| {
            let pose = pose_from_odometry(&odom);
            let mut particle_filter = smol::block_on(particle_filter.lock());
            particle_filter.iterate(Some(pose), None);
            publish_particle(publisher.clone(), &particle_filter);
        })
    }
}
