use particle_filter::{
    MapInput,
    angle::{Angle, Degrees},
    irobot_create3::Bump,
    pose::RobotPose,
    pt,
};
use serde_json;

fn main() -> anyhow::Result<()> {
    let map_inputs = [
        MapInput::Pose(RobotPose::default()),
        MapInput::Pose(RobotPose {
            pos: pt!(1.0, 2.0),
            theta: Degrees::new(90.0).radians(),
        }),
        Bump::FrontCenter.obstacle_at(),
        Bump::FrontLeft.obstacle_at(),
        Bump::FrontRight.obstacle_at(),
        Bump::Left.obstacle_at(),
        Bump::Right.obstacle_at(),
    ];
    for map_input in map_inputs {
        let json = serde_json::to_string(&map_input)?;
        println!("{json}");
    }
    Ok(())
}
