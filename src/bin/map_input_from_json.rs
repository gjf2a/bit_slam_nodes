use bit_slam_nodes::util::StampedString;
use particle_filter::{
    MapInput,
    angle::Radians,
    pose::RobotPose,
    pt,
};
use serde_json;

fn main() {
    let test = r#"{"Pose":{"pos":{"coords":[0.32544589042663574,0.008315492421388626]},"theta":-0.003339462670669313}}"#;
    println!("{test}");
    let src = MapInput::Pose(RobotPose {pos: pt!(0.32544589042663574,0.008315492421388626), theta: Radians::new(-0.003339462670669313)});
    println!("{}", serde_json::to_string(&src).unwrap());
    let regen = serde_json::from_str::<MapInput>(test).unwrap();
    println!("{regen:?}");

    println!();
    let test2 = r#"{"header": {"stamp": {"sec": 57836, "nanosec": 917077120}, "frame_id": "odom"}, "data": {"Pose": {"pos": {"coords": [1.223861813545227, 0.03784951940178871]}, "theta": -0.009354704342699804}}}"#;
    let regen2 = serde_json::from_str::<StampedString>(test2).unwrap();
    println!("regen2: {regen2:?}");
}
