use arg_vals::ArgVals;
use particle_filter::ParticleFilterSettings;

fn main() {
    let args = ArgVals::default();
    if args.len() < 1 {
        println!(
            "Usage: particle_filter_node robot_name [-num_particles=n] [-spin_time=millseconds] [-meters_per_cell=mps]"
        );
    } else {
        let mut settings = ParticleFilterSettings::default();
        if let Some(num_particles) = args.get_value("-num_particles") {
            settings.num_particles = num_particles;
        }
        if let Some(meters_per_cell) = args.get_value("-meters_per_cell") {
            settings.square_size_m = meters_per_cell;
        }
        let period = args.get_value("-spin_time").unwrap_or(0.1);
        
    }
}
