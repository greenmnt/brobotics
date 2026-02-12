use nalgebra::Vector3;
use body_km::skeleton::Skeleton;

fn main() {
    let skel = Skeleton::from_json("config/skeleton.json").expect("Failed to load skeleton");

    skel.print_tree();

    // Demonstrate ROM clamping with synthetic test rotations
    println!("\n--- ROM clamping tests ---");

    let test_cases = [
        ("HipRight", Vector3::new(-3.0, 1.0, 0.5)),
        ("KneeRight", Vector3::new(1.0, 3.0, 1.0)),
        ("AnkleRight", Vector3::new(1.0, 1.0, 1.0)),
        ("ShoulderLeft", Vector3::new(-3.5, 4.0, 2.0)),
        ("ElbowLeft", Vector3::new(1.0, 1.0, -3.0)),
        ("Neck", Vector3::new(0.8, -0.8, 1.0)),
    ];

    for (name, input) in &test_cases {
        let idx = skel.find_joint(name).expect("Joint not found");
        let clamped = skel.clamp_rotation(idx, *input);
        println!(
            "  {name:15} input({:.2}, {:.2}, {:.2}) -> clamped({:.2}, {:.2}, {:.2})",
            input.x, input.y, input.z, clamped.x, clamped.y, clamped.z
        );
    }

    // Demonstrate velocity clamping
    // Simulate sensor readings at 100 Hz (dt = 0.01s)
    println!("\n--- Velocity clamping tests (dt = 0.01s, 100 Hz) ---");

    let dt = 0.01;
    let vel_cases: [(& str, Vector3<f64>, Vector3<f64>); 4] = [
        // ShoulderLeft: prev=0, new=1.0 on all axes
        // max_vel=20 rad/s, so max_delta=0.2 per tick → clamped to 0.2
        ("ShoulderLeft", Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
        // ElbowLeft: prev=0, new=-2.0 on z
        // max_vel=25, max_delta=0.25 → clamped to -0.25
        ("ElbowLeft", Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -2.0)),
        // KneeRight: prev=1.0 on y, new=3.0 on y (big spike)
        // max_vel=20, max_delta=0.2 → clamped to 1.2 (then ROM clamps to 1.2, within 0..2.4)
        ("KneeRight", Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 3.0, 0.0)),
        // Neck: prev=(0.4, 0.4), new=(0.8, -0.8) — big jump
        // max_vel=5, max_delta=0.05 → x: 0.4+0.05=0.45, y: 0.4-0.05=0.35
        ("Neck", Vector3::new(0.4, 0.4, 0.0), Vector3::new(0.8, -0.8, 1.0)),
    ];

    for (name, prev, new) in &vel_cases {
        let idx = skel.find_joint(name).expect("Joint not found");
        let clamped = skel.clamp_velocity(idx, *prev, *new, dt);
        println!(
            "  {name:15} prev({:.2}, {:.2}, {:.2}) + new({:.2}, {:.2}, {:.2}) -> ({:.2}, {:.2}, {:.2})",
            prev.x, prev.y, prev.z, new.x, new.y, new.z, clamped.x, clamped.y, clamped.z
        );
    }
}
