//! MPU6050 DMP Quaternion Visualizer
//!
//! Rust port of the Processing MPUTeapot.pde sketch.
//! Reads quaternion data from stdin (lines: "quat\tw\tx\ty\tz")
//! and renders a 3D airplane shape rotated by the quaternion.
//!
//! Usage:
//!   probe-rs run --chip STM32F334R8Tx target/... 2>&1 | cargo run
//!   # or pipe any source of "quat\tw\tx\ty\tz" lines

use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::resource::Mesh;
use kiss3d::text::Font;
use kiss3d::window::Window;
use nalgebra::{Point2, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::cell::RefCell;
use std::io::{BufReader, BufRead};
use std::rc::Rc;
use std::fs::File;
use std::sync::{Arc, Mutex};

fn parse_quaternion_line(line: &str) -> Option<[f32; 4]> {
    let comps: Vec<f32> = line
        .split(',')
        .map(|s| s.trim().parse::<f32>().ok())
        .collect::<Option<Vec<f32>>>()?;

    if comps.len() != 5 {
        return None;
    }
    Some([comps[1], comps[2], comps[3], comps[4]])
}


const SCALE: f32 = 0.01;

fn main() {
    // Shared quaternion state (w, x, y, z) — updated by reader thread
    let quat_data = Arc::new(Mutex::new([1.0f32, 0.0, 0.0, 0.0]));

    // Spawn stdin reader thread
    {
        let qd = quat_data.clone();
        std::thread::spawn(move || {
            let file = File::open("/tmp/imu_out").expect("Failed to open IMU pipe");
            let reader = BufReader::new(file);

            for line in reader.lines() {
                if let Err(e) = line {
                    println!("Read error: {:?}", e);
                    continue;
                }
                let line = line.unwrap();
                if let Some(qline) = parse_quaternion_line(&line) {
                    println!("{:?}", qline);
                    let mut shared = qd.lock().unwrap();
                    *shared = qline;
                    //thread::sleep(Duration::from_millis(5));
                } else {
                    println!("Error parsing: {:?}", line);
                }
            }
        });
    }

    // Create window with explicit ArcBall camera at a comfortable distance
    let mut window = Window::new("MPU6050 Quaternion Visualizer");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.0, 0.0, 0.0);

    let eye = Point3::new(0.0f32, 1.0, 5.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new(eye, at);

    // All geometry in a group node for unified rotation
    let mut group = window.add_group();

    // Body: red box (10×10×200 in Processing units)
    // kiss3d add_cube takes half-extents
    let mut body = group.add_cube(0.05, 0.05, 1.0);
    body.set_color(1.0, 0.0, 0.0);

    // Nose cone: blue, at front of aircraft
    // Processing: drawCylinder(0, 20, 20, 8) at z=-120, rotated PI/2 around X
    // In OpenGL coords (z flipped): position at z=+1.2
    let mut nose = group.add_cone(0.1, 0.1);
    nose.set_color(0.0, 0.0, 1.0);
    nose.set_local_translation(Translation3::new(0.0, 0.0, 1.2));
    // Rotate cone from Y-axis to point along +Z
    nose.set_local_rotation(UnitQuaternion::from_axis_angle(
        &Vector3::x_axis(),
        -std::f32::consts::FRAC_PI_2,
    ));

    // Wings and tail fin: green custom mesh
    let mesh = build_wings_tail();
    let mut wings = group.add_mesh(mesh, Vector3::new(1.0, 1.0, 1.0));
    wings.set_color(0.0, 1.0, 0.0);
    wings.enable_backface_culling(false);

    let font = Font::default();

    // Render loop
    while window.render_with_camera(&mut camera) {
        // Keyboard zoom: +/= to zoom in, - to zoom out
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::Equals, Action::Press, _)
                | WindowEvent::Key(Key::Add, Action::Press, _) => {
                    let dist = camera.dist();
                    camera.set_dist(dist * 0.8);
                }
                WindowEvent::Key(Key::Minus, Action::Press, _)
                | WindowEvent::Key(Key::Subtract, Action::Press, _) => {
                    let dist = camera.dist();
                    camera.set_dist(dist * 1.25);
                }
                _ => {}
            }
        }

        let q = *quat_data.lock().unwrap();

        // Remap DMP quaternion for OpenGL coordinate system
        // Processing does: rotate(angle, -axis[1], axis[3], axis[2])
        // which maps quaternion (w,x,y,z) → (w, -x, z, y)
        let raw = Quaternion::new(q[0], -q[1], q[3], q[2]);
        // Guard against zero/degenerate quaternion (NaN poisons the transform matrix)
        if raw.norm_squared() > 1e-6 {
            let rotation = UnitQuaternion::from_quaternion(raw);
            group.set_local_rotation(rotation);
        }

        // Compute yaw/pitch/roll from the raw DMP quaternion
        // (same formulas as MPU6050_6Axis_MotionApps20.cpp)
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let gx = 2.0 * (x * z - w * y);
        let gy = 2.0 * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        let yaw = (2.0 * x * y - 2.0 * w * z).atan2(2.0 * w * w + 2.0 * x * x - 1.0);
        let pitch = gx.atan2((gy * gy + gz * gz).sqrt());
        let roll = gy.atan2(gz);

        let deg = 180.0 / std::f32::consts::PI;
        let text = format!(
            "Yaw:   {:+7.1}\nPitch: {:+7.1}\nRoll:  {:+7.1}",
            yaw * deg,
            pitch * deg,
            roll * deg,
        );
        let white = Point3::new(1.0, 1.0, 1.0);
        window.draw_text(&text, &Point2::new(10.0, 20.0), 48.0, &font, &white);
    }
}

/// Build the wings and tail fin as a triangle mesh.
/// Geometry matches the Processing MPUTeapot.pde sketch.
/// Processing coords (x, y_down, z_in) → OpenGL (x, -y, -z), scaled by SCALE.
fn build_wings_tail() -> Rc<RefCell<Mesh>> {
    // Helper: Processing coords → OpenGL coords, scaled
    let p =
        |x: f32, y: f32, z: f32| -> Point3<f32> { Point3::new(x * SCALE, -y * SCALE, -z * SCALE) };

    let vertices = vec![
        // Wing top face (y=2 in Processing)
        p(-100.0, 2.0, 30.0), // 0
        p(0.0, 2.0, -80.0),   // 1
        p(100.0, 2.0, 30.0),  // 2
        // Wing bottom face (y=-2)
        p(-100.0, -2.0, 30.0), // 3
        p(0.0, -2.0, -80.0),   // 4
        p(100.0, -2.0, 30.0),  // 5
        // Tail left face (x=-2)
        p(-2.0, 0.0, 98.0),   // 6
        p(-2.0, -30.0, 98.0), // 7
        p(-2.0, 0.0, 70.0),   // 8
        // Tail right face (x=2)
        p(2.0, 0.0, 98.0),   // 9
        p(2.0, -30.0, 98.0), // 10
        p(2.0, 0.0, 70.0),   // 11
    ];

    let faces = vec![
        // Wing surface triangles
        Point3::new(0u16, 1, 2), // wing top
        Point3::new(3, 4, 5),    // wing bottom
        // Tail surface triangles
        Point3::new(6, 7, 8),   // tail left
        Point3::new(9, 10, 11), // tail right
        // Wing side quads (as triangle pairs)
        // Left wing side: v0, v3, v4, v1
        Point3::new(0, 3, 4),
        Point3::new(0, 4, 1),
        // Right wing side: v2, v5, v4, v1
        Point3::new(2, 5, 4),
        Point3::new(2, 4, 1),
        // Wing trailing edge: v0, v3, v5, v2
        Point3::new(0, 3, 5),
        Point3::new(0, 5, 2),
        // Tail back face: v6, v9, v10, v7
        Point3::new(6, 9, 10),
        Point3::new(6, 10, 7),
        // Tail bottom: v6, v9, v11, v8
        Point3::new(6, 9, 11),
        Point3::new(6, 11, 8),
        // Tail front: v7, v10, v11, v8
        Point3::new(7, 10, 11),
        Point3::new(7, 11, 8),
    ];

    Rc::new(RefCell::new(Mesh::new(vertices, faces, None, None, false)))
}
