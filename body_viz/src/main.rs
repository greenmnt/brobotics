use std::env;
use std::time::Instant;

use body_km::skeleton::Skeleton;
use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::window::Window;
use nalgebra::{Point2, Point3, Quaternion, UnitQuaternion};

// ---------------------------------------------------------------------------
// Sensor frame: one timestamped rotation sample
// ---------------------------------------------------------------------------

struct SensorFrame {
    timestamp: f64,
    rotation: UnitQuaternion<f64>,
}

// ---------------------------------------------------------------------------
// DMP-to-skeleton axis remap
//
// The DMP quaternion uses axes: X=left, Y=forward, Z=up
// (derived from the viz crate's empirical OpenGL remap).
// The skeleton uses:             X=forward, Y=left, Z=up
// So we swap the x and y quaternion components.
//
// Supported remap strings (each char picks which DMP axis maps to that
// skeleton axis, prefix '-' to negate):
//   "yxz"  — swap x,y          (default, DMP→skeleton)
//   "xyz"  — identity           (no remap)
//   "xzy"  — swap y,z
//   etc.
// ---------------------------------------------------------------------------

fn parse_remap(s: &str) -> [(f64, usize); 3] {
    let mut result = [(1.0, 0); 3];
    let mut chars = s.chars();
    for slot in &mut result {
        let c = chars.next().expect("remap string too short");
        let (sign, axis_char) = if c == '-' {
            (-1.0, chars.next().expect("remap string: missing axis after '-'"))
        } else {
            (1.0, c)
        };
        let idx = match axis_char {
            'x' => 0,
            'y' => 1,
            'z' => 2,
            _ => panic!("remap: invalid axis '{axis_char}', expected x/y/z"),
        };
        *slot = (sign, idx);
    }
    result
}

fn apply_remap(q: UnitQuaternion<f64>, remap: &[(f64, usize); 3]) -> UnitQuaternion<f64> {
    let v = [q.i, q.j, q.k];
    UnitQuaternion::from_quaternion(Quaternion::new(
        q.w,
        remap[0].0 * v[remap[0].1],
        remap[1].0 * v[remap[1].1],
        remap[2].0 * v[remap[2].1],
    ))
}

// ---------------------------------------------------------------------------
// CSV loader — format: timestamp,q0,q1,q2,q3  (q0 = w)
//
// 1. Each DMP quaternion is remapped from DMP axes to skeleton axes.
// 2. First frame = calibration reference (rest pose).
// 3. Relative rotation uses RIGHT-division (q_now * q_cal⁻¹) so the result
//    is in the skeleton world frame, not the sensor's local frame.
// ---------------------------------------------------------------------------

fn load_csv(path: &str, remap: &[(f64, usize); 3]) -> Vec<SensorFrame> {
    let data = std::fs::read_to_string(path).expect("Failed to read CSV file");
    let mut frames = Vec::new();
    let mut ref_quat: Option<UnitQuaternion<f64>> = None;

    for line in data.lines() {
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() != 5 {
            continue;
        }
        let vals: Option<Vec<f64>> = parts.iter().map(|s| s.trim().parse::<f64>().ok()).collect();
        let vals = match vals {
            Some(v) => v,
            None => continue,
        };

        let timestamp = vals[0];
        let raw = Quaternion::new(vals[1], vals[2], vals[3], vals[4]);
        if raw.norm_squared() < 1e-6 {
            continue;
        }

        // Remap DMP axes → skeleton axes
        let q = apply_remap(UnitQuaternion::from_quaternion(raw), remap);

        // Calibrate: first frame = rest reference
        let ref_q = *ref_quat.get_or_insert(q);

        // Right-division: relative rotation in world frame
        let relative = q * ref_q.inverse();

        frames.push(SensorFrame {
            timestamp,
            rotation: relative,
        });
    }

    // Normalise timestamps to start from 0
    if let Some(t0) = frames.first().map(|f| f.timestamp) {
        for frame in &mut frames {
            frame.timestamp -= t0;
        }
    }

    frames
}

// ---------------------------------------------------------------------------
// Demo mode — synthetic arm-raise motion (rotation around Y axis)
// ---------------------------------------------------------------------------

fn generate_demo_frames() -> Vec<SensorFrame> {
    let hz = 50.0;
    let duration = 4.0;
    let n = (hz * duration) as usize;
    let mut frames = Vec::with_capacity(n);

    for i in 0..n {
        let t = i as f64 / hz;
        // Smooth oscillation: 0 → -90° → 0 → +90° → 0 over 4 seconds
        let angle = (t * std::f64::consts::PI / 2.0).sin() * std::f64::consts::FRAC_PI_2;
        let half = angle / 2.0;
        // Rotate around Y axis (lateral axis → forward/backward arm raise)
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            half.cos(),
            0.0,
            half.sin(),
            0.0,
        ));
        frames.push(SensorFrame {
            timestamp: t,
            rotation: q,
        });
    }

    frames
}

// ---------------------------------------------------------------------------
// Coordinate mapping: skeleton → display
//   Skeleton: X = forward, Y = lateral (left +), Z = up
//   kiss3d:   X = right,   Y = up,                Z = toward camera
// Mirror Y so the skeleton's right appears on screen-right (mirror view).
// ---------------------------------------------------------------------------

fn to_display(p: &Point3<f64>) -> Point3<f32> {
    Point3::new(-p.y as f32, p.z as f32, p.x as f32)
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = env::args().collect();

    let skel_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../body_km/config/skeleton.json");
    let skel = Skeleton::from_json(skel_path).expect("Failed to load skeleton");
    println!("Loaded skeleton: {} joints", skel.joints.len());

    // Parse arguments: body_viz [joint data.csv|--demo] [--remap yxz]
    let remap_str = args
        .iter()
        .position(|a| a == "--remap")
        .map(|i| args.get(i + 1).expect("--remap requires a value").as_str())
        .unwrap_or("yxz"); // default: swap x,y (DMP → skeleton)
    let remap = parse_remap(remap_str);

    // Filter out --remap and its value for positional arg parsing
    let positional: Vec<&str> = {
        let mut v = Vec::new();
        let mut skip = false;
        for (i, a) in args.iter().enumerate().skip(1) {
            if skip {
                skip = false;
                continue;
            }
            if a == "--remap" {
                skip = true;
                continue;
            }
            let _ = i;
            v.push(a.as_str());
        }
        v
    };

    let (sensor_joint, frames) = match positional.len() {
        0 => {
            println!("Showing rest pose.");
            println!("Usage: body_viz <joint_name> <data.csv> [--remap yxz]");
            println!("       body_viz <joint_name> --demo");
            (None, vec![])
        }
        2 => {
            let idx = skel
                .find_joint(positional[0])
                .unwrap_or_else(|| panic!("Unknown joint '{}'. Available joints:", positional[0]));

            let frames = if positional[1] == "--demo" {
                println!("Demo mode: synthetic arm raise on '{}'", positional[0]);
                generate_demo_frames()
            } else {
                let f = load_csv(positional[1], &remap);
                println!(
                    "Loaded {} frames for '{}' (remap: {remap_str})",
                    f.len(),
                    positional[0]
                );
                f
            };

            (Some(idx), frames)
        }
        _ => {
            eprintln!("Usage: body_viz [joint_name data.csv|--demo] [--remap yxz]");
            std::process::exit(1);
        }
    };

    // --- kiss3d setup ---
    let mut window = Window::new("Body Kinematics");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.08, 0.08, 0.12);

    // Camera looking at the skeleton from the front
    let eye = Point3::new(0.0f32, 0.3, 2.0);
    let at = Point3::new(0.0, 0.0, 0.0);
    let mut camera = ArcBall::new(eye, at);

    let font = Font::default();
    let mut anim_time = 0.0f64;
    let mut last_instant = Instant::now();
    let mut paused = false;
    let mut frame_idx = 0usize;

    while window.render_with_camera(&mut camera) {
        let now = Instant::now();
        let real_dt = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        // --- Input ---
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::Space, Action::Press, _) => paused = !paused,
                WindowEvent::Key(Key::Equals, Action::Press, _)
                | WindowEvent::Key(Key::Add, Action::Press, _) => {
                    camera.set_dist(camera.dist() * 0.8);
                }
                WindowEvent::Key(Key::Minus, Action::Press, _)
                | WindowEvent::Key(Key::Subtract, Action::Press, _) => {
                    camera.set_dist(camera.dist() * 1.25);
                }
                WindowEvent::Key(Key::Right, Action::Press, _) => {
                    if paused && !frames.is_empty() {
                        frame_idx = (frame_idx + 1) % frames.len();
                    }
                }
                WindowEvent::Key(Key::Left, Action::Press, _) => {
                    if paused && !frames.is_empty() {
                        frame_idx = if frame_idx == 0 {
                            frames.len() - 1
                        } else {
                            frame_idx - 1
                        };
                    }
                }
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    anim_time = 0.0;
                    frame_idx = 0;
                }
                _ => {}
            }
        }

        // --- Advance animation ---
        if !paused && !frames.is_empty() {
            anim_time += real_dt;
            if let Some(last) = frames.last() {
                let duration = last.timestamp;
                if duration > 0.0 {
                    anim_time %= duration;
                }
            }
            frame_idx = frames
                .partition_point(|f| f.timestamp <= anim_time)
                .saturating_sub(1);
        }

        // --- Build per-joint rotations ---
        let n = skel.joints.len();
        let mut rotations = vec![UnitQuaternion::identity(); n];
        if let Some(ji) = sensor_joint {
            if let Some(frame) = frames.get(frame_idx) {
                rotations[ji] = frame.rotation;
            }
        }

        // --- Forward kinematics ---
        let positions = skel.forward_kinematics(&rotations);

        // --- Draw ground grid ---
        let grid_color = Point3::new(0.25f32, 0.25, 0.25);
        let ground_y = -0.87f32; // just below ankles
        for i in -5..=5 {
            let v = i as f32 * 0.2;
            window.draw_line(
                &Point3::new(-1.0, ground_y, v),
                &Point3::new(1.0, ground_y, v),
                &grid_color,
            );
            window.draw_line(
                &Point3::new(v, ground_y, -1.0),
                &Point3::new(v, ground_y, 1.0),
                &grid_color,
            );
        }

        // --- Draw bones ---
        let bone_color = Point3::new(0.85f32, 0.85, 0.85);
        let active_color = Point3::new(0.2f32, 1.0, 0.3);

        for (i, joint) in skel.joints.iter().enumerate() {
            if let Some(pi) = joint.parent {
                let color = if sensor_joint == Some(i) {
                    &active_color
                } else {
                    &bone_color
                };
                let a = to_display(&positions[pi]);
                let b = to_display(&positions[i]);
                window.draw_line(&a, &b, color);
            }
        }

        // --- Draw joint markers (small 3D crosses) ---
        let ms = 0.012f32;
        let joint_color = Point3::new(1.0f32, 1.0, 1.0);

        for (i, pos) in positions.iter().enumerate() {
            let p = to_display(pos);
            let c = if sensor_joint == Some(i) {
                &active_color
            } else {
                &joint_color
            };
            window.draw_line(
                &Point3::new(p.x - ms, p.y, p.z),
                &Point3::new(p.x + ms, p.y, p.z),
                c,
            );
            window.draw_line(
                &Point3::new(p.x, p.y - ms, p.z),
                &Point3::new(p.x, p.y + ms, p.z),
                c,
            );
            window.draw_line(
                &Point3::new(p.x, p.y, p.z - ms),
                &Point3::new(p.x, p.y, p.z + ms),
                c,
            );
        }

        // --- HUD ---
        let white = Point3::new(1.0f32, 1.0, 1.0);
        let hud = if let Some(ji) = sensor_joint {
            if frames.is_empty() {
                format!("Sensor: {} (no data)", skel.joints[ji].name)
            } else {
                format!(
                    "Sensor: {}  remap: {}\nFrame: {}/{}\nTime: {:.2}s{}",
                    skel.joints[ji].name,
                    remap_str,
                    frame_idx + 1,
                    frames.len(),
                    frames.get(frame_idx).map_or(0.0, |f| f.timestamp),
                    if paused { "  [PAUSED]" } else { "" },
                )
            }
        } else {
            "Rest pose\n\n+/-: zoom   Space: pause\nArrows: step   R: reset".to_string()
        };
        window.draw_text(&hud, &Point2::new(10.0, 20.0), 36.0, &font, &white);
    }
}
