use std::collections::BTreeMap;
use std::env;
use std::path::Path;
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
// Sensor track: all frames for one joint
// ---------------------------------------------------------------------------

struct SensorTrack {
    joint_idx: usize,
    frames: Vec<SensorFrame>,
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
// CSV loader — auto-detects format:
//   5 columns: timestamp,q0,q1,q2,q3               (single sensor, old)
//   6 columns: channel,timestamp,q0,q1,q2,q3       (multi sensor, new)
//
// Returns Vec<(channel_id, frames)> sorted by channel_id.
// Per-channel calibration: first frame per channel = rest reference.
// Global timestamp normalization: subtract min t0 across all channels.
// ---------------------------------------------------------------------------

fn load_csv_tracks(path: &str, remap: &[(f64, usize); 3]) -> Vec<(u32, Vec<SensorFrame>)> {
    let data = std::fs::read_to_string(path).expect("Failed to read CSV file");

    // Auto-detect column count from first parseable line
    let ncols = data
        .lines()
        .find_map(|line| {
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 5 && parts[0].trim().parse::<f64>().is_ok() {
                Some(parts.len())
            } else {
                None
            }
        })
        .unwrap_or(5);

    let multi = ncols >= 6;

    // Collect raw rows per channel: channel -> Vec<(timestamp, raw_quat)>
    let mut channels: BTreeMap<u32, Vec<(f64, UnitQuaternion<f64>)>> = BTreeMap::new();

    for line in data.lines() {
        let parts: Vec<&str> = line.split(',').collect();
        if (multi && parts.len() < 6) || (!multi && parts.len() < 5) {
            continue;
        }

        let vals: Option<Vec<f64>> = parts.iter().map(|s| s.trim().parse::<f64>().ok()).collect();
        let vals = match vals {
            Some(v) => v,
            None => continue,
        };

        let (channel, timestamp, q0, q1, q2, q3) = if multi {
            (vals[0] as u32, vals[1], vals[2], vals[3], vals[4], vals[5])
        } else {
            (0, vals[0], vals[1], vals[2], vals[3], vals[4])
        };

        let raw = Quaternion::new(q0, q1, q2, q3);
        if raw.norm_squared() < 1e-6 {
            continue;
        }

        let q = apply_remap(UnitQuaternion::from_quaternion(raw), remap);
        channels.entry(channel).or_default().push((timestamp, q));
    }

    // Find global t0 (min first timestamp across all channels)
    let global_t0 = channels
        .values()
        .filter_map(|rows| rows.first().map(|(t, _)| *t))
        .fold(f64::INFINITY, f64::min);
    let global_t0 = if global_t0.is_finite() { global_t0 } else { 0.0 };

    // Per-channel: calibrate + normalize timestamps
    channels
        .into_iter()
        .map(|(ch, rows)| {
            let mut ref_quat: Option<UnitQuaternion<f64>> = None;
            let frames = rows
                .into_iter()
                .map(|(t, q)| {
                    let ref_q = *ref_quat.get_or_insert(q);
                    let relative = q * ref_q.inverse();
                    SensorFrame {
                        timestamp: t - global_t0,
                        rotation: relative,
                    }
                })
                .collect();
            (ch, frames)
        })
        .collect()
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
// Sensor color palette
// ---------------------------------------------------------------------------

fn sensor_color(track_index: usize) -> Point3<f32> {
    let palette: [(f32, f32, f32); 6] = [
        (0.2, 1.0, 0.3),   // green
        (0.3, 0.6, 1.0),   // blue
        (1.0, 0.6, 0.2),   // orange
        (1.0, 0.4, 0.7),   // pink
        (1.0, 1.0, 0.3),   // yellow
        (0.3, 1.0, 1.0),   // cyan
    ];
    let (r, g, b) = palette[track_index % palette.len()];
    Point3::new(r, g, b)
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = env::args().collect();

    let skel_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../body_km/config/skeleton.json");
    let skel = Skeleton::from_json(skel_path).expect("Failed to load skeleton");
    println!("Loaded skeleton: {} joints", skel.joints.len());

    // Parse --remap flag
    let remap_str = args
        .iter()
        .position(|a| a == "--remap")
        .map(|i| args.get(i + 1).expect("--remap requires a value").as_str())
        .unwrap_or("yxz");
    let remap = parse_remap(remap_str);

    // Filter out --remap and its value for positional arg parsing
    let positional: Vec<&str> = {
        let mut v = Vec::new();
        let mut skip = false;
        for a in args.iter().skip(1) {
            if skip {
                skip = false;
                continue;
            }
            if a == "--remap" {
                skip = true;
                continue;
            }
            v.push(a.as_str());
        }
        v
    };

    // Determine mode and build tracks
    //
    // Formats supported:
    //   (no args)                              → rest pose
    //   <joint> --demo                         → demo mode (old)
    //   --demo <joint>                         → demo mode (alt)
    //   <joint> <csv>                          → single sensor (old format)
    //   <csv> <joint1> [joint2 ...]            → multi sensor (new format)
    //
    // Heuristic: if positional[0] is an existing file path, use new format.

    let tracks: Vec<SensorTrack> = if positional.is_empty() {
        println!("Showing rest pose.");
        println!(
            "Usage: body_viz <csv_file> <joint1> [joint2 ...] [--remap yxz]"
        );
        println!("       body_viz <joint_name> <data.csv|--demo>");
        vec![]
    } else if positional.contains(&"--demo") {
        // Demo mode: find the joint name (the arg that isn't --demo)
        let joint_name = positional
            .iter()
            .find(|a| **a != "--demo")
            .expect("--demo requires a joint name");
        let idx = skel
            .find_joint(joint_name)
            .unwrap_or_else(|| panic!("Unknown joint '{joint_name}'"));
        println!("Demo mode: synthetic arm raise on '{joint_name}'");
        vec![SensorTrack {
            joint_idx: idx,
            frames: generate_demo_frames(),
        }]
    } else if positional.len() == 2 && !Path::new(positional[0]).exists() {
        // Old format: <joint> <csv>
        let joint_name = positional[0];
        let csv_path = positional[1];
        let idx = skel
            .find_joint(joint_name)
            .unwrap_or_else(|| panic!("Unknown joint '{joint_name}'"));
        let csv_tracks = load_csv_tracks(csv_path, &remap);
        let frames = csv_tracks
            .into_iter()
            .next()
            .map(|(_, f)| f)
            .unwrap_or_default();
        println!(
            "Loaded {} frames for '{}' (remap: {remap_str})",
            frames.len(),
            joint_name
        );
        vec![SensorTrack {
            joint_idx: idx,
            frames,
        }]
    } else if Path::new(positional[0]).exists() {
        // New format: <csv> <joint1> [joint2 ...]
        let csv_path = positional[0];
        let joint_names = &positional[1..];
        if joint_names.is_empty() {
            eprintln!("Error: CSV file provided but no joint names specified.");
            eprintln!("Usage: body_viz <csv_file> <joint1> [joint2 ...]");
            std::process::exit(1);
        }
        let csv_tracks = load_csv_tracks(csv_path, &remap);
        let mut tracks = Vec::new();
        for (i, &joint_name) in joint_names.iter().enumerate() {
            let idx = skel
                .find_joint(joint_name)
                .unwrap_or_else(|| panic!("Unknown joint '{joint_name}'"));
            let frames = csv_tracks
                .iter()
                .find(|(ch, _)| *ch == i as u32)
                .map(|(_, f)| {
                    f.iter()
                        .map(|sf| SensorFrame {
                            timestamp: sf.timestamp,
                            rotation: sf.rotation,
                        })
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            println!(
                "Channel {} → '{}': {} frames (remap: {remap_str})",
                i,
                joint_name,
                frames.len()
            );
            tracks.push(SensorTrack {
                joint_idx: idx,
                frames,
            });
        }
        tracks
    } else {
        // Assume old format: first arg is joint name
        let joint_name = positional[0];
        let idx = skel
            .find_joint(joint_name)
            .unwrap_or_else(|| panic!("Unknown joint '{joint_name}'. Is it a file path or joint name?"));
        if positional.len() < 2 {
            eprintln!("Error: joint name provided but no data source.");
            eprintln!("Usage: body_viz <joint_name> <data.csv|--demo>");
            std::process::exit(1);
        }
        let csv_path = positional[1];
        let csv_tracks = load_csv_tracks(csv_path, &remap);
        let frames = csv_tracks
            .into_iter()
            .next()
            .map(|(_, f)| f)
            .unwrap_or_default();
        println!(
            "Loaded {} frames for '{}' (remap: {remap_str})",
            frames.len(),
            joint_name
        );
        vec![SensorTrack {
            joint_idx: idx,
            frames,
        }]
    };

    // Compute animation duration (max timestamp across all tracks)
    let anim_duration = tracks
        .iter()
        .filter_map(|t| t.frames.last().map(|f| f.timestamp))
        .fold(0.0f64, f64::max);

    // Build a lookup: joint_idx → track index (for coloring)
    let mut joint_to_track: Vec<Option<usize>> = vec![None; skel.joints.len()];
    for (ti, track) in tracks.iter().enumerate() {
        joint_to_track[track.joint_idx] = Some(ti);
    }

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

    // Per-track frame index (for manual stepping)
    let mut frame_indices: Vec<usize> = vec![0; tracks.len()];

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
                    if paused {
                        for (ti, idx) in frame_indices.iter_mut().enumerate() {
                            if !tracks[ti].frames.is_empty() {
                                *idx = (*idx + 1) % tracks[ti].frames.len();
                            }
                        }
                    }
                }
                WindowEvent::Key(Key::Left, Action::Press, _) => {
                    if paused {
                        for (ti, idx) in frame_indices.iter_mut().enumerate() {
                            let len = tracks[ti].frames.len();
                            if len > 0 {
                                *idx = if *idx == 0 { len - 1 } else { *idx - 1 };
                            }
                        }
                    }
                }
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    anim_time = 0.0;
                    for idx in &mut frame_indices {
                        *idx = 0;
                    }
                }
                _ => {}
            }
        }

        // --- Advance animation ---
        let has_frames = tracks.iter().any(|t| !t.frames.is_empty());
        if !paused && has_frames {
            anim_time += real_dt;
            if anim_duration > 0.0 {
                anim_time %= anim_duration;
            }
            // Update frame indices via partition_point for each track
            for (ti, track) in tracks.iter().enumerate() {
                if !track.frames.is_empty() {
                    frame_indices[ti] = track
                        .frames
                        .partition_point(|f| f.timestamp <= anim_time)
                        .saturating_sub(1);
                }
            }
        }

        // --- Build per-joint rotations ---
        let n = skel.joints.len();
        let mut rotations = vec![UnitQuaternion::identity(); n];
        for (ti, track) in tracks.iter().enumerate() {
            if let Some(frame) = track.frames.get(frame_indices[ti]) {
                rotations[track.joint_idx] = frame.rotation;
            }
        }

        // --- Forward kinematics ---
        let positions = skel.forward_kinematics(&rotations);

        // --- Draw ground grid ---
        let grid_color = Point3::new(0.25f32, 0.25, 0.25);
        let ground_y = -0.87f32;
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

        for (i, joint) in skel.joints.iter().enumerate() {
            if let Some(pi) = joint.parent {
                let color = match joint_to_track[i] {
                    Some(ti) => sensor_color(ti),
                    None => bone_color,
                };
                let a = to_display(&positions[pi]);
                let b = to_display(&positions[i]);
                window.draw_line(&a, &b, &color);
            }
        }

        // --- Draw joint markers (small 3D crosses) ---
        let ms = 0.012f32;
        let joint_color = Point3::new(1.0f32, 1.0, 1.0);

        for (i, pos) in positions.iter().enumerate() {
            let p = to_display(pos);
            let c = match joint_to_track[i] {
                Some(ti) => sensor_color(ti),
                None => joint_color,
            };
            window.draw_line(
                &Point3::new(p.x - ms, p.y, p.z),
                &Point3::new(p.x + ms, p.y, p.z),
                &c,
            );
            window.draw_line(
                &Point3::new(p.x, p.y - ms, p.z),
                &Point3::new(p.x, p.y + ms, p.z),
                &c,
            );
            window.draw_line(
                &Point3::new(p.x, p.y, p.z - ms),
                &Point3::new(p.x, p.y, p.z + ms),
                &c,
            );
        }

        // --- HUD ---
        let white = Point3::new(1.0f32, 1.0, 1.0);
        let hud = if tracks.is_empty() {
            "Rest pose\n\n+/-: zoom   Space: pause\nArrows: step   R: reset".to_string()
        } else if !has_frames {
            let names: Vec<&str> = tracks
                .iter()
                .map(|t| skel.joints[t.joint_idx].name.as_str())
                .collect();
            format!("Sensors: {} (no data)", names.join(", "))
        } else {
            let mut lines = Vec::new();
            // Sensor names with their colors indicated by index
            let sensor_names: Vec<String> = tracks
                .iter()
                .enumerate()
                .map(|(ti, t)| {
                    let name = &skel.joints[t.joint_idx].name;
                    let fi = frame_indices[ti];
                    let total = t.frames.len();
                    if total > 0 {
                        format!("[{}] {} ({}/{})", ti, name, fi + 1, total)
                    } else {
                        format!("[{}] {} (no data)", ti, name)
                    }
                })
                .collect();
            lines.push(format!("Sensors: {}  remap: {}", sensor_names.join("  "), remap_str));
            lines.push(format!(
                "Time: {:.2}s / {:.2}s{}",
                anim_time,
                anim_duration,
                if paused { "  [PAUSED]" } else { "" },
            ));
            lines.join("\n")
        };
        window.draw_text(&hud, &Point2::new(10.0, 20.0), 36.0, &font, &white);
    }
}
