use std::collections::BTreeMap;
use std::env;
use std::io::{self, BufRead, BufReader};
use std::path::Path;
use std::sync::mpsc;
use std::thread;
use std::time::Instant;

use body_km::skeleton::Skeleton;
use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::window::Window;
use nalgebra::{Point2, Point3, Quaternion, UnitQuaternion, Vector3};

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
// View projections: skeleton coords → display coords
//   Skeleton: X = forward, Y = lateral (left +), Z = up
//   kiss3d:   X = right,   Y = up,                Z = toward camera
// ---------------------------------------------------------------------------

/// Front view (existing): looking at the skeleton from in front
fn to_display_front(p: &Point3<f64>) -> Point3<f32> {
    Point3::new(-p.y as f32, p.z as f32, p.x as f32)
}

/// Side view: looking from the right side (looking in +Y direction)
fn to_display_side(p: &Point3<f64>) -> Point3<f32> {
    Point3::new(p.x as f32, p.z as f32, p.y as f32)
}

/// Top-down view: looking from above (looking in -Z direction)
fn to_display_top(p: &Point3<f64>) -> Point3<f32> {
    Point3::new(-p.y as f32, -p.x as f32, -p.z as f32)
}

// ---------------------------------------------------------------------------
// Drawing helpers (shared by playback and live modes)
// ---------------------------------------------------------------------------

fn draw_ground_grid_at(window: &mut Window, offset: &Point3<f32>, scale: f32) {
    let grid_color = Point3::new(0.25f32, 0.25, 0.25);
    let ground_y = -0.87f32 * scale + offset.y;
    for i in -5..=5 {
        let v = i as f32 * 0.2 * scale;
        window.draw_line(
            &Point3::new(-1.0 * scale + offset.x, ground_y, v + offset.z),
            &Point3::new(1.0 * scale + offset.x, ground_y, v + offset.z),
            &grid_color,
        );
        window.draw_line(
            &Point3::new(v + offset.x, ground_y, -1.0 * scale + offset.z),
            &Point3::new(v + offset.x, ground_y, 1.0 * scale + offset.z),
            &grid_color,
        );
    }
}

fn draw_skeleton_at(
    window: &mut Window,
    skel: &Skeleton,
    positions: &[Point3<f64>],
    joint_to_track: &[Option<usize>],
    project: fn(&Point3<f64>) -> Point3<f32>,
    offset: &Point3<f32>,
    scale: f32,
) {
    let bone_color = Point3::new(0.85f32, 0.85, 0.85);
    for (i, joint) in skel.joints.iter().enumerate() {
        if let Some(pi) = joint.parent {
            let color = match joint_to_track[i] {
                Some(ti) => sensor_color(ti),
                None => bone_color,
            };
            let a = project(&positions[pi]);
            let b = project(&positions[i]);
            window.draw_line(
                &Point3::new(a.x * scale + offset.x, a.y * scale + offset.y, a.z * scale + offset.z),
                &Point3::new(b.x * scale + offset.x, b.y * scale + offset.y, b.z * scale + offset.z),
                &color,
            );
        }
    }

    let ms = 0.012f32 * scale;
    let joint_color = Point3::new(1.0f32, 1.0, 1.0);
    for (i, pos) in positions.iter().enumerate() {
        let p = project(pos);
        let px = p.x * scale + offset.x;
        let py = p.y * scale + offset.y;
        let pz = p.z * scale + offset.z;
        let c = match joint_to_track[i] {
            Some(ti) => sensor_color(ti),
            None => joint_color,
        };
        window.draw_line(
            &Point3::new(px - ms, py, pz),
            &Point3::new(px + ms, py, pz),
            &c,
        );
        window.draw_line(
            &Point3::new(px, py - ms, pz),
            &Point3::new(px, py + ms, pz),
            &c,
        );
        window.draw_line(
            &Point3::new(px, py, pz - ms),
            &Point3::new(px, py, pz + ms),
            &c,
        );
    }
}

/// Draw skeleton from three views: front (center), side (left), top (right)
fn draw_all_views(
    window: &mut Window,
    skel: &Skeleton,
    positions: &[Point3<f64>],
    joint_to_track: &[Option<usize>],
    font: &std::rc::Rc<Font>,
) {
    let s = 0.6f32; // scale for multi-view
    let front_off = Point3::new(0.0f32, 0.0, 0.0);
    let side_off = Point3::new(-1.5f32, 0.0, 0.0);
    let top_off = Point3::new(1.5f32, 0.0, 0.0);

    draw_ground_grid_at(window, &front_off, s);
    draw_ground_grid_at(window, &side_off, s);
    draw_ground_grid_at(window, &top_off, s);

    draw_skeleton_at(window, skel, positions, joint_to_track, to_display_front, &front_off, s);
    draw_skeleton_at(window, skel, positions, joint_to_track, to_display_side, &side_off, s);
    draw_skeleton_at(window, skel, positions, joint_to_track, to_display_top, &top_off, s);

    let label_color = Point3::new(0.5f32, 0.5, 0.5);
    window.draw_text("FRONT", &Point2::new(340.0, 580.0), 30.0, font, &label_color);
    window.draw_text("SIDE", &Point2::new(100.0, 580.0), 30.0, font, &label_color);
    window.draw_text("TOP", &Point2::new(580.0, 580.0), 30.0, font, &label_color);
}


// ---------------------------------------------------------------------------
// Live mode — BLE stdin reader + calibration state machine
// ---------------------------------------------------------------------------

/// Accumulate a quaternion sample into a running sum with sign-flip correction.
/// q and -q represent the same rotation; this keeps samples in the same hemisphere.
fn accum_quat(accum: &mut [f64; 4], count: &mut usize, q: &UnitQuaternion<f64>) {
    let comps = [q.w, q.i, q.j, q.k];
    let sign = if *count > 0
        && (accum[0] * comps[0]
            + accum[1] * comps[1]
            + accum[2] * comps[2]
            + accum[3] * comps[3])
            < 0.0
    {
        -1.0
    } else {
        1.0
    };
    for i in 0..4 {
        accum[i] += sign * comps[i];
    }
    *count += 1;
}

/// Finalize accumulated quaternion average → UnitQuaternion.
fn finalize_quat(accum: &[f64; 4]) -> UnitQuaternion<f64> {
    let avg = Quaternion::new(accum[0], accum[1], accum[2], accum[3]);
    if avg.norm_squared() > 1e-6 {
        UnitQuaternion::from_quaternion(avg)
    } else {
        UnitQuaternion::identity()
    }
}

/// Compute the expected skeleton rotation at `joint_idx` for T-pose.
///
/// The skeleton rest pose is N-pose (arms at sides). For arm joints whose
/// outgoing bone points downward, T-pose requires rotating that bone to
/// horizontal (lateral). This is derived from the skeleton geometry:
///   - Outgoing bone direction (to first child) is the rest direction
///   - T-pose direction is lateral (±Y based on which side of the body)
///   - Only applies if incoming bone (from parent) is NOT also downward
///     (i.e. this is the root of the arm chain, not a child like elbow)
fn compute_tpose_rotation(skel: &Skeleton, joint_idx: usize) -> UnitQuaternion<f64> {
    let joint = &skel.joints[joint_idx];

    // Find first child to get outgoing bone direction
    let child_idx = skel.joints.iter().position(|j| j.parent == Some(joint_idx));
    let child_idx = match child_idx {
        Some(i) => i,
        None => return UnitQuaternion::identity(),
    };

    let child_offset = skel.joints[child_idx].position - joint.position;

    // Check if incoming bone from parent is also downward (child joint, not chain root)
    let incoming_is_down = if let Some(pi) = joint.parent {
        let incoming = joint.position - skel.joints[pi].position;
        incoming.z.abs() > incoming.y.abs() && incoming.z < -0.05
    } else {
        false
    };

    // Apply correction only for chain-root arm joints:
    //   outgoing bone points down, incoming does NOT, joint is laterally offset and above pelvis
    if child_offset.z < -0.1 && !incoming_is_down && joint.position.y.abs() > 0.1 && joint.position.z > 0.2 {
        let rest_dir = child_offset.normalize();
        let lateral_sign = if joint.position.y < 0.0 { -1.0 } else { 1.0 };
        let tpose_dir = Vector3::new(0.0, lateral_sign, 0.0);
        UnitQuaternion::rotation_between(&rest_dir, &tpose_dir)
            .unwrap_or(UnitQuaternion::identity())
    } else {
        UnitQuaternion::identity()
    }
}

fn run_live(
    skel: &Skeleton,
    joint_indices: &[usize],
    remap: &[(f64, usize); 3],
    remap_str: &str,
) {
    let n_channels = joint_indices.len();

    // Build joint_to_track mapping
    let mut joint_to_track: Vec<Option<usize>> = vec![None; skel.joints.len()];
    for (ti, &ji) in joint_indices.iter().enumerate() {
        joint_to_track[ji] = Some(ti);
    }

    // Spawn stdin reader thread
    let (tx, rx) = mpsc::channel::<(u32, UnitQuaternion<f64>)>();
    let remap_owned = *remap;
    thread::spawn(move || {
        let reader = BufReader::new(io::stdin());
        let mut first = true;
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            let parts: Vec<&str> = line.split(',').collect();
            if first {
                eprintln!("stdin: first line ({} cols): {}", parts.len(), &line[..line.len().min(80)]);
                first = false;
            }
            // 5-col: ts,w,x,y,z (single sensor → channel 0)
            // 6-col: ch,ts,w,x,y,z (multi sensor)
            let (channel, q0, q1, q2, q3) = if parts.len() >= 6 {
                let vals: Option<Vec<f64>> =
                    parts.iter().map(|s| s.trim().parse::<f64>().ok()).collect();
                let vals = match vals { Some(v) => v, None => continue };
                (vals[0] as u32, vals[2], vals[3], vals[4], vals[5])
            } else if parts.len() >= 5 {
                let vals: Option<Vec<f64>> =
                    parts.iter().map(|s| s.trim().parse::<f64>().ok()).collect();
                let vals = match vals { Some(v) => v, None => continue };
                (0u32, vals[1], vals[2], vals[3], vals[4])
            } else {
                continue;
            };
            let raw = Quaternion::new(q0, q1, q2, q3);
            if raw.norm_squared() < 1e-6 {
                continue;
            }
            let q = apply_remap(UnitQuaternion::from_quaternion(raw), &remap_owned);
            if tx.send((channel, q)).is_err() {
                break;
            }
        }
    });

    // --- Calibration state machine ---
    //
    // WaitTpose  ──SPACE──▶  CollectTpose  ──auto──▶  WaitNpose
    //                                                      │
    //    ┌──────────SPACE──────────────────────────────────┘
    //    ▼
    // CollectNpose  ──auto──▶  Live  ──SPACE──▶  WaitTpose
    //
    enum State {
        WaitTpose,
        CollectTpose,
        WaitNpose,
        CollectNpose,
        Live,
    }
    let mut state = State::WaitTpose;

    let min_cal_samples: usize = 50;

    // Per-channel accumulators (reused for both T-pose and N-pose)
    let mut cal_accum: Vec<[f64; 4]> = vec![[0.0; 4]; n_channels];
    let mut cal_count: Vec<usize> = vec![0; n_channels];

    // Calibration results
    let mut ref_tpose: Vec<UnitQuaternion<f64>> =
        vec![UnitQuaternion::identity(); n_channels];
    let mut ref_npose: Vec<UnitQuaternion<f64>> =
        vec![UnitQuaternion::identity(); n_channels];

    // Per-channel axis correction (computed after both calibrations)
    // Maps sensor rotation axes to skeleton rotation axes via conjugation:
    //   joint_rot = correction * (q * ref_npose.inverse()) * correction.inverse()
    let mut corrections: Vec<UnitQuaternion<f64>> =
        vec![UnitQuaternion::identity(); n_channels];

    // Expected skeleton T-pose rotation per channel (from skeleton geometry)
    let skel_tpose: Vec<UnitQuaternion<f64>> = joint_indices
        .iter()
        .map(|&ji| compute_tpose_rotation(skel, ji))
        .collect();
    for (ti, &ji) in joint_indices.iter().enumerate() {
        let q = &skel_tpose[ti];
        if let Some((axis, angle)) = q.axis_angle() {
            eprintln!(
                "  {} T-pose rotation: {:.1}° around [{:.2}, {:.2}, {:.2}]",
                skel.joints[ji].name,
                angle.to_degrees(),
                axis.x, axis.y, axis.z,
            );
        } else {
            eprintln!("  {} T-pose rotation: identity", skel.joints[ji].name);
        }
    }

    // Latest calibrated quaternion per channel
    let mut latest: Vec<UnitQuaternion<f64>> =
        vec![UnitQuaternion::identity(); n_channels];

    // Reset accumulators helper
    let reset_accum = |accum: &mut Vec<[f64; 4]>, count: &mut Vec<usize>| {
        for a in accum.iter_mut() {
            *a = [0.0; 4];
        }
        for c in count.iter_mut() {
            *c = 0;
        }
    };

    // kiss3d setup
    let mut window = Window::new("Body Kinematics — Live");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.08, 0.08, 0.12);

    let eye = Point3::new(0.0f32, 0.3, 2.0);
    let at = Point3::new(0.0, 0.0, 0.0);
    let mut camera = ArcBall::new(eye, at);
    let font = Font::default();

    while window.render_with_camera(&mut camera) {
        // --- Input ---
        let mut space_pressed = false;
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::Space, Action::Press, _) => space_pressed = true,
                WindowEvent::Key(Key::Equals, Action::Press, _)
                | WindowEvent::Key(Key::Add, Action::Press, _) => {
                    camera.set_dist(camera.dist() * 0.8);
                }
                WindowEvent::Key(Key::Minus, Action::Press, _)
                | WindowEvent::Key(Key::Subtract, Action::Press, _) => {
                    camera.set_dist(camera.dist() * 1.25);
                }
                _ => {}
            }
        }

        // --- Drain all pending readings ---
        let mut new_readings: Vec<(u32, UnitQuaternion<f64>)> = Vec::new();
        while let Ok(msg) = rx.try_recv() {
            new_readings.push(msg);
        }

        // --- State machine ---
        match state {
            State::WaitTpose => {
                // Discard incoming data, wait for SPACE
                if space_pressed {
                    reset_accum(&mut cal_accum, &mut cal_count);
                    state = State::CollectTpose;
                    eprintln!("Collecting T-pose samples...");
                }
            }
            State::CollectTpose => {
                for &(ch, ref q) in &new_readings {
                    let ch = ch as usize;
                    if ch < n_channels {
                        accum_quat(&mut cal_accum[ch], &mut cal_count[ch], q);
                    }
                }
                // Auto-advance when all channels have enough
                if (0..n_channels).all(|ch| cal_count[ch] >= min_cal_samples) {
                    for ch in 0..n_channels {
                        ref_tpose[ch] = finalize_quat(&cal_accum[ch]);
                    }
                    reset_accum(&mut cal_accum, &mut cal_count);
                    state = State::WaitNpose;
                    eprintln!("T-pose captured — now stand in N-pose");
                }
            }
            State::WaitNpose => {
                // Discard incoming data, wait for SPACE
                if space_pressed {
                    reset_accum(&mut cal_accum, &mut cal_count);
                    state = State::CollectNpose;
                    eprintln!("Collecting N-pose samples...");
                }
            }
            State::CollectNpose => {
                for &(ch, ref q) in &new_readings {
                    let ch = ch as usize;
                    if ch < n_channels {
                        accum_quat(&mut cal_accum[ch], &mut cal_count[ch], q);
                    }
                }
                // Auto-advance when all channels have enough
                if (0..n_channels).all(|ch| cal_count[ch] >= min_cal_samples) {
                    for ch in 0..n_channels {
                        ref_npose[ch] = finalize_quat(&cal_accum[ch]);
                    }

                    // Compute per-channel axis correction from T/N calibration.
                    //
                    // sensor_delta = sensor T-pose relative to N-pose
                    // skel_tpose   = expected skeleton rotation in T-pose
                    //
                    // We need `correction` such that:
                    //   correction * sensor_delta * correction.inverse() ≈ skel_tpose
                    //
                    // Both represent the same physical rotation (~90° for shoulders)
                    // but in different coordinate frames. Aligning their rotation axes
                    // via `rotation_between` gives us the frame correction.
                    for ch in 0..n_channels {
                        let sensor_delta = ref_tpose[ch] * ref_npose[ch].inverse();
                        let skel_t = &skel_tpose[ch];

                        corrections[ch] = match (sensor_delta.axis_angle(), skel_t.axis_angle()) {
                            (Some((s_axis, _)), Some((k_axis, _))) => {
                                let c = UnitQuaternion::rotation_between(
                                    s_axis.as_ref(),
                                    k_axis.as_ref(),
                                ).unwrap_or(UnitQuaternion::identity());
                                eprintln!(
                                    "  ch{} ({}) axis correction: {:.1}°",
                                    ch, skel.joints[joint_indices[ch]].name,
                                    c.angle().to_degrees(),
                                );
                                c
                            }
                            _ => {
                                eprintln!(
                                    "  ch{} ({}) no axis correction (identity T-pose)",
                                    ch, skel.joints[joint_indices[ch]].name,
                                );
                                UnitQuaternion::identity()
                            }
                        };
                    }

                    state = State::Live;
                    eprintln!("Calibration complete — live mode active");
                }
            }
            State::Live => {
                // Calibrate each reading:
                //   1. Relative to N-pose (skeleton rest): q_rel = q * ref_npose.inverse()
                //   2. Axis correction via conjugation: correction * q_rel * correction.inverse()
                for &(ch, q) in &new_readings {
                    let ch = ch as usize;
                    if ch < n_channels {
                        let q_rel = q * ref_npose[ch].inverse();
                        latest[ch] = corrections[ch] * q_rel * corrections[ch].inverse();
                    }
                }

                if space_pressed {
                    state = State::WaitTpose;
                    latest = vec![UnitQuaternion::identity(); n_channels];
                    eprintln!("Recalibrating — get into T-pose, press SPACE");
                }
            }
        }

        // --- Build per-joint rotations ---
        let n = skel.joints.len();
        let mut rotations = vec![UnitQuaternion::identity(); n];
        if let State::Live = state {
            for (ti, &ji) in joint_indices.iter().enumerate() {
                rotations[ji] = latest[ti];
            }
        }

        // --- Forward kinematics + draw ---
        let positions = skel.forward_kinematics(&rotations);
        draw_all_views(&mut window, skel, &positions, &joint_to_track, &font);

        // --- HUD ---
        let white = Point3::new(1.0f32, 1.0, 1.0);
        let green = Point3::new(0.2f32, 1.0, 0.3);
        let hud = match state {
            State::WaitTpose => {
                "1/2  Stand in T-POSE (arms out)\n     Press SPACE to capture".to_string()
            }
            State::CollectTpose => {
                let counts: Vec<String> = (0..n_channels)
                    .map(|ch| format!("{}/{}", cal_count[ch], min_cal_samples))
                    .collect();
                format!("1/2  Capturing T-pose...  {}", counts.join("  "))
            }
            State::WaitNpose => {
                "2/2  Stand in N-POSE (arms at sides)\n     Press SPACE to capture".to_string()
            }
            State::CollectNpose => {
                let counts: Vec<String> = (0..n_channels)
                    .map(|ch| format!("{}/{}", cal_count[ch], min_cal_samples))
                    .collect();
                format!("2/2  Capturing N-pose...  {}", counts.join("  "))
            }
            State::Live => {
                let mut lines = vec!["LIVE".to_string()];
                for (ti, &ji) in joint_indices.iter().enumerate() {
                    let name = &skel.joints[ji].name;
                    let q = &latest[ti];
                    lines.push(format!(
                        "  [{}] {}: w={:.3} x={:.3} y={:.3} z={:.3}",
                        ti, name, q.w, q.i, q.j, q.k
                    ));
                }
                lines.push(format!("Remap: {}  SPACE: recalibrate", remap_str));
                lines.join("\n")
            }
        };
        let hud_color = match state {
            State::Live => green,
            _ => white,
        };
        window.draw_text(&hud, &Point2::new(10.0, 20.0), 36.0, &font, &hud_color);
    }
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

    // Filter out --remap (+ value) and --live for positional arg parsing
    let live_mode = args.iter().any(|a| a == "--live");
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
            if a == "--live" {
                continue;
            }
            v.push(a.as_str());
        }
        v
    };

    // --- Live mode: branch to run_live() and exit ---
    if live_mode {
        if positional.is_empty() {
            eprintln!("Error: --live requires at least one joint name.");
            eprintln!("Usage: ble-scan --multi | body_viz --live ShoulderRight ElbowRight");
            std::process::exit(1);
        }
        let joint_indices: Vec<usize> = positional
            .iter()
            .map(|name| {
                skel.find_joint(name)
                    .unwrap_or_else(|| panic!("Unknown joint '{name}'"))
            })
            .collect();
        for (ch, &ji) in joint_indices.iter().enumerate() {
            println!("Channel {} → '{}'", ch, skel.joints[ji].name);
        }
        println!("Live mode — reading 6-col CSV from stdin (remap: {})", remap_str);
        println!("Stand in T-pose, press SPACE to calibrate");
        run_live(&skel, &joint_indices, &remap, remap_str);
        return;
    }

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

        // --- Draw ---
        draw_all_views(&mut window, &skel, &positions, &joint_to_track, &font);

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
