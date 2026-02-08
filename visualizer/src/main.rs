#![allow(warnings)]
use bevy::gltf::Gltf;
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

mod data_handler;
mod interactive;
mod sensors;

use data_handler::*;
use interactive::{
    orbit_camera_keyboard, orbit_camera_system, setup, update_angle_text, update_plane_orientation,
};

/// Shared IMU resource for Bevy
//#[derive(Clone, Resource)]
//struct SharedImu(Arc<Mutex<ImuData>>);
//impl SharedImu {
//pub fn new() -> Self {
//Self(Arc::new(Mutex::new(ImuData::default())))
//}
//}
#[derive(Clone, Resource)]
struct SharedOrientation(Arc<Mutex<Orientation>>);

impl SharedOrientation {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Orientation::default())))
    }
}

fn startup_thread_system(imu_data: Res<SharedOrientation>) {
    let imu_clone = imu_data.0.clone(); // clone the Arc
    thread::spawn(move || {
        let file = File::open("/tmp/imu_out").expect("Failed to open IMU pipe");
        let mut reader = BufReader::new(file);

        let accel_sens = sensors::mpu_6050::accel::ACCEL_TABLE[0].lsb_per_g;
        let gyro_sens = sensors::mpu_6050::gyro::FS_SEL_0;

        let num_samples = 500;
        let now = Instant::now();
        println!("collecting sample");
        info!("collecting sample");
        let bias_sample = collect_sample(&mut reader, num_samples);
        let sample_time = Instant::now() - now;
        info!(timer = now.elapsed().as_secs_f64(), "sample collected");

        info!("calculating bias");
        let now = Instant::now();
        let gyro_bias = Some(calculate_gyro_bias_from_sample(bias_sample));
        info!(timer = now.elapsed().as_secs_f64(), "bias calculated");
        let accel_bias = None;

        let mut filter = ImuFilter::new(TiltCalculationMethod::Complementary {
            alpha: 0.98,
            acceleration_method: TiltFromAccelerationMethod::TiltCompensatedRoll,
        });
        let mut prev_time: f32 = 0.0;

        let mut last_hash: u64 = 0;

        info!("reading lines");
        for line in reader.lines() {
            if let Err(e) = line {
                eprintln!("Read error: {:?}", e);
                continue;
            }
            let line = line.unwrap();
            let h = hash_line(&line);
            if h == last_hash {
                eprintln!("Hashes are equal...skipping");
                continue;
            }
            last_hash = h;
            if let Some(raw_imu) = parse_imu_line(&line) {
                info!(line = ?line, raw_imu = ?raw_imu);
                let ts = raw_imu.timestamp();
                if prev_time > ts {
                    eprintln!("prev_time >= current_timestamp {:?} {:?}", prev_time, ts);
                    continue;
                }

                let dt = ts - prev_time;
                prev_time = ts;

                let scaled =
                    clean_raw_readings(raw_imu, &gyro_bias, &accel_bias, gyro_sens, accel_sens);

                filter.update(scaled, dt);

                let mut shared = imu_clone.lock().unwrap();
                *shared = *filter.get_orientation();
                thread::sleep(Duration::from_millis(5));
            } else {
                println!("Error parsing: {:?}", line);
            }
        }
    });
}

fn main() {
    // Shared state between thread and Bevy app
    let imu_data = SharedOrientation(Arc::new(Mutex::new(Orientation::default())));
    let imu_clone = imu_data.clone();
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(imu_data)
        .add_systems(Startup, setup)
        .add_systems(Startup, startup_thread_system)
        .add_systems(Update, orbit_camera_system)
        .add_systems(Update, orbit_camera_keyboard)
        .add_systems(Update, update_angle_text)
        .add_systems(Update, update_plane_orientation)
        .run();
}

fn hash_line(line: &str) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    let mut hasher = DefaultHasher::new();
    line.hash(&mut hasher);
    hasher.finish()
}
