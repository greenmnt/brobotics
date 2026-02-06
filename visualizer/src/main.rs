use bevy::gltf::Gltf;
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Component)]
struct OrbitCamera {
    radius: f32,
    yaw: f32,
    pitch: f32,
    target: Vec3, // where the camera is looking at
}

fn orbit_camera_keyboard(
    keys: Res<Input<KeyCode>>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    for (mut orbit, mut transform) in query.iter_mut() {
        let speed = 0.05;
        let zoom_speed = 0.5;

        // rotate
        if keys.pressed(KeyCode::Left) {
            orbit.yaw -= speed;
        }
        if keys.pressed(KeyCode::Right) {
            orbit.yaw += speed;
        }
        if keys.pressed(KeyCode::Up) {
            orbit.pitch = (orbit.pitch + speed).clamp(-1.5, 1.5);
        }
        if keys.pressed(KeyCode::Down) {
            orbit.pitch = (orbit.pitch - speed).clamp(-1.5, 1.5);
        }

        // zoom
        if keys.pressed(KeyCode::Q) {
            orbit.radius = (orbit.radius - zoom_speed).max(0.1);
        }
        if keys.pressed(KeyCode::E) {
            orbit.radius += zoom_speed;
        }

        // convert orbit parameters to Transform
        let x = orbit.radius * orbit.pitch.cos() * orbit.yaw.sin();
        let y = orbit.radius * orbit.pitch.sin();
        let z = orbit.radius * orbit.pitch.cos() * orbit.yaw.cos();

        transform.translation = orbit.target + Vec3::new(x, y, z);
        transform.look_at(orbit.target, Vec3::Y);
    }
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Load the GLB file
    //let glb_handle: Handle<Scene> = asset_server.load("models/Box_2.0.glb#Scene0");
    //let glb_handle: Handle<Scene> = asset_server.load("models/Jet_2.0.glb#Scene0");
    let glb_handle: Handle<Scene> = asset_server.load("models/jet.glb#Scene0");
    // Note: "#Scene0" tells Bevy to load the first scene in the GLB

    // Spawn the GLB scene
    commands.spawn(SceneBundle {
        scene: glb_handle,
        //transform: Transform::from_translation(Vec3::ZERO).with_scale(Vec3::splat(0.5)), // optional scaling
        ..Default::default()
    });

    // Camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        OrbitCamera {
            radius: 10.0,
            yaw: 0.0,
            pitch: 0.0,
            target: Vec3::ZERO,
        },
    ));

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            range: 100.0,
            shadows_enabled: true,
            ..Default::default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..Default::default()
    });
}

fn orbit_camera_system(
    time: Res<Time>,
    mouse_input: Res<Input<MouseButton>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
) {
    for (mut transform, mut orbit) in query.iter_mut() {
        if mouse_input.pressed(MouseButton::Left) {
            for motion in mouse_motion.iter() {
                orbit.yaw -= motion.delta.x * 0.005;
                orbit.pitch -= motion.delta.y * 0.005;
                orbit.pitch = orbit.pitch.clamp(-1.5, 1.5);
            }
        }

        // Update camera position
        let x = orbit.radius * orbit.yaw.cos() * orbit.pitch.cos();
        let y = orbit.radius * orbit.pitch.sin();
        let z = orbit.radius * orbit.yaw.sin() * orbit.pitch.cos();

        transform.translation = Vec3::new(x, y, z);
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

// Shared IMU data
#[derive(Default, Clone, Copy)]
struct GyroData {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Clone, Resource)]
struct SharedGyro(Arc<Mutex<GyroData>>);

#[derive(Default)]
struct GyroIntegrator {
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    rotation_x_error: f32,
    rotation_y_error: f32,
    rotation_z_error: f32,
    scale: f32, // equivalent to *0.01 in C++ example
}

impl GyroIntegrator {
    fn new() -> Self {
        Self {
            rotation_x_error: 0.05,
            rotation_y_error: 0.02,
            rotation_z_error: 0.01,
            scale: 0.01,
            ..Default::default()
        }
    }

    fn update(&mut self, gx: f32, gy: f32, gz: f32) {
        if gx.abs() > self.rotation_x_error {
            self.rotation_x += gx * self.scale;
        }
        if gy.abs() > self.rotation_y_error {
            self.rotation_y += gy * self.scale;
        }
        if gz.abs() > self.rotation_z_error {
            self.rotation_z += gz * self.scale;
        }
    }

    fn get_rotations(&self) -> GyroData {
        GyroData {
            x: self.rotation_x,
            y: self.rotation_y,
            z: self.rotation_z,
        }
    }
}

fn main() {
    // Shared state between thread and Bevy app
    let gyro_data = SharedGyro(Arc::new(Mutex::new(GyroData::default())));
    let gyro_clone = gyro_data.clone();

    // Spawn a thread to read IMU data
    thread::spawn(move || {
        let file = File::open("/tmp/imu_out").expect("Failed to open IMU pipe");
        let reader = BufReader::new(file);

        let mut integrator = GyroIntegrator::new();

        for line in reader.lines() {
            match line {
                Ok(line) => {
                    // Strip timestamp if present
                    let line = if let Some(idx) = line.find(':') {
                        &line[(idx + 1)..]
                    } else {
                        &line
                    };
                    let line = line.trim();

                    let values: Vec<&str> = line.split(',').collect();
                    if values.len() >= 6 {
                        let gx: f32 = values[3].parse().unwrap_or(0.0);
                        let gy: f32 = values[4].parse().unwrap_or(0.0);
                        let gz: f32 = values[5].parse().unwrap_or(0.0);

                        // Apply noise filtering & integration
                        integrator.update(gx, gy, gz);

                        // Update shared gyro resource
                        let mut data = gyro_clone.0.lock().unwrap();
                        *data = integrator.get_rotations();
                    }
                }
                Err(e) => println!("Read error: {:?}", e),
            }

            // Small delay to prevent busy-loop
            thread::sleep(Duration::from_millis(5));
        }
    });
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::WHITE))
        .insert_resource(gyro_data)
        .add_systems(Startup, setup)
        .add_systems(Update, orbit_camera_system)
        .add_systems(Update, orbit_camera_keyboard)
        .add_systems(Update, update_plane_rotation)
        .run();
}
// System to rotate the plane based on IMU gyro data
fn update_plane_rotation(gyro: Res<SharedGyro>, mut query: Query<&mut Transform, With<Name>>) {
    let data = gyro.0.lock().unwrap();
    let gx = data.x.to_radians() / 100.0; // scale down for visualization
    let gy = data.y.to_radians() / 100.0;
    let gz = data.z.to_radians() / 100.0;

    for mut transform in query.iter_mut() {
        transform.rotation = Quat::from_euler(
            EulerRot::XYZ,
            gy, // map gy to x rotation
            gz, // map gz to y rotation
            gx, // map gx to z rotation
        );
    }
}
