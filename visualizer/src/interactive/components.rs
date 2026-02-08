use crate::{SharedOrientation, SharedQuaternionData};
use bevy::gltf::Gltf;
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;

#[derive(Component)]
pub struct OrbitCamera {
    pub radius: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub target: Vec3, // where the camera is looking at
}

pub fn orbit_camera_keyboard(
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

#[derive(Component)]
pub struct AngleText;

pub fn update_angle_text(
    imu: Res<SharedOrientation>,
    mut query: Query<&mut Text, With<AngleText>>,
) {
    let data = imu.0.lock().unwrap();

    let pitch = data.pitch.to_degrees();
    let roll = data.roll.to_degrees();
    let yaw = data.yaw.to_degrees();

    //println!("{:?} {:?} {:?}", pitch, roll, yaw);

    for mut text in &mut query {
        text.sections[0].value = format!(
            "Pitch:  {:.1}°\nRoll: {:.1}°\nYaw:   {:.1}°",
            pitch, roll, yaw
        );
    }
}

pub fn update_plane_orientation(
    imu_data: Res<SharedOrientation>,
    mut query: Query<&mut Transform, With<Name>>,
) {
    let data = imu_data.0.lock().unwrap();
    for mut transform in query.iter_mut() {
        transform.rotation = Quat::from_euler(EulerRot::XYZ, data.pitch, data.roll, data.yaw);
    }
}

pub fn update_plane_orientation_q(
    imu_data: Res<SharedQuaternionData>,
    mut query: Query<&mut Transform, With<Name>>,
) {
    let data = imu_data.0.lock().unwrap();
    tracing::info!(data = ?data);
    for mut transform in query.iter_mut() {
        let sensor_quat = Quat::from_xyzw(data.q.x, data.q.y, data.q.z, data.q.w);
        let fix_rotation = Quat::from_rotation_x(std::f32::consts::PI);
        transform.rotation = fix_rotation * sensor_quat;
    }
}

//pub fn update_plane_orientation(
//imu_data: Res<SharedOrientation>,
//mut query: Query<&mut Transform, With<Name>>,
//) {
//let data = imu_data.0.lock().unwrap();
//for mut transform in query.iter_mut() {
//transform.rotation = Quat::from_euler(EulerRot::XYZ, data.pitch, data.roll, data.yaw);
//}
//}

pub fn orbit_camera_system(
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

//// System to rotate the plane based on IMU gyro data
//pub fn update_plane_rotation(gyro: Res<SharedGyro>, mut query: Query<&mut Transform, With<Name>>) {
//let data = gyro.0.lock().unwrap();
//let gx = data.x.to_radians() / 100.0; // scale down for visualization
//let gy = data.y.to_radians() / 100.0;
//let gz = data.z.to_radians() / 100.0;

//for mut transform in query.iter_mut() {
//transform.rotation = Quat::from_euler(
//EulerRot::XYZ,
//gy, // map gy to x rotation
//gz, // map gz to y rotation
//gx, // map gx to z rotation
//);
//}
//}
