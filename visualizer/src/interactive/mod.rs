pub mod components;

pub fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Load the GLB file
    //let glb_handle: Handle<Scene> = asset_server.load("models/Box_2.0.glb#Scene0");
    //let glb_handle: Handle<Scene> = asset_server.load("models/Jet_2.0.glb#Scene0");
    let glb_handle: Handle<Scene> = asset_server.load("models/jet.glb#Scene0");
    // Note: "#Scene0" tells Bevy to load the first scene in the GLB

    //Spawn the GLB scene
    commands.spawn(SceneBundle {
        scene: glb_handle,
        //transform: Transform::from_translation(Vec3::ZERO).with_scale(Vec3::splat(0.5)), // optional scaling
        ..Default::default()
    });

    commands.spawn((
        TextBundle::from_section(
            "Angles",
            TextStyle {
                font: asset_server.load("fonts/FiraSans-Bold.ttf"),
                font_size: 20.0,
                color: Color::BLACK,
            },
        )
        .with_style(Style {
            position_type: PositionType::Absolute,
            left: Val::Px(10.0),
            top: Val::Px(10.0),
            ..default()
        }),
        AngleText,
    ));

    //commands.spawn((
    //PbrBundle {
    //mesh: meshes.add(Mesh::from(shape::Plane {
    //size: 2.0,
    //subdivisions: 1,
    //})),
    //material: materials.add(Color::rgb(0.3, 0.5, 0.7).into()),
    //..default()
    //},
    //PlaneMarker,
    //));

    //add_axes(&mut commands, &mut meshes, &mut materials);

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

// Draw X, Y, Z axes
fn add_axes(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let axis_length = 1.0;

    // X axis - red
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Box::new(axis_length, 0.02, 0.02))),
        material: materials.add(Color::RED.into()),
        transform: Transform::from_xyz(axis_length / 2.0, 0.0, 0.0),
        ..default()
    });

    // Y axis - green
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Box::new(0.02, axis_length, 0.02))),
        material: materials.add(Color::GREEN.into()),
        transform: Transform::from_xyz(0.0, axis_length / 2.0, 0.0),
        ..default()
    });

    // Z axis - blue
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Box::new(0.02, 0.02, axis_length))),
        material: materials.add(Color::BLUE.into()),
        transform: Transform::from_xyz(0.0, 0.0, axis_length / 2.0),
        ..default()
    });
}
