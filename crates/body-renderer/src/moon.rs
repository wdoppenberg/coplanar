use crate::dem::DEMManager;

use bevy::core_pipeline::core_3d::{Camera3dDepthLoadOp, ScreenSpaceTransmissionQuality};
use bevy::pbr::CascadeShadowConfigBuilder;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::Face;
use gdal::raster::StatisticsAll;
use gdal::Dataset;
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct MoonPlugin;

impl Plugin for MoonPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup, setup_ui)).add_systems(
            Update,
            (
                flyby_camera,
                update_moon_lod,
                update_lod_text,
                update_sunlight,
            ),
        );
    }
}

#[derive(Component)]
pub struct Moon;

#[derive(Component)]
struct MoonLOD {
    current_lod: AtomicUsize,
    max_lod: usize,
}

#[derive(Component)]
struct LODText;

fn setup_ui(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands
        .spawn(NodeBundle {
            style: Style {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                top: Val::Px(10.0),
                ..default()
            },
            ..default()
        })
        .with_children(|parent| {
            parent.spawn((
                TextBundle::from_section(
                    "LOD: 0",
                    TextStyle {
                        font: asset_server.load("fonts/FiraSans-Bold.ttf"),
                        font_size: 24.0,
                        color: Color::WHITE,
                    },
                ),
                LODText,
            ));
        });
}

fn update_lod_text(moon_query: Query<&MoonLOD>, mut text_query: Query<&mut Text, With<LODText>>) {
    if let Ok(moon_lod) = moon_query.get_single() {
        if let Ok(mut text) = text_query.get_single_mut() {
            let current_lod = moon_lod.current_lod.load(Ordering::Relaxed);
            text.sections[0].value = format!("LOD: {}", current_lod);
        }
    }
}

#[derive(Component)]
struct FlybyCamera {
    time: f32,
    height: f32,
    orbit_radius: f32,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut dem: ResMut<DEMManager>,
    mut images: ResMut<Assets<Image>>,
) {
    let max_lod = 9; // Increased for higher detail
    let initial_lod = 0;

    // Load high-res color texture
    let color_texture = load_color_texture("/Users/wdoppenberg/Downloads/lroc_color_poles.tif");
    let texture_handle = images.add(color_texture);

    // Create initial mesh
    info!("Creating mesh...");
    let moon_mesh = create_moon_mesh(initial_lod, &mut dem);
    let mesh_handle = meshes.add(moon_mesh);

    // Moon material with proper culling
    commands.spawn((
        PbrBundle {
            mesh: mesh_handle,
            material: materials.add(StandardMaterial {
                base_color_texture: Some(texture_handle),
                base_color: Color::WHITE,
                metallic: 0.0,
                perceptual_roughness: 0.98,  // Increased roughness
                reflectance: 0.12,           // Increased to match lunar albedo
                cull_mode: Some(Face::Back), // Enable back-face culling
                double_sided: false,         // Disable double-sided rendering
                parallax_mapping_method: ParallaxMappingMethod::Relief { max_steps: 32 },
                parallax_depth_scale: 0.1,
                ..default()
            }),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },
        Moon,
        MoonLOD {
            current_lod: AtomicUsize::new(initial_lod),
            max_lod,
        },
    ));

    // Main sunlight
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::WHITE,
            illuminance: 1.27e5,
            shadows_enabled: true,
            shadow_depth_bias: 0.00001, // Reduced from 0.001 for better precision
            shadow_normal_bias: 0.00001, // Reduced from 0.01 for better precision
        },
        // Add these new shadow quality settings
        cascade_shadow_config: CascadeShadowConfigBuilder {
            first_cascade_far_bound: 10.0,
            maximum_distance: 10.0,
            minimum_distance: 0.1,
            num_cascades: 6,
            ..default()
        }
        .into(),
        transform: Transform::from_xyz(4.0, 2.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Subtle ambient light
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 1.0,
            shadows_enabled: false,
            ..default()
        },
        transform: Transform::from_xyz(-4.0, 4.0, -4.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Camera with black background
    commands.spawn((
        Camera3dBundle {
            camera: Camera {
                clear_color: ClearColorConfig::Custom(Color::BLACK),
                hdr: true,
                ..default()
            },
            transform: Transform::from_xyz(0.0, 0.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
            camera_3d: Camera3d {
                depth_load_op: Camera3dDepthLoadOp::Clear(0.),
                screen_space_specular_transmission_quality: ScreenSpaceTransmissionQuality::Ultra,
                ..default()
            },
            ..default()
        },
        FlybyCamera {
            time: 0.0,
            height: 2.,        // Closer to surface
            orbit_radius: 1.3, // Closer orbit
        },
    ));
}

fn flyby_camera(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut FlybyCamera, &mut Transform)>,
) {
    for (mut camera, mut transform) in query.iter_mut() {
        let mult = {
            if keys.pressed(KeyCode::Space) {
                0.5
            } else {
                0.05
            }
        };
        camera.time += time.delta_seconds() * mult; // Slower movement

        // Complex orbital path that varies in height and distance
        let angle = camera.time;
        let vertical_offset = (angle * 0.5).sin() * camera.height;
        let radius = camera.orbit_radius + (angle * 0.3).cos() * 0.1;

        let x = angle.cos() * radius;
        let y = vertical_offset;
        let z = angle.sin() * radius;

        transform.translation = Vec3::new(x, y, z);

        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

fn update_moon_lod(
    mut meshes: ResMut<Assets<Mesh>>,
    mut dem: ResMut<DEMManager>,
    query: Query<(Entity, &Handle<Mesh>, &MoonLOD)>,
) {
    for (_entity, mesh_handle, moon_lod) in query.iter() {
        let current_lod = moon_lod.current_lod.load(Ordering::Relaxed);

        // Check if we need to increase LOD and if enough tiles are loaded
        if current_lod < moon_lod.max_lod {
            let required_tiles = 4u32.pow(current_lod as u32) as usize;
            let loaded_tiles = dem.get_loaded_tile_count();

            if loaded_tiles >= required_tiles {
                // Create new mesh at higher LOD
                let new_lod = current_lod + 1;
                let new_mesh = create_moon_mesh(new_lod, &mut dem);

                // Update the mesh
                if let Some(mesh) = meshes.get_mut(mesh_handle) {
                    *mesh = new_mesh;
                    moon_lod.current_lod.store(new_lod, Ordering::Relaxed);
                }

                info!("Increased moon LOD to {}", new_lod);
            }
        }
    }
}

fn update_sunlight(time: Res<Time>, mut query: Query<&mut Transform, With<DirectionalLight>>) {
    for mut transform in query.iter_mut() {
        let angle = time.elapsed_seconds() * 0.1; // Slow rotation
        let x = 4.0 * angle.cos();
        let z = 4.0 * angle.sin();
        *transform = Transform::from_xyz(x, 2.0, z).looking_at(Vec3::ZERO, Vec3::Y);
    }
}

fn create_moon_mesh(lod: usize, dem: &mut DEMManager) -> Mesh {
    let longitude_segments = 128 * (2u32.pow(lod as u32));
    let latitude_segments = longitude_segments / 2;

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut indices = Vec::new();

    // Generate vertices
    for lat in 0..=latitude_segments {
        let lat_frac = lat as f32 / latitude_segments as f32;
        // Convert to degrees, going from 90° (North) to -90° (South)
        let lat_deg = 90.0 - lat_frac * 180.0;

        for lon in 0..=longitude_segments {
            let lon_frac = lon as f32 / longitude_segments as f32;
            // Convert to degrees, going from -180° to 180°
            let lon_deg = lon_frac * 360.0 - 180.0;

            let elevation = dem.get_elevation(lat_deg, lon_deg);
            let elevation_scaled = elevation / 1_737_400.0;

            let lat_rad = lat_deg.to_radians();
            let lon_rad = lon_deg.to_radians();
            let cos_lat = lat_rad.cos();

            let radius = 1.0 + elevation_scaled;
            let position = Vec3::new(
                radius * cos_lat * lon_rad.cos(),
                radius * lat_rad.sin(),
                radius * cos_lat * lon_rad.sin(),
            );

            let normal = position.normalize();

            positions.push([position.x, position.y, position.z]);
            normals.push([normal.x, normal.y, normal.z]);

            let u = (lon_deg + 180.0) / 360.0; // Convert [-180, 180] to [0, 1]
            let v = (90.0 - lat_deg) / 180.0; // Convert [90, -90] to [0, 1]

            uvs.push([u, v]);
        }
    }

    // Generate indices with correct winding order
    for lat in 0..latitude_segments {
        for lon in 0..longitude_segments {
            let first = lat * (longitude_segments + 1) + lon;
            let second = first + longitude_segments + 1;

            // First triangle (counter-clockwise winding)
            indices.extend_from_slice(&[first as u32, (first + 1) as u32, second as u32]);

            // Second triangle (counter-clockwise winding)
            indices.extend_from_slice(&[(first + 1) as u32, (second + 1) as u32, second as u32]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );

    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));

    mesh
}

fn load_color_texture(path: &str) -> Image {
    let dataset = Dataset::open(path).expect("Failed to open color texture");
    let (width, height) = dataset.raster_size();

    let max_dim = 16384;
    let scale = (max_dim as f64 / width.max(height) as f64).min(1.0);
    let new_width = (width as f64 * scale) as usize;
    let new_height = (height as f64 * scale) as usize;

    info!(
        "Resizing texture from {}x{} to {}x{}",
        width, height, new_width, new_height
    );

    let mut data = Vec::new();

    // Read RGB bands as f32 to preserve precision
    for band_index in 1..=3 {
        let band = dataset.rasterband(band_index).unwrap();

        // Get band statistics
        let StatisticsAll { min, max, .. } = band.get_statistics(true, true).unwrap().unwrap();

        // Create buffer for f32 data
        let mut band_data = vec![0f32; new_width * new_height];

        // Read raw data
        band.read_into_slice(
            (0, 0),
            (width, height),
            (new_width, new_height),
            &mut band_data,
            None,
        )
        .expect("Failed to read color band");

        // Normalize the data with better precision
        for value in band_data.iter_mut() {
            // Normalize considering the full range
            *value = ((*value - min as f32) / (max - min) as f32).clamp(0.0, 1.0);

            // Apply subtle gamma correction to preserve detail
            *value = value.powf(1.0 / 1.2);

            // Convert to u8 with dithering to reduce banding
            let byte_value = (*value * 255.0) as u8;
            data.push(byte_value);
        }
    }

    // Convert to RGBA with proper color space handling
    let mut rgba_data = Vec::with_capacity((new_width * new_height * 4) as usize);
    for i in 0..(new_width * new_height) {
        rgba_data.push(data[i]); // R
        rgba_data.push(data[i + (new_width * new_height)]); // G
        rgba_data.push(data[i + 2 * (new_width * new_height)]); // B
        rgba_data.push(255); // A
    }

    Image::new(
        bevy::render::render_resource::Extent3d {
            width: new_width as u32,
            height: new_height as u32,
            depth_or_array_layers: 1,
        },
        bevy::render::render_resource::TextureDimension::D2,
        rgba_data,
        // Use sRGB format for better color reproduction
        bevy::render::render_resource::TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}
