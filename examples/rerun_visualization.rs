use anyhow::Result;
use ellipse_matcher::{spatial_ellipse::SpatialEllipse, utils::sample_ellipse_points};
use nalgebra as na;
use num_traits::Float;
use rerun as rr;
use std::f64::consts::PI;

/// Convert nalgebra vector to rerun Vector3D
fn vec_to_vector3d<F: Float + na::RealField>(vec: &na::Vector3<F>) -> rr::datatypes::Vec3D {
    rr::datatypes::Vec3D::new(
        vec.x.to_f32().unwrap(),
        vec.y.to_f32().unwrap(),
        vec.z.to_f32().unwrap(),
    )
}

/// Convert nalgebra point to rerun vec3
fn point_to_vec3<F: Float + na::RealField>(point: &na::Point3<F>) -> rr::datatypes::Vec3D {
    rr::datatypes::Vec3D::new(
        point.x.to_f32().unwrap(),
        point.y.to_f32().unwrap(),
        point.z.to_f32().unwrap(),
    )
}

fn create_coordinate_frame_arrows<F: Float + na::RealField>(
    transform: &na::Matrix4<F>,
    scale: f32,
) -> Vec<rr::datatypes::Vec3D> {
    let scale = F::from(scale).unwrap();

    // Create the axis vectors directly
    vec![
        vec_to_vector3d(&transform.fixed_view::<3, 1>(0, 0).scale(scale)),
        vec_to_vector3d(&transform.fixed_view::<3, 1>(0, 1).scale(scale)),
        vec_to_vector3d(&transform.fixed_view::<3, 1>(0, 2).scale(scale)),
    ]
}

fn main() -> Result<()> {
    // Initialize rerun
    let rec = rr::RecordingStreamBuilder::new("spatial_ellipse_transform").spawn()?;

    // Create an ellipse
    let original_ellipse = SpatialEllipse::new(
        na::Point3::new(0.0, 0.0, 0.0),
        na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
        2.0, // semi-major
        1.0, // semi-minor
        0.0, // rotation
    );

    // Sample points for visualization
    let original_points = sample_ellipse_points(&original_ellipse, 50);

    // Log original ellipse points with label
    rec.log(
        "original/points",
        &rr::Points3D::new(
            original_points
                .iter()
                .map(point_to_vec3)
                .collect::<Vec<_>>(),
        )
        .with_colors([rr::Color::from_rgb(0, 255, 0)]) // Green
        .with_labels(["Original Ellipse"].repeat(original_points.len())),
    )?;

    // Log original coordinate frame
    let original_transform = original_ellipse.local_to_world();
    let origin = original_ellipse.center;

    rec.log(
        "original/frame",
        &rr::Arrows3D::from_vectors(create_coordinate_frame_arrows(&original_transform, 1.0))
            .with_origins(vec![point_to_vec3(&origin); 3]) // Set origin points
            .with_colors([
                rr::Color::from_rgb(255, 0, 0), // X axis - Red
                rr::Color::from_rgb(0, 255, 0), // Y axis - Green
                rr::Color::from_rgb(0, 0, 255), // Z axis - Blue
            ])
            .with_labels(["X axis", "Y axis", "Z axis"]),
    )?;

    // Create transformations with descriptive names
    let transforms = vec![
        (
            "Translation (+3 in X)",
            na::Translation3::new(3.0, 0.0, 0.0).to_homogeneous(),
        ),
        (
            "Rotation (45° around Y)",
            na::UnitQuaternion::from_euler_angles(0.0, PI / 4.0, 0.0).to_homogeneous(),
        ),
        (
            "Combined (translate + rotate)",
            na::Translation3::new(2.0, 2.0, 2.0).to_homogeneous()
                * na::UnitQuaternion::from_euler_angles(PI / 6.0, PI / 4.0, 0.0).to_homogeneous(),
        ),
    ];

    // Apply and visualize each transformation
    for (i, (transform_name, transform)) in transforms.iter().enumerate() {
        // Transform ellipse
        let transformed = original_ellipse.transform(transform)?;
        let transformed_points = sample_ellipse_points(&transformed, 50);

        // Create a unique path for this transformation
        let base_path = format!("transform_{}/{}", i, transform_name);

        // Log transformed points
        rec.log(
            format!("{}/points", base_path),
            &rr::Points3D::new(
                transformed_points
                    .iter()
                    .map(point_to_vec3)
                    .collect::<Vec<_>>(),
            )
            .with_colors([rr::Color::from_rgb(255, 0, 0)]) // Red
            .with_labels([*transform_name].repeat(transformed_points.len())),
        )?;

        // Log coordinate frame
        let transform_matrix = transformed.local_to_world();
        let origin = transformed.center;

        rec.log(
            format!("{}/frame", base_path),
            &rr::Arrows3D::from_vectors(create_coordinate_frame_arrows(&transform_matrix, 1.0))
                .with_origins(vec![point_to_vec3(&origin); 3])
                .with_colors([
                    rr::Color::from_rgb(255, 0, 0),
                    rr::Color::from_rgb(0, 255, 0),
                    rr::Color::from_rgb(0, 0, 255),
                ])
                .with_labels([
                    format!("{} - X axis", transform_name),
                    format!("{} - Y axis", transform_name),
                    format!("{} - Z axis", transform_name),
                ]),
        )?;

        // Log normal vector
        rec.log(
            format!("{}/normal", base_path),
            &rr::Arrows3D::from_vectors([vec_to_vector3d(&transformed.normal.into_inner())])
                .with_origins([point_to_vec3(&transformed.center)])
                .with_colors([rr::Color::from_rgb(255, 165, 0)]) // Orange
                .with_labels([format!("{} - Normal", transform_name)]),
        )?;
    }

    // Configure space
    rec.log_static("space", &rr::ViewCoordinates::RIGHT_HAND_Y_UP)?;

    Ok(())
}