use approx::assert_relative_eq;
use coplanar::{compute_invariants, SpatialEllipse};
use nalgebra as na;

fn main() -> Result<(), coplanar::Error> {
    let spatial_ellipses = [
        (0.0, 0.0, 0.0),   // Center
        (1.0, 0.0, 0.0),   // Right
        (0.5, 0.866, 0.0), // Top
    ]
    .map(|(x, y, z)| {
        SpatialEllipse::new(
            na::Point3::new(x, y, z),
            na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
            1.0, // Using circles (equal semi-axes) for stability
            1.0,
            0.0, // No rotation initially
        )
        .unwrap()
    });

    // Compute features
    let original_inv = compute_invariants(
        &spatial_ellipses[0].ellipse,
        &spatial_ellipses[1].ellipse,
        &spatial_ellipses[2].ellipse,
    )?;

    println!("Features for the initial triad: {:?}", original_inv);

    // Apply global translation
    let translation = na::Translation3::new(10.0, -5.0, 3.0).to_homogeneous();
    let translated_ellipses: Vec<_> = spatial_ellipses
        .iter()
        .map(|e| e.transform(&translation).unwrap())
        .collect();

    // Compute features for translated ellipse triad
    let translated_inv = compute_invariants(
        &translated_ellipses[0].ellipse,
        &translated_ellipses[1].ellipse,
        &translated_ellipses[2].ellipse,
    )?;

    println!("Features for the translated triad: {:?}", translated_inv);

    let dist = original_inv.distance(&translated_inv);

    println!("Euclidean distance between features: {:.2e}", dist);

    assert_relative_eq!(dist, 0.0, epsilon = 1e-6);
    Ok(())
}
