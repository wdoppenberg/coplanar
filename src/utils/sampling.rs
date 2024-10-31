use std::fmt::Debug;

use crate::SpatialEllipse;
use nalgebra as na;

/// Helper function to create points along an ellipse for visualization
pub fn sample_ellipse_points<F: na::RealField + Copy + Debug>(
    spatial_ellipse: &SpatialEllipse<F>,
    num_points: usize,
) -> Vec<na::Point3<F>> {
    let mut points = Vec::with_capacity(num_points);

    // Get local to world transform
    let transform = spatial_ellipse.local_to_world();

    // Sample points in local frame
    for i in 0..num_points {
        let t = F::from_f64(i as f64 * 2.0 / num_points as f64).unwrap() * F::pi();
        let cos_t = t.cos();
        let sin_t = t.sin();

        // Extract semi-major and semi-minor from ellipse matrix
        // Note: This is a simplification, should properly extract axes
        let (a, b) = (F::one(), F::from_f64(0.5).unwrap()); // Example values

        // Point in local frame
        let local_point = na::Point3::new(a * cos_t, b * sin_t, F::zero());

        // Transform to world frame
        let world_point = transform.transform_point(&local_point);
        points.push(world_point);
    }

    points
}
