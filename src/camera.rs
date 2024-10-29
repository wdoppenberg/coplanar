use crate::EllipseMatrix;
use anyhow::{anyhow, Result};
use nalgebra as na;
use num_traits::Float;
use std::fmt::Debug;

/// Camera intrinsic parameters
#[derive(Debug, Clone)]
pub struct CameraIntrinsics<F: Float> {
    /// Focal length in x direction (pixels)
    pub fx: F,
    /// Focal length in y direction (pixels)
    pub fy: F,
    /// Principal point x coordinate (pixels)
    pub cx: F,
    /// Principal point y coordinate (pixels)
    pub cy: F,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
}

/// Camera pose in MCMF frame
#[derive(Debug, Clone)]
pub struct CameraPose<F: Float + na::RealField> {
    /// Rotation matrix from MCMF to camera frame
    pub rotation: na::Matrix3<F>,
    /// Translation vector from MCMF origin to camera center in MCMF frame
    pub translation: na::Vector3<F>,
}

impl<F: Float + na::RealField> CameraPose<F> {
    /// Creates camera pose from position and attitude
    pub fn from_position_attitude(
        position: na::Vector3<F>,
        attitude: na::UnitQuaternion<F>,
    ) -> Self {
        Self {
            rotation: attitude.to_rotation_matrix().into_inner(),
            translation: position,
        }
    }

    /// Computes the projection matrix P = K[R|t]
    pub fn projection_matrix(&self, intrinsics: &CameraIntrinsics<F>) -> na::Matrix3x4<F> {
        let zero = F::zero();
        let one = F::one();

        let k = na::Matrix3::new(
            intrinsics.fx,
            zero,
            intrinsics.cx,
            zero,
            intrinsics.fy,
            intrinsics.cy,
            zero,
            zero,
            one,
        );

        let rt = na::Matrix::from_columns(&[
            self.rotation.column(0),
            self.rotation.column(1),
            self.rotation.column(2),
            self.translation.as_view(),
        ]);

        k * rt
    }
}

/// Projects a 3D point in MCMF frame onto the camera image plane
pub fn project_point<F>(
    point: &na::Point3<F>,
    camera: &CameraPose<F>,
    intrinsics: &CameraIntrinsics<F>,
) -> Result<na::Point2<F>>
where
    F: Float + na::RealField,
{
    // Transform point to camera frame
    let p_cam = camera.rotation * (point - camera.translation);

    // Check if point is in front of camera
    if p_cam.z <= F::zero() {
        return Err(anyhow!("Point is behind camera"));
    }

    // Perspective projection
    let x = intrinsics.fx * p_cam.x / p_cam.z + intrinsics.cx;
    let y = intrinsics.fy * p_cam.y / p_cam.z + intrinsics.cy;

    // Check if point is within image bounds
    if x < F::zero()
        || x >= F::from(intrinsics.width).unwrap()
        || y < F::zero()
        || y >= F::from(intrinsics.height).unwrap()
    {
        return Err(anyhow!("Point projects outside image bounds"));
    }

    Ok(na::Point2::new(x, y))
}

/// Projects an ellipse from MCMF frame onto the camera image plane
pub fn project_ellipse<F>(
    ellipse: &na::Unit<EllipseMatrix<F>>,
    camera: &CameraPose<F>,
    intrinsics: &CameraIntrinsics<F>,
) -> Result<EllipseMatrix<F>>
where
    F: Float + na::RealField + Debug,
{
    let zero = F::zero();
    let one = F::one();

    // Extract the quadric matrix Q from the ellipse matrix
    let a = ellipse.fixed_view::<2, 2>(0, 0);
    let b = ellipse.fixed_view::<2, 1>(0, 2);
    let c = ellipse[(2, 2)];

    // Construct the 4x4 quadric matrix
    let mut q = na::Matrix4::zeros();
    q.fixed_view_mut::<2, 2>(0, 0).copy_from(&a);
    q.fixed_view_mut::<2, 1>(0, 2).copy_from(&b);
    q.fixed_view_mut::<1, 2>(2, 0).copy_from(&b.transpose());
    q[(2, 2)] = c;

    // Get the projection matrix
    let p = camera.projection_matrix(intrinsics);
    let p_homo = na::Matrix4::new(
        p[(0, 0)],
        p[(0, 1)],
        p[(0, 2)],
        p[(0, 3)],
        p[(1, 0)],
        p[(1, 1)],
        p[(1, 2)],
        p[(1, 3)],
        p[(2, 0)],
        p[(2, 1)],
        p[(2, 2)],
        p[(2, 3)],
        zero,
        zero,
        zero,
        one,
    );

    // Project the quadric
    let q_img = p_homo * q * p_homo.transpose();

    // Extract the projected ellipse matrix
    let mut e_img = na::Matrix3::zeros();
    e_img
        .fixed_view_mut::<2, 2>(0, 0)
        .copy_from(&q_img.fixed_view::<2, 2>(0, 0));
    e_img
        .fixed_view_mut::<2, 1>(0, 2)
        .copy_from(&q_img.fixed_view::<2, 1>(0, 2));
    e_img
        .fixed_view_mut::<1, 2>(2, 0)
        .copy_from(&q_img.fixed_view::<1, 2>(2, 0));
    e_img[(2, 2)] = q_img[(2, 2)];

    Ok(EllipseMatrix::new(e_img))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::EllipseParametric;
    use approx::assert_relative_eq;
    use nalgebra::Normed;

    #[test]
    fn test_simple_projection() -> Result<()> {
        // Create a camera looking at origin from +Z
        let camera = CameraPose::from_position_attitude(
            na::Vector3::new(0.0f64, 0.0, 10.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        );

        // Simple intrinsics
        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        // Create a circular crater at origin
        let crater = EllipseParametric::new(1.0, 1.0, 0.0, 0.0, 0.0);
        let crater_matrix = EllipseMatrix::from(crater);
        let unit_crater = na::Unit::new_normalize(crater_matrix);

        // Project the crater
        let projected = project_ellipse(&unit_crater, &camera, &intrinsics)?;

        // The projection of a circle under perspective should be an ellipse
        // Test some basic properties
        let det = projected.determinant();
        assert!(det != 0.0, "Projected ellipse should be non-degenerate");

        Ok(())
    }

    #[test]
    fn test_off_center_projection() -> Result<()> {
        let pi: f64 = std::f64::consts::PI;

        // Create a camera looking at a 45-degree angle
        let camera = CameraPose::from_position_attitude(
            na::Vector3::new(5.0, 5.0, 5.0),
            na::UnitQuaternion::from_euler_angles(-pi / 4.0, pi / 4.0, 0.0),
        );

        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        // Create an elliptical crater
        let crater = EllipseParametric::new(2.0, 1.0, pi / 6.0, 1.0, 1.0);
        let crater_matrix = EllipseMatrix::from(crater);
        let unit_crater = na::Unit::new_normalize(crater_matrix);

        let projected = project_ellipse(&unit_crater, &camera, &intrinsics)?;

        // Verify the projected ellipse exists
        assert!(projected.norm() > 0.0);

        // Test symmetry of projected ellipse matrix
        let proj_mat = projected;
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(proj_mat[(i, j)], proj_mat[(j, i)], epsilon = 1e-10);
            }
        }

        Ok(())
    }
}
