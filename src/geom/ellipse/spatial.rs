use anyhow::{anyhow, Result};
use nalgebra as na;
use std::fmt::Debug;

use crate::geom::ellipse::planar::Quadratic;
use crate::{
    vision::camera::{project_ellipse, CameraIntrinsics, CameraPose},
    PlanarEllipse,
};

/// Represents an ellipse embedded in 3D space
#[derive(Debug, Clone)]
pub struct SpatialEllipse<F: na::RealField + Debug + Copy> {
    /// Center position in 3D space
    pub center: na::Point3<F>,
    /// Normal vector to the ellipse plane
    pub normal: na::Unit<na::Vector3<F>>,
    /// Elliptical shape in the local plane
    pub ellipse: na::Unit<PlanarEllipse<Quadratic<F>>>,
}

impl<F: na::RealField + Copy + Debug> SpatialEllipse<F> {
    /// Create a new ellipse from its center, normal, and ellipse parameters
    pub fn new(
        center: na::Point3<F>,
        normal: na::Unit<na::Vector3<F>>,
        semi_major: F,
        semi_minor: F,
        rotation: F,
    ) -> Result<Self, crate::Error> {
        let ellipse = PlanarEllipse::from_parameters(
            semi_major,
            semi_minor,
            rotation,
            F::zero(), // local coordinates, so no offset
            F::zero(),
        );

        let ellipse_quad = ellipse.try_into_quadratic()?;

        Ok(Self {
            center,
            normal,
            ellipse: na::Unit::new_normalize(ellipse_quad),
        })
    }

    /// Create a spatial ellipse from an existing ellipse matrix and a pose in 3D
    pub fn from_ellipse_and_pose(
        ellipse: na::Unit<PlanarEllipse<Quadratic<F>>>,
        center: na::Point3<F>,
        normal: na::Unit<na::Vector3<F>>,
    ) -> Self {
        Self {
            center,
            normal,
            ellipse,
        }
    }

    /// Get the local-to-world transformation matrix
    pub fn local_to_world(&self) -> na::Matrix4<F> {
        // Create local coordinate frame
        let z = self.normal.into_inner();

        // Choose arbitrary x direction perpendicular to z
        // We use the minimal component strategy to avoid numerical issues
        let x = if z.x.abs() < z.y.abs() && z.x.abs() < z.z.abs() {
            na::Unit::new_normalize(na::Vector3::new(F::one(), F::zero(), F::zero()))
        } else if z.y.abs() < z.x.abs() && z.y.abs() < z.z.abs() {
            na::Unit::new_normalize(na::Vector3::new(F::zero(), F::one(), F::zero()))
        } else {
            na::Unit::new_normalize(na::Vector3::new(F::zero(), F::zero(), F::one()))
        };

        // Complete right-handed frame
        let y = na::Unit::new_normalize(z.cross(&x.into_inner()));
        let x = y.cross(&z);

        // Build transformation matrix
        let mut transform = na::Matrix4::identity();
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&na::Matrix3::from_columns(&[x, y.into_inner(), z]));
        transform
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&self.center.coords);

        transform
    }

    /// Transform the ellipse by a 4x4 transformation matrix
    pub fn transform(&self, transform: &na::Matrix4<F>) -> Result<Self> {
        // Transform center point
        let homogeneous_center =
            transform * na::Vector4::new(self.center.x, self.center.y, self.center.z, F::one());

        // Extract new center
        let w = homogeneous_center[3];
        if w == F::zero() {
            return Err(anyhow!("Invalid transformation: point at infinity"));
        }

        let new_center = na::Point3::new(
            homogeneous_center[0] / w,
            homogeneous_center[1] / w,
            homogeneous_center[2] / w,
        );

        // Transform normal (using inverse transpose for correct normal transformation)
        let normal_transform = transform
            .fixed_view::<3, 3>(0, 0)
            .try_inverse()
            .ok_or(anyhow!("Transformation is not invertible"))?
            .transpose();

        let new_normal = na::Unit::new_normalize(normal_transform * self.normal.into_inner());

        // Create the transformed ellipse
        Ok(Self {
            center: new_center,
            normal: new_normal,
            ellipse: self.ellipse.clone(), // Shape is preserved in local coordinates
        })
    }

    /// Project the ellipse onto a camera image plane
    pub fn project(
        &self,
        camera: &CameraPose<F>,
        intrinsics: &CameraIntrinsics<F>,
    ) -> Result<PlanarEllipse<Quadratic<F>>> {
        // First transform to camera coordinates
        let view = camera.view_matrix();
        let transformed = self.transform(&view)?;

        // Check if ellipse is in front of camera
        if transformed.center.z <= F::zero() {
            return Err(anyhow!("Ellipse is behind camera"));
        }

        // Project to image plane using perspective projection
        let projected = project_ellipse(&transformed.ellipse, camera, intrinsics)?;

        Ok(projected)
    }
}

// Helper impl for camera pose
impl<F: na::RealField + Copy> CameraPose<F> {
    /// Get the view matrix (world to camera transform)
    pub fn view_matrix(&self) -> na::Matrix4<F> {
        let mut view = na::Matrix4::identity();
        view.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.rotation);
        view.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(-self.rotation * self.translation));
        view
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_spatial_ellipse_creation() {
        let center = na::Point3::new(1.0, 2.0, 3.0);
        let normal = na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0));

        let ellipse = SpatialEllipse::new(
            center, normal, 2.0, // semi-major
            1.0, // semi-minor
            0.0, // rotation
        )
        .unwrap();

        assert_relative_eq!(
            ellipse.center.coords.norm(),
            14.0f64.sqrt(),
            epsilon = 1e-10
        );
        assert_relative_eq!(ellipse.normal.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_transformation() -> Result<()> {
        let ellipse = SpatialEllipse::new(
            na::Point3::new(0.0, 0.0, 0.0),
            na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
            1.0,
            0.5,
            0.0,
        )?;

        // Create a translation and rotation
        let translation = na::Translation3::new(1.0, 2.0, 3.0);
        let rotation = na::UnitQuaternion::from_euler_angles(PI / 4.0, 0.0, 0.0);
        let transform = translation.to_homogeneous() * rotation.to_homogeneous();

        let transformed = ellipse.transform(&transform)?;

        // Check translation
        assert_relative_eq!(
            transformed.center.coords.norm(),
            14.0f64.sqrt(),
            epsilon = 1e-10
        );

        // Check normal transformation
        let expected_normal = rotation * na::Vector3::new(0.0, 0.0, 1.0);
        assert_relative_eq!(
            transformed.normal.into_inner(),
            expected_normal,
            epsilon = 1e-10
        );

        Ok(())
    }

    #[test]
    fn test_projection() -> Result<()> {
        let ellipse = SpatialEllipse::new(
            na::Point3::new(0.0, 0.0, 5.0),
            na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
            1.0,
            0.5,
            0.0,
        )?;

        let camera = CameraPose::from_position_attitude(
            na::Vector3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        );

        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        let projected = ellipse.project(&camera, &intrinsics)?;

        // Basic checks on projected ellipse
        assert!(projected.norm() > 0.);

        Ok(())
    }

    #[test]
    fn test_behind_camera() {
        let ellipse = SpatialEllipse::new(
            na::Point3::new(0.0, 0.0, -5.0), // Behind camera
            na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
            1.0,
            0.5,
            0.0,
        )
        .unwrap();

        let camera = CameraPose::from_position_attitude(
            na::Vector3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        );

        let intrinsics = CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };

        assert!(ellipse.project(&camera, &intrinsics).is_err());
    }
}
