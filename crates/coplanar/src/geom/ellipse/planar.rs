use core::fmt::Debug;
use core::ops::{Deref, DerefMut};
use nalgebra as na;
use thiserror::Error;

use crate::geom::ellipse::repr::EllipseRepr;
use crate::math::conic;

#[derive(Debug, Error)]
pub enum PlanarEllipseError {
    #[error("No center coordinates found: {0}")]
    NoCenterCoordinates(&'static str),
}

/// Parametric representation of an ellipse in a 2D plane.
#[derive(Debug, Clone)]
pub struct Parametric<F: na::RealField + Copy> {
    /// Semi-major axis
    pub a: F,
    /// Semi-minor axis
    pub b: F,
    /// Rotation in radians
    pub theta: F,
    /// Center x-coordinate
    pub x: F,
    /// Center y-coordinate
    pub y: F,
}

/// An ellipse in a 2D plane described as a 3x3 matrix of the conic section described by the
/// quadratic equation $A_Q$ [1].
///
/// [1] https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
#[derive(Debug, Clone)]
pub struct Quadratic<F: na::RealField>(na::Matrix3<F>);

impl<F: na::RealField + Copy> Quadratic<F> {
    pub fn semi_axes(&self) -> (F, F) {
        let det_33 = self.fixed_view::<2, 2>(0, 0).determinant();
        let det_q = self.determinant();
        let eigvals = self.fixed_view::<2, 2>(0, 0).symmetric_eigenvalues();
        let scale = -det_q / det_33;

        let a1 = (F::one() / (eigvals.x / scale)).sqrt();
        let a2 = (F::one() / (eigvals.y / scale)).sqrt();

        (a2, a1)
    }
}

#[derive(Debug, Clone)]
pub struct PlanarEllipse<R: EllipseRepr>(R);

impl<F: na::RealField + Copy> PlanarEllipse<Parametric<F>> {
    pub fn from_parameters(a: F, b: F, theta: F, x: F, y: F) -> Self {
        let (a, b) = if b > a { (b, a) } else { (a, b) };
        Self(Parametric { a, b, theta, x, y })
    }

    pub fn try_into_quadratic(self) -> Result<PlanarEllipse<Quadratic<F>>, crate::Error> {
        let Parametric::<F> { a, b, theta, x, y } = self.0;
        let mat = conic::compute_ellipse_matrix(a, b, theta, x, y)?;
        PlanarEllipse::<Quadratic<F>>::try_from_matrix(mat)
    }
}

impl<F: na::RealField + Copy> PlanarEllipse<Quadratic<F>> {
    pub fn try_from_matrix(matrix: na::Matrix3<F>) -> Result<Self, crate::Error> {
        conic::check_ellipse_conditions(&matrix)?;
        Ok(Self(Quadratic(matrix)))
    }

    pub fn try_into_parametric(self) -> Result<PlanarEllipse<Parametric<F>>, PlanarEllipseError> {
        let Parametric { a, b, theta, x, y } = self.0.get_parametric()?;

        Ok(PlanarEllipse::<Parametric<F>>::from_parameters(
            a, b, theta, x, y,
        ))
    }
}

impl<F: na::RealField + Debug + Copy> na::Normed for PlanarEllipse<Quadratic<F>> {
    type Norm = F;
    fn norm(&self) -> Self::Norm {
        self.0.0.norm()
    }

    fn norm_squared(&self) -> Self::Norm {
        self.0.0.norm_squared()
    }

    fn scale_mut(&mut self, n: Self::Norm) {
        self.0.0.scale_mut(n);
    }

    fn unscale_mut(&mut self, n: Self::Norm) {
        self.0.0.unscale_mut(n);
    }
}

impl<F: na::RealField + Copy> Deref for Quadratic<F> {
    type Target = na::Matrix3<F>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<F: na::RealField + Copy> DerefMut for Quadratic<F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<R: EllipseRepr> AsRef<R> for PlanarEllipse<R> {
    fn as_ref(&self) -> &R {
        &self.0
    }
}

impl<R: EllipseRepr> Deref for PlanarEllipse<R> {
    type Target = R;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<R: EllipseRepr> DerefMut for PlanarEllipse<R> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<F: na::RealField + Copy> AsRef<na::Matrix3<F>> for PlanarEllipse<Quadratic<F>> {
    fn as_ref(&self) -> &na::Matrix3<F> {
        &self.0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    mod rotation_tests {
        use super::*;
        use crate::geom::ellipse::repr::EllipseRepr;
        use crate::math::conic;
        use approx::assert_relative_eq;
        use core::f64::consts::PI;

        #[test]
        fn test_rotation_angles() {
            let test_angles = [0.0, PI / 6.0, PI / 4.0, PI / 3.0, PI / 2.0];

            for &theta in &test_angles {
                // Create an ellipse with the given rotation
                let matrix = conic::compute_matrix(2.0, 1.0, theta, 0.0, 0.0);
                let ellipse = PlanarEllipse::<Quadratic<f64>>::try_from_matrix(matrix).unwrap();

                // Extract the rotation and compare
                let extracted = ellipse.rotation();

                assert_relative_eq!(extracted, theta, epsilon = 1e-10);
            }
        }

        #[test]
        fn test_horizontal_ellipse() {
            let matrix = conic::compute_ellipse_matrix(2.0, 1.0, 0.0, 0.0, 0.0).unwrap();
            let ellipse = PlanarEllipse::<Quadratic<f64>>::try_from_matrix(matrix).unwrap();

            assert_relative_eq!(ellipse.rotation(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn test_vertical_ellipse() {
            let matrix = conic::compute_matrix(2.0, 1.0, PI / 2.0, 0.0, 0.0);
            let ellipse = PlanarEllipse::<Quadratic<f64>>::try_from_matrix(matrix).unwrap();

            assert_relative_eq!(ellipse.rotation(), PI / 2.0, epsilon = 1e-10);
        }
    }
    mod conversion_tests {
        use super::*;
        use crate::geom::ellipse::repr::EllipseRepr;
        use approx::assert_relative_eq;
        use core::f64::consts::PI;

        fn normalize_rotation(rot: f64) -> f64 {
            // Handle rotation ambiguity - normalize to [0, PI)
            let mut rot = rot % PI;
            if rot < 0.0 {
                rot += PI;
            }
            rot
        }

        fn assert_ellipse_params_eq<R1: EllipseRepr<F = f64>, R2: EllipseRepr<F = f64>>(
            e1: &R1,
            e2: &R2,
            epsilon: f64,
        ) {
            let (x1, y1) = e1.center().unwrap();
            let (x2, y2) = e2.center().unwrap();

            // For non-circular ellipses, we need to handle the fact that swapping axes
            // and rotating by PI/2 gives an equivalent ellipse
            let (maj1, min1, rot1) = if e1.semi_major() >= e1.semi_minor() {
                (
                    e1.semi_major(),
                    e1.semi_minor(),
                    normalize_rotation(e1.rotation()),
                )
            } else {
                (
                    e1.semi_minor(),
                    e1.semi_major(),
                    normalize_rotation(e1.rotation() + PI / 2.0),
                )
            };

            let (maj2, min2, rot2) = if e2.semi_major() >= e2.semi_minor() {
                (
                    e2.semi_major(),
                    e2.semi_minor(),
                    normalize_rotation(e2.rotation()),
                )
            } else {
                (
                    e2.semi_minor(),
                    e2.semi_major(),
                    normalize_rotation(e2.rotation() + PI / 2.0),
                )
            };

            assert_relative_eq!(maj1, maj2, epsilon = epsilon);
            assert_relative_eq!(min1, min2, epsilon = epsilon);
            assert_relative_eq!(rot1, rot2, epsilon = epsilon);
            assert_relative_eq!(x1, x2, epsilon = epsilon);
            assert_relative_eq!(y1, y2, epsilon = epsilon);
        }

        #[test]
        fn test_circle_conversion() {
            let parametric = PlanarEllipse::from_parameters(2.0, 2.0, 0.0, 0.0, 0.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();

            let parametric_back = quadratic.try_into_parametric();

            assert_ellipse_params_eq(
                parametric.as_ref(),
                parametric_back.unwrap().as_ref(),
                1e-10,
            );
        }

        #[test]
        fn test_ellipse_conversion() {
            let parametric = PlanarEllipse::from_parameters(3.0, 1.0, 0.0, 0.0, 0.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();

            let parametric_back = quadratic.try_into_parametric();

            assert_ellipse_params_eq(
                parametric.as_ref(),
                parametric_back.unwrap().as_ref(),
                1e-10,
            );
        }

        #[test]
        fn test_rotated_ellipse_conversion() {
            let parametric = PlanarEllipse::from_parameters(3.0, 1.0, PI / 4.0, 0.0, 0.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();

            let parametric_back = quadratic.try_into_parametric();

            assert_ellipse_params_eq(
                parametric.as_ref(),
                parametric_back.unwrap().as_ref(),
                1e-10,
            );
        }

        #[test]
        fn test_translated_ellipse_conversion() {
            let parametric = PlanarEllipse::from_parameters(3.0, 1.0, 0.0, 2.0, 3.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();

            let parametric_back = quadratic.try_into_parametric();

            assert_ellipse_params_eq(
                parametric.as_ref(),
                parametric_back.unwrap().as_ref(),
                1e-10,
            );
        }

        #[test]
        fn test_rotated_translated_ellipse_conversion() {
            let parametric = PlanarEllipse::from_parameters(3.0, 1.0, PI / 6.0, -2.0, 1.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            let parametric_back = quadratic.try_into_parametric().unwrap();

            assert_ellipse_params_eq(parametric.as_ref(), parametric_back.as_ref(), 1e-10);
        }

        #[test]
        fn test_almost_circle_conversion() {
            let parametric = PlanarEllipse::from_parameters(2.0, 1.99999, 0.0, 0.0, 0.0);
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            let parametric_back = quadratic.try_into_parametric().unwrap();

            assert_ellipse_params_eq(parametric.as_ref(), parametric_back.as_ref(), 1e-10);
        }
    }
}
