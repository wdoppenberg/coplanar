use crate::math::conic;
use nalgebra as na;
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum PlanarEllipseError {
    #[error("No center coordinates found: {0}")]
    NoCenterCoordinates(&'static str),
}

/// Defines the behavior of an ellipse representation
pub trait EllipseRepr: Debug {
    /// Underlying data type, must be real e.g. [f32] or [f64] or any other type that implements
    /// [na::RealField].
    type F: na::RealField + Copy;

    /// Returns the semi-major axis length
    fn semi_major(&self) -> Self::F;

    /// Returns the semi-minor axis length
    fn semi_minor(&self) -> Self::F;

    /// Returns the rotation angle in radians
    fn rotation(&self) -> Self::F;

    /// Returns the center coordinates
    fn center(&self) -> Result<(Self::F, Self::F), PlanarEllipseError>;

    /// Returns the parametric representation
    fn get_parametric(&self) -> Result<Parametric<Self::F>, PlanarEllipseError> {
        let (x, y) = self.center()?;
        Ok(Parametric::<Self::F> {
            a: self.semi_major(),
            b: self.semi_minor(),
            theta: self.rotation(),
            x,
            y,
        })
    }

    /// Converts to matrix representation
    fn to_matrix(&self) -> na::Matrix3<Self::F>;
}

/// Parametric representation of an ellipse in a 2D plane.
#[derive(Debug, Clone)]
pub struct Parametric<F: na::RealField + Copy> {
    /// Semi-major axis
    a: F,
    /// Semi-minor axis
    b: F,
    /// Rotation in radians
    theta: F,
    /// Center x-coordinate
    x: F,
    /// Center y-coordinate
    y: F,
}

/// An ellipse in a 2D plane described as a 3x3 matrix of the conic section described by the
/// quadratic equation $A_Q$ [1].
///
/// [1] https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
#[derive(Debug, Clone)]
pub struct Quadratic<F: na::RealField>(na::Matrix3<F>);

impl<F: na::RealField + Copy> Quadratic<F> {
    fn extract_semi_axes(&self) -> (F, F) {
        let det_33 = self.fixed_view::<2, 2>(0, 0).determinant();
        let det_q = self.determinant();
        let eigvals = self.fixed_view::<2, 2>(0, 0).symmetric_eigenvalues();
        let [a1, a2] = eigvals
            .as_ref()
            .map(|l| l / (-det_q / det_33))
            .map(|l| (F::one() / l).sqrt());

        (a2, a1)
    }
}

impl<F: na::RealField + Copy> EllipseRepr for Parametric<F> {
    type F = F;
    fn semi_major(&self) -> F {
        self.a
    }
    fn semi_minor(&self) -> F {
        self.b
    }
    fn rotation(&self) -> F {
        self.theta
    }
    fn center(&self) -> Result<(F, F), PlanarEllipseError> {
        Ok((self.x, self.y))
    }
    fn to_matrix(&self) -> na::Matrix3<F> {
        conic::compute_matrix(self.a, self.b, self.theta, self.x, self.y)
    }
}

impl<F: na::RealField + Copy> EllipseRepr for Quadratic<F> {
    /*
    See https://linux-blog.anracom.com/2023/09/04/properties-of-ellipses-by-matrix-coefficients-i-two-defining-matrices/
    for the math behind implementation.
    */
    type F = F;
    fn semi_major(&self) -> F {
        let (a, b) = self.extract_semi_axes();
        if a > b {
            a
        } else {
            b
        }
    }

    fn semi_minor(&self) -> F {
        let (a, b) = self.extract_semi_axes();
        if a > b {
            b
        } else {
            a
        }
    }

    fn rotation(&self) -> F {
        let m = &self.0;

        let a = m[(0, 0)]; // A[0,0]
        let b = m[(0, 1)]; // A[0,1] or A[1,0] (symmetric)
        let c = m[(1, 1)]; // A[1,1]

        let numerator = F::from_usize(2).unwrap() * b;
        let denominator = c - a;

        -F::atan2(numerator, denominator) / F::from_usize(2).unwrap()
    }

    fn center(&self) -> Result<(F, F), PlanarEllipseError> {
        // Get the top-left 2x2 block
        let a_33 = self.fixed_view::<2, 2>(0, 0);

        // Get its inverse
        let a_33_inv = a_33
            .try_inverse()
            .ok_or(PlanarEllipseError::NoCenterCoordinates(
                "Matrix is not invertible.",
            ))?;

        // Get the upper-right 2x1 block (last column of first two rows)
        let b = na::Vector2::new(self[(0, 2)], self[(1, 2)]);

        // Multiply inverse by -b to get center coordinates
        let coords = -(a_33_inv * b);
        let x = coords[0];
        let y = coords[1];

        Ok((x, y))
    }

    fn to_matrix(&self) -> na::Matrix3<F> {
        self.0
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
        self.0 .0.norm()
    }

    fn norm_squared(&self) -> Self::Norm {
        self.0 .0.norm_squared()
    }

    fn scale_mut(&mut self, n: Self::Norm) {
        self.0 .0.scale_mut(n);
    }

    fn unscale_mut(&mut self, n: Self::Norm) {
        self.0 .0.unscale_mut(n);
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
        &self.0 .0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    mod rotation_tests {
        use super::*;
        use crate::math::conic;
        use approx::assert_relative_eq;
        use std::f64::consts::PI;

        #[test]
        fn test_rotation_angles() {
            let test_angles = vec![0.0, PI / 6.0, PI / 4.0, PI / 3.0, PI / 2.0];

            for &theta in &test_angles {
                println!("Testing angle: {}", theta);

                // Create an ellipse with the given rotation
                let matrix = conic::compute_matrix(2.0, 1.0, theta, 0.0, 0.0);
                let ellipse = PlanarEllipse::<Quadratic<f64>>::try_from_matrix(matrix).unwrap();

                // Extract the rotation and compare
                let extracted = ellipse.rotation();
                println!("Input angle: {}, Extracted: {}", theta, extracted);

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
        use approx::assert_relative_eq;
        use std::f64::consts::PI;

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
            // Print parameters for debugging
            println!("\nOriginal parameters:");
            println!("semi_major: {}", e1.semi_major());
            println!("semi_minor: {}", e1.semi_minor());
            println!("rotation: {}", e1.rotation());
            let (x1, y1) = e1.center().unwrap();
            println!("center: ({}, {})", x1, y1);

            println!("\nConverted parameters:");
            println!("semi_major: {}", e2.semi_major());
            println!("semi_minor: {}", e2.semi_minor());
            println!("rotation: {}", e2.rotation());
            let (x2, y2) = e2.center().unwrap();
            println!("center: ({}, {})", x2, y2);

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

            println!("\nNormalized parameters for comparison:");
            println!("Original: a={}, b={}, θ={}", maj1, min1, rot1);
            println!("Converted: a={}, b={}, θ={}", maj2, min2, rot2);

            assert_relative_eq!(maj1, maj2, epsilon = epsilon);
            assert_relative_eq!(min1, min2, epsilon = epsilon);
            assert_relative_eq!(rot1, rot2, epsilon = epsilon);
            assert_relative_eq!(x1, x2, epsilon = epsilon);
            assert_relative_eq!(y1, y2, epsilon = epsilon);
        }

        #[test]
        fn test_circle_conversion() {
            let parametric = PlanarEllipse::from_parameters(2.0, 2.0, 0.0, 0.0, 0.0);

            println!("\nTesting circle conversion:");
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            println!("\nQuadratic matrix:");
            println!("{:?}", quadratic);

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

            println!("\nTesting horizontal ellipse conversion:");
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            println!("\nQuadratic matrix:");
            println!("{:?}", quadratic);

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

            println!("\nTesting rotated ellipse conversion:");
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            println!("\nQuadratic matrix:");
            println!("{:?}", quadratic);

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

            println!("\nTesting translated ellipse conversion:");
            let quadratic = parametric.clone().try_into_quadratic().unwrap();
            println!("\nQuadratic matrix:");
            println!("{:?}", quadratic);

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
