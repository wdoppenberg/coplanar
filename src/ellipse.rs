use core::fmt;
use std::{
    fmt::{Debug, Display, Formatter},
    ops::{Deref, DerefMut},
};

use nalgebra::{self as na};
use num_traits::Float;

/// An ellipse in a 2D plane described as a 3x3 matrix of the conic section described by the
/// quadratic equation $A_Q$ [1].
///
/// Is compatible with [na::Unit] to ensure a normalised underlying matrix.
///
/// [1] https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
#[derive(Debug, Clone)]
pub struct EllipseMatrix<F: Float + na::RealField + Debug>(na::Matrix3<F>);

impl<F: Float + na::RealField + Debug + Display> Display for EllipseMatrix<F> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl<F: Float + na::RealField + Debug> EllipseMatrix<F> {
    pub fn new(mat: na::Matrix3<F>) -> Self {
        Self(mat)
    }
}

impl<F: Float + na::RealField + Debug> Deref for EllipseMatrix<F> {
    type Target = na::Matrix3<F>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<F: Float + na::RealField + Debug> DerefMut for EllipseMatrix<F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<F: Float + na::RealField + Debug> AsRef<na::Matrix3<F>> for EllipseMatrix<F> {
    fn as_ref(&self) -> &na::Matrix3<F> {
        &self.0
    }
}

impl<F: Float + na::RealField + Debug> AsMut<na::Matrix3<F>> for EllipseMatrix<F> {
    fn as_mut(&mut self) -> &mut na::Matrix3<F> {
        &mut self.0
    }
}

impl<F: Float + na::RealField + Debug> na::Normed for EllipseMatrix<F> {
    type Norm = F;
    fn norm(&self) -> Self::Norm {
        self.0.norm()
    }

    fn norm_squared(&self) -> Self::Norm {
        self.0.norm_squared()
    }

    fn scale_mut(&mut self, n: Self::Norm) {
        self.0.scale_mut(n);
    }

    fn unscale_mut(&mut self, n: Self::Norm) {
        self.0.unscale_mut(n);
    }
}

// pub struct EllipseCollection<F: Float>(Vec<EllipseMatrix<F>>);

/// An ellipse in a 2D plane described in parametric form
#[derive(Debug)]
pub struct EllipseParametric<F: Float> {
    /// Semi-major axis
    a: F,
    /// Semi-minor axis
    b: F,
    /// Rotation
    theta: F,
    /// Offset in X-direction
    x: F,
    /// Offset in Y-direction
    y: F,
}

impl<F: Float> EllipseParametric<F> {
    pub fn new(a: F, b: F, theta: F, x: F, y: F) -> Self {
        if b > a {
            Self {
                a: b,
                b: a,
                theta,
                x,
                y,
            }
        } else {
            Self { a, b, theta, x, y }
        }
    }
}

impl<F: Float + na::Scalar + na::RealField> From<EllipseParametric<F>> for EllipseMatrix<F> {
    fn from(ell: EllipseParametric<F>) -> Self {
        let EllipseParametric { a, b, theta, x, y } = ell;

        // Verbose, but necessary to be generic over F
        let two = F::from(2).expect("Could not initialize 2.0 scalar.");

        let a_quad = Float::powi(a * Float::sin(theta), 2) + Float::powi(b * Float::cos(theta), 2);
        let b_quad = two
            * ((Float::powi(b, 2)) - (Float::powi(a, 2)))
            * Float::cos(theta)
            * Float::sin(theta);
        let c_quad = Float::powi(a * Float::cos(theta), 2) + Float::powi(b * Float::sin(theta), 2);
        let d_quad = -two * a_quad * x - b_quad * y;
        let e_quad = -b_quad * x - two * c_quad * y;
        let f_quad = a_quad * Float::powi(x, 2) + b_quad * x * y + c_quad * Float::powi(y, 2)
            - Float::powi(a, 2) * Float::powi(b, 2);

        let mat = na::Matrix3::from_rows(&[
            [a_quad, b_quad / two, d_quad / two].into(),
            [b_quad / two, c_quad, e_quad / two].into(),
            [d_quad / two, e_quad / two, f_quad].into(),
        ]);

        Self(mat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;
    use approx::{assert_relative_eq, RelativeEq};
    use nalgebra::{Normed, Unit};

    fn verify_ellipse_parametric_with_type<
        F: Float + na::Scalar + na::ComplexField + RelativeEq,
    >() -> Result<()> {
        // Creating instances of EllipseParametric with generic type F
        let ellipse = EllipseParametric::new(
            F::from(3.0).unwrap(),
            F::from(2.0).unwrap(),
            F::from(0.0).unwrap(),
            F::from(1.0).unwrap(),
            F::from(2.0).unwrap(),
        );

        // Just performing some basic assertions to ensure type compatibility
        assert_relative_eq!(ellipse.a, F::from(3.0).unwrap());
        assert_relative_eq!(ellipse.b, F::from(2.0).unwrap());
        assert_relative_eq!(ellipse.theta, F::from(0.0).unwrap());
        assert_relative_eq!(ellipse.x, F::from(1.0).unwrap());
        assert_relative_eq!(ellipse.y, F::from(2.0).unwrap());

        Ok(())
    }

    #[test]
    fn test_ellipse_parametric_f32() -> Result<()> {
        verify_ellipse_parametric_with_type::<f32>()
    }

    #[test]
    fn test_ellipse_parametric_f64() -> Result<()> {
        verify_ellipse_parametric_with_type::<f64>()
    }

    #[test]
    fn test_ellipse_parametric_new() {
        // Test when a > b (normal case)
        let e1 = EllipseParametric::new(5.0, 3.0, 0.0, 0.0, 0.0);
        assert_eq!(e1.a, 5.0);
        assert_eq!(e1.b, 3.0);

        // Test when b > a (should swap)
        let e2 = EllipseParametric::new(3.0, 5.0, 0.0, 0.0, 0.0);
        assert_eq!(e2.a, 5.0);
        assert_eq!(e2.b, 3.0);

        // Test with rotation and offset
        let e3 = EllipseParametric::new(4.0, 2.0, std::f64::consts::PI / 4.0, 1.0, 2.0);
        assert_eq!(e3.theta, std::f64::consts::PI / 4.0);
        assert_eq!(e3.x, 1.0);
        assert_eq!(e3.y, 2.0);
    }

    #[test]
    fn test_ellipse_matrix_conversion() {
        // Test circle (special case where a = b)
        let circle = EllipseParametric::new(2.0, 2.0, 0.0, 0.0, 0.0);
        let circle_matrix = EllipseMatrix::from(circle);

        // For a circle at origin, the matrix should have this form:
        // [ 1  0  0 ]
        // [ 0  1  0 ]
        // [ 0  0 -4 ] (normalized)
        let expected = circle_matrix.as_ref() / circle_matrix.0[(0, 0)];
        assert_relative_eq!(expected[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(1, 1)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(0, 1)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(1, 0)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(2, 2)], -4.0, epsilon = 1e-10);

        // Test standard ellipse without rotation or translation
        let ellipse = EllipseParametric::new(3.0, 2.0, 0.0, 0.0, 0.0);
        let matrix = EllipseMatrix::from(ellipse);

        // For standard position ellipse, the matrix should have form:
        // [ 1/a²   0    0  ]
        // [  0    1/b²  0  ]
        // [  0     0   -1  ] (normalized)
        let expected = matrix.as_ref() / matrix.as_ref()[(0, 0)];
        assert_relative_eq!(expected[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(1, 1)], (3.0 / 2.0).powi(2), epsilon = 1e-10);
        assert_relative_eq!(expected[(0, 1)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(expected[(1, 0)], 0.0, epsilon = 1e-10);

        // Test rotated ellipse
        let rot_ellipse = EllipseParametric::new(3.0, 2.0, std::f64::consts::PI / 4.0, 0.0, 0.0);
        let rot_matrix = EllipseMatrix::from(rot_ellipse);

        // Verify that the matrix is symmetric
        assert_relative_eq!(rot_matrix.0[(0, 1)], rot_matrix.0[(1, 0)], epsilon = 1e-10);
        assert_relative_eq!(rot_matrix.0[(0, 2)], rot_matrix.0[(2, 0)], epsilon = 1e-10);
        assert_relative_eq!(rot_matrix.0[(1, 2)], rot_matrix.0[(2, 1)], epsilon = 1e-10);

        // Test translated ellipse
        let trans_ellipse = EllipseParametric::new(3.0, 2.0, 0.0, 1.0, 2.0);
        let trans_matrix = EllipseMatrix::from(trans_ellipse);

        // Verify that the translation terms are present
        assert_ne!(trans_matrix.0[(0, 2)], 0.0);
        assert_ne!(trans_matrix.0[(1, 2)], 0.0);
    }

    #[test]
    fn test_edge_cases() {
        // Test very small ellipse
        let small = EllipseParametric::new(1e-6, 5e-7, 0.0, 0.0, 0.0);
        let small_matrix = EllipseMatrix::from(small);
        assert!(!small_matrix.0.iter().any(|&x| x.is_nan()));

        // Test nearly circular ellipse
        let nearly_circular = EllipseParametric::new(1.0, 0.99999, 0.0, 0.0, 0.0);
        let nc_matrix = EllipseMatrix::from(nearly_circular);
        assert!(!nc_matrix.0.iter().any(|&x| x.is_nan()));

        // Test large rotation angles
        let large_rotation =
            EllipseParametric::new(2.0, 1.0, 10.0 * std::f64::consts::PI, 0.0, 0.0);
        let lr_matrix = EllipseMatrix::from(large_rotation);
        assert!(!lr_matrix.0.iter().any(|&x| x.is_nan()));
    }

    #[test]
    fn test_matrix_properties() -> Result<()> {
        // Create a test ellipse
        let e1 = EllipseParametric::new(1.0, 0.5, 0.0, 0.0, 0.0);
        let matrix = EllipseMatrix::from(e1);

        // Print the matrix for inspection
        println!("Original matrix:\n{}", matrix.as_ref());

        // Check determinant
        let det = matrix.0.determinant();
        println!("Determinant: {}", det);

        // Check rank
        let svd = matrix.0.svd(true, true);
        println!("Singular values: {}", svd.singular_values);

        Ok(())
    }

    #[test]
    fn test_basic_ellipse_properties() -> Result<()> {
        let e = EllipseParametric::new(1., 0.5, 0., 0., 0.);
        let matrix = EllipseMatrix::from(e);

        // A conic matrix should be symmetric
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(matrix.0[(i, j)], matrix.0[(j, i)], epsilon = 1e-10);
            }
        }

        // The upper-left 2x2 block should be positive definite for an ellipse
        let upper_block = matrix.0.fixed_view::<2, 2>(0, 0);
        let eigenvalues = upper_block
            .eigenvalues()
            .ok_or(anyhow::anyhow!("Failed to compute eigenvalues"))?;

        assert!(
            eigenvalues.iter().all(|&e| e > 0.0),
            "Upper-left block should be positive definite"
        );

        Ok(())
    }

    #[test]
    fn test_unit_ellipse_matrix() -> Result<()> {
        let e = EllipseParametric::new(1., 0.5, 0., 0., 0.);
        let matrix = EllipseMatrix::from(e);

        let umatrix = Unit::new_normalize(matrix);

        let norm = umatrix.norm();

        assert_relative_eq!(norm, 1.0);

        Ok(())
    }
}
