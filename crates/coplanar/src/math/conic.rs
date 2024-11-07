use nalgebra as na;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum EllipseMatrixError {
    #[error("Semi-major axis 'a' must be positive, got {0}")]
    NonPositiveSemiMajorAxis(f64),

    #[error("Semi-minor axis 'b' must be positive, got {0}")]
    NonPositiveSemiMinorAxis(f64),

    #[error("Semi-major axis 'a' must be greater than or equal to semi-minor axis 'b', got a={0}, b={1}")]
    InvalidAxesRatio(f64, f64),

    #[error("Matrix is degenerate, det={0}")]
    Degenerate(f64),

    #[error("Matrix does not define an ellipse")]
    NotAnEllipse(f64),
}

pub fn compute_matrix<F: na::RealField + Copy>(a: F, b: F, theta: F, x: F, y: F) -> na::Matrix3<F> {
    let two =
        F::from_usize(2).expect("Could not initialize 2.0 scalar. Check if usize is supported.");

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let a_pow2 = a.powi(2);
    let b_pow2 = b.powi(2);
    let sin_theta_pow2 = sin_theta.powi(2);
    let cos_theta_pow2 = cos_theta.powi(2);

    let a_quad = a_pow2 * sin_theta_pow2 + b_pow2 * cos_theta_pow2;
    let b_quad = two * (b_pow2 - a_pow2) * cos_theta * sin_theta;
    let c_quad = a_pow2 * cos_theta_pow2 + b_pow2 * sin_theta_pow2;
    let d_quad = -two * a_quad * x - b_quad * y;
    let e_quad = -b_quad * x - two * c_quad * y;
    let f_quad = a_quad * x.powi(2) + b_quad * x * y + c_quad * y.powi(2) - a_pow2 * b_pow2;

    na::Matrix3::from_rows(&[
        [a_quad, b_quad / two, d_quad / two].into(),
        [b_quad / two, c_quad, e_quad / two].into(),
        [d_quad / two, e_quad / two, f_quad].into(),
    ])
}

pub(crate) fn check_ellipse_conditions<F: na::RealField + Copy>(
    matrix: &na::Matrix3<F>,
) -> Result<(), EllipseMatrixError> {
    // Get the determinant
    let det = matrix.determinant();
    if det.abs() < F::default_epsilon() {
        return Err(EllipseMatrixError::Degenerate(det.to_subset_unchecked()));
    }

    // Get the A_33 submatrix (upper-left 2x2)
    let a_33 = matrix.fixed_view::<2, 2>(0, 0);

    // Check if the matrix represents an ellipse
    // For an ellipse, we need the determinant of A₃₃ to be positive
    let discriminant = a_33.determinant();
    if discriminant <= F::zero() {
        return Err(EllipseMatrixError::NotAnEllipse(
            discriminant.to_subset_unchecked(),
        ));
    }

    Ok(())
}

pub fn compute_ellipse_matrix<F: na::RealField + Copy>(
    a: F,
    b: F,
    theta: F,
    x: F,
    y: F,
) -> Result<na::Matrix3<F>, EllipseMatrixError> {
    // Validate inputs
    if a <= F::zero() {
        return Err(EllipseMatrixError::NonPositiveSemiMajorAxis(
            a.to_subset_unchecked(),
        ));
    }
    if b <= F::zero() {
        return Err(EllipseMatrixError::NonPositiveSemiMinorAxis(
            b.to_subset_unchecked(),
        ));
    }
    if a < b {
        return Err(EllipseMatrixError::InvalidAxesRatio(
            a.to_subset_unchecked(),
            b.to_subset_unchecked(),
        ));
    }

    // Compute the matrix using the helper function
    let matrix = compute_matrix(a, b, theta, x, y);

    // Check if the matrix represents a valid ellipse
    check_ellipse_conditions(&matrix)?;

    Ok(matrix)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn assert_matrix_eq(a: &na::Matrix3<f64>, b: &na::Matrix3<f64>) {
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(a[(i, j)], b[(i, j)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn test_circle_at_origin() {
        let m = compute_matrix(1.0, 1.0, 0.0, 0.0, 0.0);
        let expected = na::Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
        assert_matrix_eq(&m, &expected);

        let m = compute_matrix(2.0, 1.0, 0.0, 0.0, 0.0);
        let expected = na::Matrix3::new(1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, -4.0);
        assert_matrix_eq(&m, &expected);
    }

    #[test]
    fn test_circle_rotation_invariance() {
        // A circle should produce the same matrix regardless of rotation
        let m1 = compute_matrix(1.0, 1.0, 0.0, 0.0, 0.0);
        let m2 = compute_matrix(1.0, 1.0, std::f64::consts::PI / 4.0, 0.0, 0.0);
        assert_matrix_eq(&m1, &m2);
    }

    #[test]
    fn test_translated_circle() {
        let m = compute_matrix(1.0, 1.0, 0.0, 1.0, 1.0);
        // For a circle translated to (1,1), the implicit equation should be:
        // (x-1)² + (y-1)² = 1
        // which expands to: x² + y² - 2x - 2y + 1 = 0
        let expected = na::Matrix3::new(
            1.0, 0.0, -1.0, 0.0, 1.0, -1.0, -1.0, -1.0, 1.0, // This was wrong before
        );
        assert_matrix_eq(&m, &expected);
    }

    #[test]
    fn test_horizontal_ellipse() {
        // Test an ellipse with semi-major axis = 2, semi-minor axis = 1
        let m = compute_matrix(2.0, 1.0, 0.0, 0.0, 0.0);
        // The implicit equation should be: x²/4 + y² = 1
        // which is equivalent to: x²/4 + y² - 1 = 0
        // Multiply by 4 to clear fractions: x² + 4y² - 4 = 0
        let expected = na::Matrix3::new(1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, -4.0);
        assert_matrix_eq(&m.scale(0.25), &expected.scale(0.25));
    }

    #[test]
    fn test_rotated_ellipse() {
        // Test a 45-degree rotated ellipse
        let m = compute_matrix(2.0, 1.0, std::f64::consts::PI / 4.0, 0.0, 0.0);
        // For a rotated ellipse, off-diagonal terms should be non-zero
        // and matrix should be symmetric
        assert_relative_eq!(m[(0, 1)], m[(1, 0)], epsilon = 1e-10);
        assert!(m[(0, 1)].abs() > 1e-10); // Should have non-zero off-diagonal terms
    }

    #[test]
    fn test_degenerate_cases() {
        // Test a degenerate case where one axis is zero
        let m = compute_matrix(1.0, 0.0, 0.0, 0.0, 0.0);
        // Should produce valid matrix with some zeros
        assert!(m.iter().any(|&x| x != 0.0)); // Matrix shouldn't be all zeros
        assert!(m.iter().any(|&x| x == 0.0)); // Should have some zeros
    }

    #[test]
    fn test_valid_ellipse() {
        let result = compute_ellipse_matrix(2.0f64, 1.0, 0.0, 0.0, 0.0);
        println!("{:?}", result);
        assert!(result.is_ok());
    }

    #[test]
    fn test_degenerate_matrix() {
        // Creating a degenerate case by using very small values
        let result = compute_ellipse_matrix(1e-10f64, 1e-10, 0.0, 0.0, 0.0);
        println!("{:?}", result);
        assert!(matches!(result, Err(EllipseMatrixError::Degenerate(_))));
    }

    #[test]
    fn test_not_ellipse() {
        // Create a matrix representing a hyperbola: x^2/a^2 - y^2/b^2 = 1
        let a = 2.0f64;
        let b = 1.0f64;
        let matrix = na::Matrix3::new(
            1.0 / (a * a),
            0.0,
            0.0,
            0.0,
            -1.0 / (b * b),
            0.0,
            0.0,
            0.0,
            -1.0,
        );

        // The discriminant (a_11 a_22) should be negative for a hyperbola
        let a33 = matrix.fixed_view::<2, 2>(0, 0);
        assert!(
            a33.determinant() < 0.0,
            "Matrix should represent a hyperbola"
        );

        // Now test our check_ellipse_conditions function
        assert!(matches!(
            check_ellipse_conditions(&matrix),
            Err(EllipseMatrixError::NotAnEllipse(_))
        ));
    }
}
