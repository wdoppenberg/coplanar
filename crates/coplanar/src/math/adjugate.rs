use std::fmt::Debug;

use nalgebra::{self as na, Matrix3, MatrixView3};
use thiserror::Error;

/// Errors that can occur during adjugate matrix calculation
#[derive(Error, Debug, PartialEq)]
pub enum AdjugateError {
    #[error("Matrix contains NaN or infinite values")]
    NanError,
    #[error("Matrix is numerically unstable: {0}")]
    NumericalInstabilityError(String),
}

/// Calculates the adjugate (classical adjoint) of a 3x3 matrix.
///
/// The adjugate matrix is the transpose of the cofactor matrix.
/// For a 3x3 matrix, it can be calculated directly using cross products
/// of rows/columns to ensure numerical stability.
///
/// # Arguments
/// * `matrix` - A 3x3 matrix view reference
///
/// # Returns
/// * `Result<Matrix3<f64>, AdjugateError>` - The adjugate matrix or an error
///
/// # Errors
/// * `AdjugateError::NanError` - If the input matrix contains NaN or infinity values
/// * `AdjugateError::NumericalInstabilityError` - If the matrix values are too large or small
pub fn matrix_adjugate<F: na::RealField + Copy + Debug>(
    matrix: &MatrixView3<F>,
) -> Result<Matrix3<F>, AdjugateError> {
    // Check for infinite or NaN values
    if matrix.iter().any(|&x| !x.is_finite()) {
        return Err(AdjugateError::NanError);
    }

    // Check for numerical stability
    for &value in matrix.iter() {
        if value != F::zero() && value.abs() < F::from_f64(1e-10).unwrap() {
            return Err(AdjugateError::NumericalInstabilityError(format!(
                "Value too small: {}",
                value
            )));
        }
        if value.abs() > F::from_f64(1e10).unwrap() {
            return Err(AdjugateError::NumericalInstabilityError(format!(
                "Value too large: {}",
                value
            )));
        }
    }

    // Extract rows as vectors for easier cross product calculation
    let row0 = matrix.row(0);
    let row1 = matrix.row(1);
    let row2 = matrix.row(2);

    // Calculate cross products of rows
    let col0 = row1.cross(&row2).transpose();
    let col1 = row2.cross(&row0).transpose();
    let col2 = row0.cross(&row1).transpose();

    // Construct the adjugate matrix
    Ok(Matrix3::from_columns(&[col0, col1, col2]))
}

#[cfg(test)]
mod tests {
    use core::{f32, f64};

    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Matrix3;

    #[test]
    fn test_valid_matrix_adjugate() {
        let matrix = Matrix3::new(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);

        let expected = Matrix3::new(-24.0, 18.0, 5.0, 20.0, -15.0, -4.0, -5.0, 4.0, 1.0);

        let view = matrix.as_view();
        let result = matrix_adjugate(&view).unwrap();

        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(result[(i, j)], expected[(i, j)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn test_identity_matrix_adjugate() {
        let identity = Matrix3::<f64>::identity();
        let view = identity.as_view();
        let result = matrix_adjugate(&view).unwrap();

        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(
                    result[(i, j)],
                    if i == j { 1.0 } else { 0.0 },
                    epsilon = 1e-10
                );
            }
        }
    }

    #[test]
    fn test_nan_matrix() {
        let matrix = Matrix3::new(1.0, f64::NAN, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
        let view = matrix.as_view();
        assert_eq!(matrix_adjugate(&view), Err(AdjugateError::NanError));
    }

    #[test]
    fn test_infinite_matrix() {
        let matrix = Matrix3::new(1.0, f64::INFINITY, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
        let view = matrix.as_view();
        assert_eq!(matrix_adjugate(&view), Err(AdjugateError::NanError));
    }

    #[test]
    fn test_numerically_unstable_matrix_large() {
        // Use a value close to f64::MAX / 3 to ensure overflow in cross product
        let large_value = f64::MAX / 3.;
        let matrix = Matrix3::new(large_value, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
        let view = matrix.as_view();
        assert!(matches!(
            matrix_adjugate(&view),
            Err(AdjugateError::NumericalInstabilityError(_))
        ));
    }

    #[test]
    fn test_numerically_unstable_matrix_small() {
        // Use a value close to f64::MIN_POSITIVE to test small values
        let small_value = f64::MIN_POSITIVE * 3.;
        let matrix = Matrix3::new(small_value, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
        let view = matrix.as_view();
        assert!(matches!(
            matrix_adjugate(&view),
            Err(AdjugateError::NumericalInstabilityError(_))
        ));
    }

    #[test]
    fn test_f32_matrix() {
        let matrix = Matrix3::<f32>::new(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0);
        let view = matrix.as_view();
        let result = matrix_adjugate(&view).unwrap();
        let expected = Matrix3::<f32>::new(-24.0, 18.0, 5.0, 20.0, -15.0, -4.0, -5.0, 4.0, 1.0);

        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(result[(i, j)], expected[(i, j)], epsilon = 1e-5);
            }
        }
    }
}
