use crate::geom::ellipse::planar::EllipseRepr;
use crate::math::matrix_adjugate;
use crate::PlanarEllipse;
use anyhow::{anyhow, Result};
use nalgebra as na;
use nalgebra::Unit;
use std::fmt::Debug;

/// Translation-invariant features describing the relationship between three ellipses
#[derive(Debug, Clone, PartialEq)]
pub struct CoplanarInvariants<F: na::RealField + Copy> {
    /// Trace of A_i^(-1) * A_j
    pub i_ij: F,
    /// Trace of A_j^(-1) * A_i
    pub i_ji: F,
    /// Trace of A_i^(-1) * A_k
    pub i_ik: F,
    /// Trace of A_k^(-1) * A_i
    pub i_ki: F,
    /// Trace of A_j^(-1) * A_k
    pub i_jk: F,
    /// Trace of A_k^(-1) * A_j
    pub i_kj: F,
    /// Trace of ((Adj(A_j + A_k) - Adj(A_j - A_k)) * A_i)
    pub i_ijk: F,
}

impl<F: na::RealField + Copy> CoplanarInvariants<F> {
    /// Create new invariants from raw values
    pub fn new(i_ij: F, i_ji: F, i_ik: F, i_ki: F, i_jk: F, i_kj: F, i_ijk: F) -> Self {
        Self {
            i_ij,
            i_ji,
            i_ik,
            i_ki,
            i_jk,
            i_kj,
            i_ijk,
        }
    }

    /// Compute the distance between two sets of invariants
    pub fn distance(&self, other: &Self) -> F {
        let diff = [
            self.i_ij - other.i_ij,
            self.i_ji - other.i_ji,
            self.i_ik - other.i_ik,
            self.i_ki - other.i_ki,
            self.i_jk - other.i_jk,
            self.i_kj - other.i_kj,
            self.i_ijk - other.i_ijk,
        ];

        diff.iter().fold(F::zero(), |acc, &x| acc + x * x).sqrt()
    }
}

impl<F: na::RealField + Copy> From<CoplanarInvariants<F>> for [F; 7] {
    fn from(value: CoplanarInvariants<F>) -> Self {
        [
            value.i_ij,
            value.i_ji,
            value.i_ik,
            value.i_ki,
            value.i_jk,
            value.i_kj,
            value.i_ijk,
        ]
    }
}

impl<F: na::RealField + Copy> From<CoplanarInvariants<F>> for Vec<F> {
    fn from(value: CoplanarInvariants<F>) -> Self {
        vec![
            value.i_ij,
            value.i_ji,
            value.i_ik,
            value.i_ki,
            value.i_jk,
            value.i_kj,
            value.i_ijk,
        ]
    }
}

impl<F: na::RealField + Copy + Debug + 'static> From<CoplanarInvariants<F>> for na::SVector<F, 7> {
    fn from(value: CoplanarInvariants<F>) -> Self {
        let arr: [F; 7] = value.into();
        Self::from(arr)
    }
}

fn try_scale_det<F: na::RealField + Copy + Debug>(
    matrix: &na::Matrix3<F>,
) -> Result<na::Matrix3<F>> {
    let det = matrix.determinant();
    if det == F::zero() {
        return Err(anyhow!("Determinant is zero"));
    }
    if !det.is_finite() {
        return Err(anyhow!("Singular matrix"));
    }
    Ok(matrix * (F::one() / det))
}

/// Generate invariant feature for a pair of conics
pub fn ellipse_pair_feature<F: na::RealField + Copy + Debug>(
    a_1: &na::Matrix3<F>,
    a_2: &na::Matrix3<F>,
) -> Result<(F, F)> {
    // First normalize the matrices
    let det1 = a_1.determinant();
    let det2 = a_2.determinant();

    if !det1.is_finite() || !det2.is_finite() || det1 == F::zero() || det2 == F::zero() {
        return Err(anyhow!("Invalid determinant in matrices"));
    }

    let a1_norm = a_1 * (F::one() / det1.abs().powf(F::one() / F::from_usize(3).unwrap()));
    let a2_norm = a_2 * (F::one() / det2.abs().powf(F::one() / F::from_usize(3).unwrap()));

    let f12 = (a1_norm
        .try_inverse()
        .ok_or(anyhow!("Matrix a_1 is not invertible"))?
        * a2_norm)
        .trace();
    let f21 = (a2_norm
        .try_inverse()
        .ok_or(anyhow!("Matrix a_2 is not invertible"))?
        * a1_norm)
        .trace();

    Ok((f12, f21))
}

/// Generate invariant feature for a set of three conics
pub fn ellipse_triad_feature<F: na::RealField + Copy + Debug>(
    a_1: &na::Matrix3<F>,
    a_2: &na::Matrix3<F>,
    a_3: &na::Matrix3<F>,
) -> Result<F> {
    // Normalize matrices
    let det1 = a_1.determinant();
    let det2 = a_2.determinant();
    let det3 = a_3.determinant();

    if !det1.is_finite()
        || !det2.is_finite()
        || !det3.is_finite()
        || det1 == F::zero()
        || det2 == F::zero()
        || det3 == F::zero()
    {
        return Err(anyhow!("Invalid determinant in matrices"));
    }

    let root3 = F::one() / F::from_usize(3).unwrap();
    let a1_norm = a_1 * (F::one() / det1.abs().powf(root3));
    let a2_norm = a_2 * (F::one() / det2.abs().powf(root3));
    let a3_norm = a_3 * (F::one() / det3.abs().powf(root3));

    let sum = a2_norm + a3_norm;
    let diff = a2_norm - a3_norm;

    let f1 = matrix_adjugate(&sum.as_view())?;
    let f2 = matrix_adjugate(&diff.as_view())?;
    Ok(((f1 - f2) * a1_norm).trace())
}

pub fn coplanar_invariants<F: na::RealField + Debug + Copy>(
    a_i: &na::Matrix3<F>,
    a_j: &na::Matrix3<F>,
    a_k: &na::Matrix3<F>,
) -> Result<CoplanarInvariants<F>> {
    let (i_ij, i_ji) = ellipse_pair_feature(a_i, a_j)?;
    let (i_ik, i_ki) = ellipse_pair_feature(a_i, a_k)?;
    let (i_jk, i_kj) = ellipse_pair_feature(a_j, a_k)?;

    let i_ijk = ellipse_triad_feature(a_i, a_j, a_k)?;

    Ok(CoplanarInvariants {
        i_ij,
        i_ji,
        i_ik,
        i_ki,
        i_jk,
        i_kj,
        i_ijk,
    })
}

/// Compute coplanar invariants from three normalized ellipse matrices
pub fn compute_invariants<F, R>(
    ellipses: [&Unit<PlanarEllipse<R>>; 3],
) -> Result<CoplanarInvariants<F>>
where
    F: na::RealField + Copy + Debug,
    R: EllipseRepr<F = F>,
{
    let mats = ellipses.map(|e| e.to_matrix());
    coplanar_invariants(&mats[0], &mats[1], &mats[2])
}

// Modified coplanar_invariants function
pub fn coplanar_invariants_stable<F: na::RealField + Copy + Debug>(
    a_i: &na::Matrix3<F>,
    a_j: &na::Matrix3<F>,
    a_k: &na::Matrix3<F>,
) -> Result<CoplanarInvariants<F>> {
    // First normalize all matrices
    let a_i_norm = try_scale_det(a_i)?;
    let a_j_norm = try_scale_det(a_j)?;
    let a_k_norm = try_scale_det(a_k)?;

    let (i_ij, i_ji) = ellipse_pair_feature(&a_i_norm, &a_j_norm)?;
    let (i_ik, i_ki) = ellipse_pair_feature(&a_i_norm, &a_k_norm)?;
    let (i_jk, i_kj) = ellipse_pair_feature(&a_j_norm, &a_k_norm)?;
    let i_ijk = ellipse_triad_feature(&a_i_norm, &a_j_norm, &a_k_norm)?;

    Ok(CoplanarInvariants {
        i_ij,
        i_ji,
        i_ik,
        i_ki,
        i_jk,
        i_kj,
        i_ijk,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::ellipse::planar::Quadratic;
    use crate::PlanarEllipse;
    use crate::SpatialEllipse;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    fn create_test_ellipses() -> [na::Unit<PlanarEllipse<Quadratic<f64>>>; 3] {
        // Create a triangular configuration for better stability
        [
            (0.0, 0.0), // Center
            (4.0, 0.0), // Right
            (-4., -1.), // Top (equilateral triangle)
        ]
        .map(|(x, y)| {
            let e = PlanarEllipse::from_parameters(1.0, 1.0, 0.0, x, y); // Using circles for stability
            na::Unit::new_normalize(e.try_into_quadratic().unwrap())
        })
    }

    fn create_spatial_ellipses() -> [SpatialEllipse<f64>; 3] {
        [
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
        })
    }

    #[test]
    fn test_matrix_properties() -> Result<()> {
        let ellipses = create_test_ellipses();

        // Check that matrices are normalized
        for e in &ellipses {
            assert_relative_eq!(e.norm(), 1.0, epsilon = 1e-10);

            // Check determinant is non-zero
            let det = e.determinant();
            println!("Determinant: {}", det);
            assert!(det != 0.0);

            // Check that matrix is invertible
            assert!(e.try_inverse().is_some());
        }

        Ok(())
    }

    #[test]
    fn test_invariant_under_translation() -> Result<()> {
        let spatial_ellipses = create_spatial_ellipses();
        let original_inv = compute_invariants([
            &spatial_ellipses[0].ellipse,
            &spatial_ellipses[1].ellipse,
            &spatial_ellipses[2].ellipse,
        ])?;

        // Apply global translation
        let translation = na::Translation3::new(10.0, -5.0, 3.0).to_homogeneous();
        let translated_ellipses: Vec<_> = spatial_ellipses
            .iter()
            .map(|e| e.transform(&translation).unwrap())
            .collect();

        let translated_inv = compute_invariants([
            &translated_ellipses[0].ellipse,
            &translated_ellipses[1].ellipse,
            &translated_ellipses[2].ellipse,
        ])?;

        assert_relative_eq!(original_inv.distance(&translated_inv), 0.0, epsilon = 1e-6);

        Ok(())
    }

    #[test]
    fn test_invariant_under_rotation() -> Result<()> {
        let spatial_ellipses = create_spatial_ellipses();
        let original_inv = compute_invariants([
            &spatial_ellipses[0].ellipse,
            &spatial_ellipses[1].ellipse,
            &spatial_ellipses[2].ellipse,
        ])?;

        // Apply global rotation
        let rotation = na::UnitQuaternion::from_euler_angles(PI / 4.0, 0.0, 0.0).to_homogeneous();
        let rotated_ellipses: Vec<_> = spatial_ellipses
            .iter()
            .map(|e| e.transform(&rotation).unwrap())
            .collect();

        let rotated_inv = compute_invariants([
            &rotated_ellipses[0].ellipse,
            &rotated_ellipses[1].ellipse,
            &rotated_ellipses[2].ellipse,
        ])?;

        assert_relative_eq!(original_inv.distance(&rotated_inv), 0.0, epsilon = 1e-6);

        Ok(())
    }

    #[test]
    fn test_invariant_stability() -> Result<()> {
        let spatial_ellipses = create_spatial_ellipses();
        let base_inv = compute_invariants([
            &spatial_ellipses[0].ellipse,
            &spatial_ellipses[1].ellipse,
            &spatial_ellipses[2].ellipse,
        ])?;

        // Create slightly perturbed ellipse
        let perturbed = SpatialEllipse::new(
            na::Point3::new(1e-6, 1e-6, 0.0),
            na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
            1.0 + 1e-6,
            1.0 + 1e-6,
            1e-6,
        )
        .unwrap();

        let perturbed_inv = compute_invariants([
            &perturbed.ellipse,
            &spatial_ellipses[1].ellipse,
            &spatial_ellipses[2].ellipse,
        ])?;

        assert!(base_inv.distance(&perturbed_inv) < 1e-3);

        Ok(())
    }
}
