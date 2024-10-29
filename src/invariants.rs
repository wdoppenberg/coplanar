use std::fmt::Debug;

use anyhow::{anyhow, Result};
use nalgebra::{self as na};
use num_traits::Float;

#[derive(Debug)]
pub struct CoplanarInvariants<F: Float> {
    i_ij: F,
    i_ji: F,
    i_ik: F,
    i_ki: F,
    i_jk: F,
    i_kj: F,
    i_ijk: F,
}

impl<F: Float> From<CoplanarInvariants<F>> for [F; 7] {
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

impl<F: Float> From<CoplanarInvariants<F>> for Vec<F> {
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

impl<F: Float + Debug + 'static> From<CoplanarInvariants<F>> for na::SVector<F, 7> {
    fn from(value: CoplanarInvariants<F>) -> Self {
        let arr: [F; 7] = value.into();
        Self::from(arr)
    }
}

fn try_scale_det<F: Float + na::RealField + Debug>(
    matrix: na::MatrixView3<F>,
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

fn matrix_adjugate<F: Float + na::RealField + Debug>(
    matrix: na::MatrixView3<F>,
) -> Result<na::Matrix3<F>> {
    let det = matrix.determinant();
    if !det.is_finite() {
        return Err(anyhow!("Singular matrix: Adjugate not defined"));
    }
    // Safe since `matrix` is square at build-time
    let matrix_inv = matrix.try_inverse();
    match matrix_inv {
        Some(m) => {
            let cofactor = m.transpose() * det;
            Ok(cofactor.transpose())
        }
        None => Err(anyhow!("Matrix is not invertible")),
    }
}

fn ellipse_pair_feature<F: Float + na::RealField + Debug>(
    a_1: na::MatrixView3<F>,
    a_2: na::MatrixView3<F>,
) -> Result<(F, F)> {
    let f12 = (a_1
        .try_inverse()
        .ok_or(anyhow!("Matrix a_1 is not invertible"))?
        * a_2)
        .trace();
    let f21 = (a_2
        .try_inverse()
        .ok_or(anyhow!("Matrix a_2 is not invertible"))?
        * a_1)
        .trace();

    Ok((f12, f21))
}

fn ellipse_triad_feature<F: Float + na::RealField + Debug>(
    a_1: na::MatrixView3<F>,
    a_2: na::MatrixView3<F>,
    a_3: na::MatrixView3<F>,
) -> Result<F> {
    let f1 = matrix_adjugate((a_2 + a_3).as_view())?;
    let f2 = matrix_adjugate((a_2 - a_3).as_view())?;
    Ok(((f1 - f2) * a_1).trace())
}

pub fn coplanar_invariants<F: Float + na::RealField + Debug>(
    a_i: na::MatrixView3<F>,
    a_j: na::MatrixView3<F>,
    a_k: na::MatrixView3<F>,
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

// Modified coplanar_invariants function
pub fn coplanar_invariants_stable<F: Float + na::RealField + Debug>(
    a_i: na::MatrixView3<F>,
    a_j: na::MatrixView3<F>,
    a_k: na::MatrixView3<F>,
) -> Result<CoplanarInvariants<F>> {
    // First normalize all matrices
    let a_i_norm = try_scale_det(a_i)?;
    let a_j_norm = try_scale_det(a_j)?;
    let a_k_norm = try_scale_det(a_k)?;

    let (i_ij, i_ji) = ellipse_pair_feature(a_i_norm.as_view(), a_j_norm.as_view())?;
    let (i_ik, i_ki) = ellipse_pair_feature(a_i_norm.as_view(), a_k_norm.as_view())?;
    let (i_jk, i_kj) = ellipse_pair_feature(a_j_norm.as_view(), a_k_norm.as_view())?;
    let i_ijk = ellipse_triad_feature(a_i_norm.as_view(), a_j_norm.as_view(), a_k_norm.as_view())?;

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
    use crate::ellipse::*;

    #[test]
    fn test_normalized_coplanar_invariants() -> Result<()> {
        // Create ellipses with normalized matrices
        let [e1, e2, e3] = [(0., 0.), (-5., -4.), (6., 5.)]
            .map(|(x, y)| EllipseParametric::new(1., 0.5, 0., x, y))
            .map(EllipseMatrix::from)
            .map(na::Unit::new_normalize);

        println!("Normalized matrices:");
        println!("E1:\n{}", e1.as_ref());
        println!("E2:\n{}", e2.as_ref());
        println!("E3:\n{}", e3.as_ref());

        // Check determinants
        println!("Determinants:");
        println!("det(E1) = {}", e1.determinant());
        println!("det(E2) = {}", e2.determinant());
        println!("det(E3) = {}", e3.determinant());

        let invariants = coplanar_invariants(e1.as_view(), e2.as_view(), e3.as_view())?;

        println!("Invariants: {:?}", invariants);

        Ok(())
    }
}
