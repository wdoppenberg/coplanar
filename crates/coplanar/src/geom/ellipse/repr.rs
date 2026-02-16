use crate::geom::ellipse::PlanarEllipseError;
use crate::geom::ellipse::planar::{Parametric, Quadratic};
use crate::math::conic;
use core::fmt::Debug;
use nalgebra as na;

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
        let (a, b) = self.semi_axes();
        if a > b { a } else { b }
    }

    fn semi_minor(&self) -> F {
        let (a, b) = self.semi_axes();
        if a > b { b } else { a }
    }

    fn rotation(&self) -> F {
        let m: &na::Matrix3<_> = self;

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
        **self
    }
}
