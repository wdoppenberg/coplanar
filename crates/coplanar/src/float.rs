use core::fmt::Debug;
use nalgebra as na;
use num_traits::{Bounded, ToPrimitive};

/// Trait bounds for a float-type
pub trait Float:
    na::RealField + Debug + Copy + Send + Sync + PartialOrd + Bounded + ToPrimitive
{
}

impl Float for f32 {}
impl Float for f64 {}
