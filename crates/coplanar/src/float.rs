use nalgebra as na;
use num_traits::Bounded;
use std::fmt::Debug;

/// Trait bounds for a float-type
pub trait Float: na::RealField + Debug + Copy + Send + Sync + PartialOrd + Bounded {}

impl Float for f32 {}
impl Float for f64 {}