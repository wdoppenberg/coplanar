#![doc = include_str!("../README.md")]

pub mod error;
pub mod geom;
pub mod math;
pub mod utils;
pub mod vision;

// Exports
pub use error::Error;
pub use geom::{PlanarEllipse, SpatialEllipse};
pub use math::compute_invariants;

pub type EllipseIdx = usize;
