#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

extern crate alloc;

pub mod db;
pub mod error;
mod float;
pub mod geom;
mod matcher;
pub mod math;
pub mod utils;
pub mod vision;

// Exports
pub use error::Error;
pub use geom::{PlanarEllipse, SpatialEllipse};
pub use math::compute_invariants;

pub type EllipseIdx = usize;
