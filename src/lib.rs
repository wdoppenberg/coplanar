//! # `coplanar`
//!
//! This crate provides functionality for:
//! - Geometric operations on ellipses
//! - Camera calibration and modeling
//! - Feature matching
//!
//! ## Main Components
//!
//! ### Geometry
//! - [`Ellipse`] - 2D ellipse representation
//! - [`SpatialEllipse`] - 3D ellipse representation
//!
//! ### Vision
//! - [`Camera`] - Camera model and parameters
//! - [`Matcher`] - Feature matching functionality

pub mod error;
pub mod geom;
pub mod math;
pub mod utils;
pub mod vision;

// Exports
pub use error::Error;
pub use geom::{PlanarEllipse, SpatialEllipse};

pub type EllipseIdx = usize;
