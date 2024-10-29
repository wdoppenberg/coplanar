pub mod camera;
pub mod ellipse;
mod error;
pub mod invariants;
pub mod matcher;
pub mod spatial_ellipse;
pub mod utils;

pub type EllipseIdx = usize;
pub use ellipse::{EllipseMatrix, EllipseParametric};
