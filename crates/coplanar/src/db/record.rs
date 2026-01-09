use std::fmt::Debug;

use crate::float::Float;
use crate::{EllipseIdx, SpatialEllipse};


#[derive(Clone, Debug)]
pub struct EllipseRecord<F: Float>
{
	pub id: EllipseIdx,
	pub ellipse: SpatialEllipse<F>
}

impl<F: Float> PartialEq for EllipseRecord<F> {
	fn eq(&self, other: &Self) -> bool {
		self.id == other.id
	}

	fn ne(&self, other: &Self) -> bool {
		!self.eq(other)
	}
}

impl<F: Float> From<(&EllipseIdx, SpatialEllipse<F>)> for EllipseRecord<F>
{
	fn from(value: (&EllipseIdx, SpatialEllipse<F>)) -> Self {
		Self {
			id: *value.0,
			ellipse: value.1
		}
	}
}