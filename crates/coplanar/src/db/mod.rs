use crate::float::Float;
use backend::StorageBackend;
use record::EllipseRecord;
use std::marker::PhantomData;

mod backend;
mod record;
mod memory;
pub mod entry;
pub mod index;

pub struct EllipseTriadDatabase<B, F>
where
	B: StorageBackend<F> + 'static,
	F: Float
{
	storage: B,
	_marker: PhantomData<F>
}

enum Query<F>
where
	F: Float
{
	ByEllipse(EllipseRecord<F>),
	ByTriad([EllipseRecord<F>; 3]),
}

impl<F> Query<F>
where
	F: Float
{
	fn ellipse(val: impl Into<EllipseRecord<F>>) -> Self {
		Self::ByEllipse(val.into())
	}

	fn triad(val: impl Into<[EllipseRecord<F>; 3]>) -> Self {
		Self::ByTriad(val.into())
	}
}


impl<B, F> EllipseTriadDatabase<B, F>
where
	B: StorageBackend<F> + 'static,
	F: Float
{
	fn select(query: Query<F>) -> Vec<EllipseRecord<F>> { 
		todo!() 
	}
	
	fn from_ellipse_iter(iter: impl IntoIterator<Item = impl Into<EllipseRecord<F>>>) {
		todo!()
	}

	fn from_triad_iter(iter: impl IntoIterator<Item = [impl Into<EllipseRecord<F>>; 3]>) {
		todo!()
	}
}
