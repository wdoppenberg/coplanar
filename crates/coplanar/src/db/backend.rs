use std::fmt::Debug;
use thiserror::Error;

use crate::db::index::IndexOptions;
use crate::db::record::EllipseRecord;
use crate::float::Float;
use crate::{EllipseIdx, SpatialEllipse};

#[derive(Error, Debug)]
pub enum BackendError {
	#[error("A record with index {0} already exists.")]
	DuplicateIndex(EllipseIdx),
	#[error("A record with index {0} does not exist.")]
	IndexNotFound(EllipseIdx),
}

/// Marker trait for state of backend (unindexed, indexed)
pub trait BackendState {}

pub trait StorageBackend<F: Float>: Sized {
	/// Store a single ellipse
	async fn add(self, index: &EllipseIdx, ellipse: SpatialEllipse<F>) -> Result<Self, BackendError> {
		self.add_record((index, ellipse)).await
	}

	/// Store a single ellipse record
    async fn add_record(self, record: impl Into<EllipseRecord<F>>) -> Result<Self, BackendError>;

	/// Remove a single ellipse (record)
    async fn remove(self, index: &EllipseIdx) -> Result<Self, BackendError>;

	/// Select a single ellipse by index
    async fn select(&self, ellipse_idx: &EllipseIdx) -> Result<&SpatialEllipse<F>, BackendError>;

	/// Build the index and return the indexed version of the backend
    async fn build_index<BI: IndexedStorageBackend<F>>(self, opts: &IndexOptions) -> Result<BI, BackendError>;
}

pub trait IndexedStorageBackend<F: Float>: StorageBackend<F> {
	/// Find similar triads based on translation invariants
    async fn find_similar_triads(&self, query_invariants: &[F], threshold: F) -> Result<Vec<[EllipseIdx; 3]>, BackendError>;
}


