use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Debug;
use thiserror::Error;

use crate::EllipseIdx;
use crate::db::entry::InvariantEntry;
use crate::db::index::IndexOptions;
use crate::db::record::EllipseRecord;
use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;

#[derive(Error, Debug)]
pub enum BackendError {
    #[error("A record with index {0} already exists.")]
    DuplicateIndex(EllipseIdx),
    #[error("A record with index {0} does not exist.")]
    IndexNotFound(EllipseIdx),
    #[cfg(feature = "std")]
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    #[error("Serialization error: {0}")]
    SerializationError(String),
    #[error("Index not built")]
    IndexNotBuilt,
    #[error("Invalid triad: need exactly 3 distinct ellipse indices")]
    InvalidTriad,
}

/// Result type for search queries
#[derive(Debug, Clone)]
pub struct SearchResult<F: Float> {
    /// The matching triad of ellipse indices
    pub triad: [EllipseIdx; 3],
    /// The distance metric (lower is better)
    pub distance: F,
}

/// Basic storage backend trait for unindexed operations
/// Designed to work on embedded systems with minimal allocation
pub trait StorageBackend<F: Float> {
    /// Store a single ellipse record
    fn add_record(&mut self, record: EllipseRecord<F>) -> Result<(), BackendError>;

    /// Remove a single ellipse by index
    fn remove(&mut self, index: &EllipseIdx) -> Result<(), BackendError>;

    /// Select a single ellipse by index
    fn select(&self, ellipse_idx: &EllipseIdx) -> Option<&EllipseRecord<F>>;

    /// Get all records
    fn all_records(&self) -> &[EllipseRecord<F>];

    /// Number of records
    fn len(&self) -> usize;

    /// Check if empty
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Indexed storage backend supporting fast triad matching
pub trait IndexedStorageBackend<F: Float>: StorageBackend<F> {
    /// Build the invariant index from stored ellipses
    fn build_index(&mut self, opts: &IndexOptions) -> Result<(), BackendError>;

    /// Check if index is built
    fn is_indexed(&self) -> bool;

    /// Find similar triads based on projected invariants
    /// Returns matches sorted by distance (best match first)
    fn find_similar_triads(
        &self,
        query_invariants: &CoplanarInvariants<F>,
        threshold: F,
        max_results: usize,
    ) -> Result<Vec<SearchResult<F>>, BackendError>;

    /// Get invariant entries for debugging/inspection
    fn get_invariants(&self) -> &[InvariantEntry<F>];
}
