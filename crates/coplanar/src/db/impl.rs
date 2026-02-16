use crate::db::{BackendError, EllipseRecord, IndexOptions, IndexedStorageBackend, SearchResult};
use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;
use crate::{EllipseIdx, SpatialEllipse};
use alloc::vec::Vec;

/// High-level database interface for ellipse triad matching
pub struct EllipseTriadDatabase<B, F>
where
    B: IndexedStorageBackend<F>,
    F: Float,
{
    backend: B,
    _phantom: core::marker::PhantomData<F>,
}

impl<B, F> EllipseTriadDatabase<B, F>
where
    B: IndexedStorageBackend<F>,
    F: Float,
{
    /// Create a new database with the given backend
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Add an ellipse to the database
    pub fn add_ellipse(
        &mut self,
        id: EllipseIdx,
        ellipse: SpatialEllipse<F>,
    ) -> Result<(), BackendError> {
        self.backend.add_record(EllipseRecord { id, ellipse })
    }

    /// Add multiple ellipses
    pub fn add_ellipses(
        &mut self,
        ellipses: impl IntoIterator<Item = (EllipseIdx, SpatialEllipse<F>)>,
    ) -> Result<(), BackendError> {
        for (id, ellipse) in ellipses {
            self.add_ellipse(id, ellipse)?;
        }
        Ok(())
    }

    /// Remove an ellipse from the database
    pub fn remove_ellipse(&mut self, id: &EllipseIdx) -> Result<(), BackendError> {
        self.backend.remove(id)
    }

    /// Get an ellipse by ID
    pub fn get_ellipse(&self, id: &EllipseIdx) -> Option<&SpatialEllipse<F>> {
        self.backend.select(id).map(|r| &r.ellipse)
    }

    /// Get all ellipses
    pub fn all_ellipses(&self) -> impl Iterator<Item = (EllipseIdx, &SpatialEllipse<F>)> {
        self.backend
            .all_records()
            .iter()
            .map(|r| (r.id, &r.ellipse))
    }

    /// Build the invariant index
    pub fn build_index(&mut self, opts: &IndexOptions) -> Result<(), BackendError> {
        self.backend.build_index(opts)
    }

    /// Check if the index is built
    pub fn is_indexed(&self) -> bool {
        self.backend.is_indexed()
    }

    /// Find matching triads given query invariants
    pub fn find_matches(
        &self,
        query_invariants: &CoplanarInvariants<F>,
        threshold: F,
        max_results: usize,
    ) -> Result<Vec<SearchResult<F>>, BackendError> {
        self.backend
            .find_similar_triads(query_invariants, threshold, max_results)
    }

    /// Get the number of ellipses in the database
    pub fn len(&self) -> usize {
        self.backend.len()
    }

    /// Check if the database is empty
    pub fn is_empty(&self) -> bool {
        self.backend.is_empty()
    }

    /// Get access to the underlying backend
    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Get mutable access to the underlying backend
    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }
}
