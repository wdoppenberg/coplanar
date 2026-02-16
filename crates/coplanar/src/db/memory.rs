use crate::EllipseIdx;
use crate::db::backend::{BackendError, IndexedStorageBackend, SearchResult, StorageBackend};
use crate::db::entry::InvariantEntry;
use crate::db::index::IndexOptions;
use crate::db::record::EllipseRecord;
use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;
use alloc::vec::Vec;
use rstar::RTree;

/// In-memory storage backend using Vec for records and R*-tree for invariant search
#[derive(Debug, Clone)]
pub struct InMemoryBackend<F: Float> {
    /// Linear storage of all ellipse records
    records: Vec<EllipseRecord<F>>,
    /// Invariant index: R*-tree over 7D invariant space
    invariant_index: Option<RTree<InvariantPoint<F>>>,
    /// Pre-computed invariant entries
    invariant_entries: Vec<InvariantEntry<F>>,
}

/// Wrapper for invariants as an R*-tree point
#[derive(Debug, Clone, PartialEq)]
struct InvariantPoint<F: Float> {
    point: [F; 7],
    triad: [EllipseIdx; 3],
}

impl<F: Float> rstar::Point for InvariantPoint<F> {
    type Scalar = F;
    const DIMENSIONS: usize = 7;

    fn generate(mut generator: impl FnMut(usize) -> Self::Scalar) -> Self {
        Self {
            point: core::array::from_fn(generator),
            triad: [0, 0, 0],
        }
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        self.point[index]
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        &mut self.point[index]
    }
}

impl<F: Float> InMemoryBackend<F> {
    /// Create a new empty in-memory backend
    pub fn new() -> Self {
        Self {
            records: Vec::new(),
            invariant_index: None,
            invariant_entries: Vec::new(),
        }
    }

    /// Create with pre-allocated capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            records: Vec::with_capacity(capacity),
            invariant_index: None,
            invariant_entries: Vec::new(),
        }
    }

    /// Check if a triad is valid (ellipses are sufficiently separated)
    fn is_valid_triad(&self, indices: [usize; 3], min_distance: Option<f64>) -> bool {
        let Some(min_dist) = min_distance else {
            return true;
        };

        let ellipses = [
            &self.records[indices[0]].ellipse,
            &self.records[indices[1]].ellipse,
            &self.records[indices[2]].ellipse,
        ];

        // Check pairwise distances
        for i in 0..3 {
            for j in (i + 1)..3 {
                let dist = (ellipses[i].center.coords - ellipses[j].center.coords).norm();
                if dist.to_f64().unwrap() < min_dist {
                    return false;
                }
            }
        }
        true
    }
}

impl<F: Float> Default for InMemoryBackend<F> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F: Float> StorageBackend<F> for InMemoryBackend<F> {
    fn add_record(&mut self, record: EllipseRecord<F>) -> Result<(), BackendError> {
        // Check for duplicate index
        if self.records.iter().any(|r| r.id == record.id) {
            return Err(BackendError::DuplicateIndex(record.id));
        }
        self.records.push(record);
        // Invalidate index when adding records
        self.invariant_index = None;
        self.invariant_entries.clear();
        Ok(())
    }

    fn remove(&mut self, index: &EllipseIdx) -> Result<(), BackendError> {
        let pos = self
            .records
            .iter()
            .position(|r| r.id == *index)
            .ok_or(BackendError::IndexNotFound(*index))?;
        self.records.remove(pos);
        // Invalidate index when removing records
        self.invariant_index = None;
        self.invariant_entries.clear();
        Ok(())
    }

    fn select(&self, ellipse_idx: &EllipseIdx) -> Option<&EllipseRecord<F>> {
        self.records.iter().find(|r| r.id == *ellipse_idx)
    }

    fn all_records(&self) -> &[EllipseRecord<F>] {
        &self.records
    }

    fn len(&self) -> usize {
        self.records.len()
    }
}

impl<F: Float> IndexedStorageBackend<F> for InMemoryBackend<F> {
    fn build_index(&mut self, opts: &IndexOptions) -> Result<(), BackendError> {
        use crate::math::invariants::compute_invariants;
        use itertools::Itertools;

        let n = self.records.len();
        if n < 3 {
            return Err(BackendError::InvalidTriad);
        }

        let mut invariant_points = Vec::new();
        self.invariant_entries.clear();

        // Generate all possible triads or limit per options
        for (i, j, k) in (0..n).tuple_combinations() {
            if !self.is_valid_triad([i, j, k], opts.min_triad_distance) {
                continue;
            }

            // Compute invariants for this triad
            let e1 = &self.records[i].ellipse.ellipse;
            let e2 = &self.records[j].ellipse.ellipse;
            let e3 = &self.records[k].ellipse.ellipse;

            match compute_invariants(e1, e2, e3) {
                Ok(invariants) => {
                    let triad = [self.records[i].id, self.records[j].id, self.records[k].id];
                    let inv_array: [F; 7] = invariants.clone().into();

                    invariant_points.push(InvariantPoint {
                        point: inv_array,
                        triad,
                    });

                    self.invariant_entries
                        .push(InvariantEntry { triad, invariants });
                }
                Err(_) => {
                    // Skip invalid triads (degenerate configurations)
                    continue;
                }
            }
        }

        // Build R*-tree from invariant points
        self.invariant_index = Some(RTree::bulk_load(invariant_points));

        Ok(())
    }

    fn is_indexed(&self) -> bool {
        self.invariant_index.is_some()
    }

    fn find_similar_triads(
        &self,
        query_invariants: &CoplanarInvariants<F>,
        threshold: F,
        max_results: usize,
    ) -> Result<Vec<SearchResult<F>>, BackendError> {
        let index = self
            .invariant_index
            .as_ref()
            .ok_or(BackendError::IndexNotBuilt)?;

        let query_point: [F; 7] = query_invariants.clone().into();
        let query = InvariantPoint {
            point: query_point,
            triad: [0, 0, 0], // Dummy triad for query
        };

        // Use nearest neighbor search
        let mut results = Vec::new();

        for neighbor in index.nearest_neighbor_iter(&query) {
            let distance = query_invariants.distance(&CoplanarInvariants::new(
                neighbor.point[0],
                neighbor.point[1],
                neighbor.point[2],
                neighbor.point[3],
                neighbor.point[4],
                neighbor.point[5],
                neighbor.point[6],
            ));

            if distance <= threshold {
                results.push(SearchResult {
                    triad: neighbor.triad,
                    distance,
                });

                if results.len() >= max_results {
                    break;
                }
            } else if !results.is_empty() {
                // Since results are sorted by distance, we can stop
                break;
            }
        }

        Ok(results)
    }

    fn get_invariants(&self) -> &[InvariantEntry<F>] {
        &self.invariant_entries
    }
}
