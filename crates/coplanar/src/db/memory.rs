use crate::db::backend::{BackendError, BackendState, IndexedStorageBackend, StorageBackend};
use crate::db::index::IndexOptions;
use crate::db::record::EllipseRecord;
use crate::float::Float;
use crate::{EllipseIdx, SpatialEllipse};
use rstar::RTree;
use std::marker::PhantomData;

/// Marker struct for indexed backends
pub struct Indexed {}
impl BackendState for Indexed {}

/// Marker struct for unindexed backends
pub struct Unindexed {}
impl BackendState for Unindexed {}

type EllipseRTree<F> = RTree<EllipseRecord<F>>;

pub struct InMemory<F: Float, State: BackendState> {
    /// R* Tree representing position in 3D space
    pos_tree: RTree<[F; 3]>,
    _state: PhantomData<State>,
}

impl<F: Float, State: BackendState> StorageBackend<F> for InMemory<F, State> {
    async fn add_record(
        self,
        record: impl Into<EllipseRecord<F>>,
    ) -> Result<Self, BackendError> {
        let mut pos_tree = self.pos_tree;
        let pos = record.into().ellipse.center.into();
        pos_tree.insert(pos);
        
        Ok(Self {
            pos_tree,
            _state: PhantomData
        })
    }

    async fn remove(self, index: &EllipseIdx) -> Result<Self, BackendError> {
        todo!()
    }

    async fn select(&self, ellipse_idx: &EllipseIdx) -> Result<&SpatialEllipse<F>, BackendError> {
        todo!()
    }

    async fn build_index<BI: IndexedStorageBackend<F>>(
        self,
        opts: &IndexOptions,
    ) -> Result<BI, BackendError> {
        todo!()
    }
}

impl<F: Float> IndexedStorageBackend<F> for InMemory<F, Indexed> {
    async fn find_similar_triads(
        &self,
        query_invariants: &[F],
        threshold: F,
    ) -> Result<Vec<[EllipseIdx; 3]>, BackendError> {
        todo!()
    }
}
