pub mod backend;
pub mod entry;
#[cfg(feature = "std")]
pub mod file;
pub mod index;
pub mod memory;
pub mod record;

mod r#impl;
#[cfg(test)]
mod tests;

// Re-exports
pub use backend::{BackendError, IndexedStorageBackend, SearchResult, StorageBackend};
pub use entry::InvariantEntry;
#[cfg(feature = "std")]
pub use file::FileBackend;
pub use r#impl::EllipseTriadDatabase;
pub use index::IndexOptions;
pub use memory::InMemoryBackend;
pub use record::EllipseRecord;
