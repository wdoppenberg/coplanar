use crate::EllipseIdx;
use crate::db::backend::{BackendError, IndexedStorageBackend, SearchResult, StorageBackend};
use crate::db::entry::InvariantEntry;
use crate::db::index::IndexOptions;
use crate::db::memory::InMemoryBackend;
use crate::db::record::EllipseRecord;
use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

/// File-based storage backend with in-memory index
/// Data is persisted to disk in a simple binary format
#[derive(Debug)]
pub struct FileBackend<F: Float> {
    /// Path to the data file
    path: PathBuf,
    /// In-memory backend for actual operations
    memory: InMemoryBackend<F>,
    /// Track if changes have been made since last save
    dirty: bool,
}

impl<F: Float> FileBackend<F> {
    /// Create a new file backend, loading from path if it exists
    pub fn new(path: impl AsRef<Path>) -> Result<Self, BackendError> {
        let path = path.as_ref().to_path_buf();
        let memory = if path.exists() {
            Self::load_from_file(&path)?
        } else {
            InMemoryBackend::new()
        };

        Ok(Self {
            path,
            memory,
            dirty: false,
        })
    }

    /// Load records from a file
    fn load_from_file(path: &Path) -> Result<InMemoryBackend<F>, BackendError> {
        let file = File::open(path)?;
        let mut reader = BufReader::new(file);

        // Read magic number and version
        let mut magic = [0u8; 8];
        reader.read_exact(&mut magic)?;
        if &magic != b"COPLANAR" {
            return Err(BackendError::SerializationError(
                "Invalid file format".to_string(),
            ));
        }

        let mut version = [0u8; 4];
        reader.read_exact(&mut version)?;
        let version = u32::from_le_bytes(version);
        if version != 1 {
            return Err(BackendError::SerializationError(format!(
                "Unsupported version: {}",
                version
            )));
        }

        // Read number of records
        let mut count_bytes = [0u8; 8];
        reader.read_exact(&mut count_bytes)?;
        let count = usize::from_le_bytes(count_bytes);

        let mut backend = InMemoryBackend::with_capacity(count);

        // Read each record
        for _ in 0..count {
            let record = Self::read_record(&mut reader)?;
            backend.add_record(record)?;
        }

        Ok(backend)
    }

    /// Read a single ellipse record from the reader
    fn read_record(reader: &mut impl Read) -> Result<EllipseRecord<F>, BackendError> {
        use crate::SpatialEllipse;

        // Read index
        let mut id_bytes = [0u8; 8];
        reader.read_exact(&mut id_bytes)?;
        let id = usize::from_le_bytes(id_bytes);

        // Read center (3 floats)
        let center = Self::read_point3(reader)?;

        // Read normal (3 floats)
        let normal = nalgebra::Unit::new_normalize(Self::read_vector3(reader)?);

        // Read ellipse matrix (9 floats)
        let mut matrix_data = [F::zero(); 9];
        for v in &mut matrix_data {
            *v = Self::read_float(reader)?;
        }
        let matrix = nalgebra::Matrix3::from_column_slice(&matrix_data);

        // Reconstruct ellipse
        use crate::geom::ellipse::planar::{PlanarEllipse, Quadratic};
        let ellipse = PlanarEllipse::<Quadratic<F>>::try_from_matrix(matrix)
            .map_err(|e| BackendError::SerializationError(e.to_string()))?;

        let spatial_ellipse = SpatialEllipse::from_ellipse_and_pose(ellipse, center, normal);

        Ok(EllipseRecord {
            id,
            ellipse: spatial_ellipse,
        })
    }

    /// Save all records to file
    pub fn save(&mut self) -> Result<(), BackendError> {
        if !self.dirty {
            return Ok(());
        }

        // Create parent directory if it doesn't exist
        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let file = File::create(&self.path)?;
        let mut writer = BufWriter::new(file);

        // Write magic number and version
        writer.write_all(b"COPLANAR")?;
        writer.write_all(&1u32.to_le_bytes())?;

        // Write number of records
        let records = self.memory.all_records();
        writer.write_all(&records.len().to_le_bytes())?;

        // Write each record
        for record in records {
            Self::write_record(&mut writer, record)?;
        }

        writer.flush()?;
        self.dirty = false;
        Ok(())
    }

    /// Write a single record
    fn write_record(
        writer: &mut impl Write,
        record: &EllipseRecord<F>,
    ) -> Result<(), BackendError> {
        use crate::geom::ellipse::repr::EllipseRepr;

        // Write index
        writer.write_all(&record.id.to_le_bytes())?;

        // Write center
        Self::write_point3(writer, &record.ellipse.center)?;

        // Write normal
        Self::write_vector3(writer, &record.ellipse.normal.into_inner())?;

        // Write ellipse matrix
        let matrix = record.ellipse.ellipse.to_matrix();
        for i in 0..9 {
            Self::write_float(writer, matrix[i])?;
        }

        Ok(())
    }

    // Helper methods for reading/writing primitives
    fn read_float(reader: &mut impl Read) -> Result<F, BackendError> {
        let mut bytes = [0u8; 8];
        reader.read_exact(&mut bytes)?;
        let f = f64::from_le_bytes(bytes);
        Ok(F::from_f64(f).unwrap())
    }

    fn write_float(writer: &mut impl Write, value: F) -> Result<(), BackendError> {
        let f = value.to_f64().unwrap();
        writer.write_all(&f.to_le_bytes())?;
        Ok(())
    }

    fn read_point3(reader: &mut impl Read) -> Result<nalgebra::Point3<F>, BackendError> {
        let x = Self::read_float(reader)?;
        let y = Self::read_float(reader)?;
        let z = Self::read_float(reader)?;
        Ok(nalgebra::Point3::new(x, y, z))
    }

    fn write_point3(
        writer: &mut impl Write,
        point: &nalgebra::Point3<F>,
    ) -> Result<(), BackendError> {
        Self::write_float(writer, point.x)?;
        Self::write_float(writer, point.y)?;
        Self::write_float(writer, point.z)?;
        Ok(())
    }

    fn read_vector3(reader: &mut impl Read) -> Result<nalgebra::Vector3<F>, BackendError> {
        let x = Self::read_float(reader)?;
        let y = Self::read_float(reader)?;
        let z = Self::read_float(reader)?;
        Ok(nalgebra::Vector3::new(x, y, z))
    }

    fn write_vector3(
        writer: &mut impl Write,
        vec: &nalgebra::Vector3<F>,
    ) -> Result<(), BackendError> {
        Self::write_float(writer, vec.x)?;
        Self::write_float(writer, vec.y)?;
        Self::write_float(writer, vec.z)?;
        Ok(())
    }

    /// Get the file path
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl<F: Float> Drop for FileBackend<F> {
    fn drop(&mut self) {
        // Auto-save on drop if dirty
        if self.dirty {
            let _ = self.save();
        }
    }
}

impl<F: Float> StorageBackend<F> for FileBackend<F> {
    fn add_record(&mut self, record: EllipseRecord<F>) -> Result<(), BackendError> {
        self.memory.add_record(record)?;
        self.dirty = true;
        Ok(())
    }

    fn remove(&mut self, index: &EllipseIdx) -> Result<(), BackendError> {
        self.memory.remove(index)?;
        self.dirty = true;
        Ok(())
    }

    fn select(&self, ellipse_idx: &EllipseIdx) -> Option<&EllipseRecord<F>> {
        self.memory.select(ellipse_idx)
    }

    fn all_records(&self) -> &[EllipseRecord<F>] {
        self.memory.all_records()
    }

    fn len(&self) -> usize {
        self.memory.len()
    }
}

impl<F: Float> IndexedStorageBackend<F> for FileBackend<F> {
    fn build_index(&mut self, opts: &IndexOptions) -> Result<(), BackendError> {
        self.memory.build_index(opts)
    }

    fn is_indexed(&self) -> bool {
        self.memory.is_indexed()
    }

    fn find_similar_triads(
        &self,
        query_invariants: &CoplanarInvariants<F>,
        threshold: F,
        max_results: usize,
    ) -> Result<Vec<SearchResult<F>>, BackendError> {
        self.memory
            .find_similar_triads(query_invariants, threshold, max_results)
    }

    fn get_invariants(&self) -> &[InvariantEntry<F>] {
        self.memory.get_invariants()
    }
}
