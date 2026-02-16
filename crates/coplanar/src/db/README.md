# Ellipse Triad Database

A lightweight, embedded-friendly database for matching coplanar ellipse triads using projected invariants.

## Overview

This database implementation enables efficient matching of ellipse triads based on their projected invariants, which are transformation-invariant features. This is particularly useful for:

- Camera pose estimation from known marker patterns
- Object localization using detected ellipses
- Robust matching under rotation and translation

## Features

- **Multiple backends**: In-memory and file-based storage
- **Fast querying**: R*-tree spatial indexing over 7D invariant space
- **Embedded-friendly**: Minimal allocation, sync API suitable for microcontrollers
- **Persistence**: File backend with auto-save functionality
- **Type-safe**: Generic over float types (f32, f64)

## Architecture

### Backend Traits

- `StorageBackend`: Basic CRUD operations for ellipse records
- `IndexedStorageBackend`: Extended backend with invariant indexing and search

### Backends

1. **InMemoryBackend**: Stores ellipses in Vec, uses R*-tree for fast invariant search
2. **FileBackend**: Wraps InMemoryBackend, provides persistence to disk

### Key Components

- **EllipseRecord**: Stores a spatial ellipse with an ID
- **InvariantEntry**: Pre-computed invariants for a triad
- **SearchResult**: Match result with triad indices and distance
- **IndexOptions**: Configuration for index building

## Usage

```rust
use coplanar::db::{EllipseTriadDatabase, InMemoryBackend, IndexOptions};
use coplanar::math::invariants::compute_invariants;
use coplanar::SpatialEllipse;

// Create database
let backend = InMemoryBackend::<f64>::new();
let mut db = EllipseTriadDatabase::new(backend);

// Add ellipses
for (id, ellipse) in ellipses {
    db.add_ellipse(id, ellipse)?;
}

// Build index
db.build_index(&IndexOptions::default())?;

// Query for matches
let query_invariants = compute_invariants(&e1, &e2, &e3)?;
let matches = db.find_matches(&query_invariants, 0.01, 10)?;

// Process results
for result in matches {
    println!("Match: {:?}, distance: {}", result.triad, result.distance);
}
```

## File Backend

```rust
use coplanar::db::{FileBackend, EllipseTriadDatabase};

// Create or load from file
let backend = FileBackend::<f64>::new("ellipses.db")?;
let mut db = EllipseTriadDatabase::new(backend);

// Add ellipses...

// Auto-saves on drop, or manually:
db.backend_mut().save()?;
```

## Invariants

The database uses 7 projected invariants computed from ellipse matrix representations:

- `i_ij`, `i_ji`: Pairwise invariants between ellipses i and j
- `i_ik`, `i_ki`: Pairwise invariants between ellipses i and k
- `i_jk`, `i_kj`: Pairwise invariants between ellipses j and k
- `i_ijk`: Triple invariant for the triad

These invariants are invariant under:
- Translation
- Rotation
- Uniform scaling (when normalized)

## Performance

- **Index building**: O(n³) where n is the number of ellipses (generates all triads)
- **Query**: O(log m) where m is the number of triads, using R*-tree nearest neighbor search
- **Storage**: ~152 bytes per ellipse record (f64), ~56 bytes per invariant entry

## Examples

See `examples/database_demo.rs` for a complete demonstration including:
- Creating a marker pattern
- Building the index
- Simulating detected ellipses with transformation
- Matching and visualization with rerun

Run with:
```bash
cargo run --example database_demo
cargo run --example database_demo --features rerun  # With visualization
```

## Future Extensions

The trait-based design allows for future backends:
- Embedded flash storage
- Memory-mapped files
- Distributed databases
- GPU-accelerated search
