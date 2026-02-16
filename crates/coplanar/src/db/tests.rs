use super::*;
use crate::SpatialEllipse;
use crate::db::r#impl::EllipseTriadDatabase;
use crate::math::invariants::compute_invariants;
use alloc::vec;
use alloc::vec::Vec;
use approx::assert_relative_eq;
use core::f64::consts::PI;
use nalgebra as na;

fn create_test_ellipses() -> Vec<(usize, SpatialEllipse<f64>)> {
    // Create a set of ellipses in a known configuration
    vec![
        (
            0,
            SpatialEllipse::new(
                na::Point3::new(0.0, 0.0, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                1.0,
                0.5,
                0.0,
            )
            .unwrap(),
        ),
        (
            1,
            SpatialEllipse::new(
                na::Point3::new(3.0, 0.0, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                1.0,
                0.5,
                0.0,
            )
            .unwrap(),
        ),
        (
            2,
            SpatialEllipse::new(
                na::Point3::new(1.5, 2.6, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                1.0,
                0.5,
                0.0,
            )
            .unwrap(),
        ),
        (
            3,
            SpatialEllipse::new(
                na::Point3::new(5.0, 0.0, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                0.8,
                0.4,
                0.0,
            )
            .unwrap(),
        ),
        (
            4,
            SpatialEllipse::new(
                na::Point3::new(2.5, 4.3, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                1.2,
                0.6,
                0.0,
            )
            .unwrap(),
        ),
    ]
}

#[test]
fn test_memory_backend_add_and_select() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    // Add records
    for (id, ellipse) in ellipses.iter() {
        backend
            .add_record(EllipseRecord {
                id: *id,
                ellipse: ellipse.clone(),
            })
            .unwrap();
    }

    assert_eq!(backend.len(), 5);

    // Select specific records
    let record = backend.select(&0).unwrap();
    assert_eq!(record.id, 0);

    let record = backend.select(&2).unwrap();
    assert_eq!(record.id, 2);
}

#[test]
fn test_memory_backend_remove() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    for (id, ellipse) in ellipses {
        backend.add_record(EllipseRecord { id, ellipse }).unwrap();
    }

    assert_eq!(backend.len(), 5);

    // Remove one
    backend.remove(&2).unwrap();
    assert_eq!(backend.len(), 4);
    assert!(backend.select(&2).is_none());

    // Try to remove non-existent
    assert!(backend.remove(&100).is_err());
}

#[test]
fn test_memory_backend_duplicate_index() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    backend
        .add_record(EllipseRecord {
            id: 0,
            ellipse: ellipses[0].1.clone(),
        })
        .unwrap();

    // Try to add duplicate
    let result = backend.add_record(EllipseRecord {
        id: 0,
        ellipse: ellipses[1].1.clone(),
    });

    assert!(matches!(result, Err(BackendError::DuplicateIndex(0))));
}

#[test]
fn test_memory_backend_build_index() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    for (id, ellipse) in ellipses {
        backend.add_record(EllipseRecord { id, ellipse }).unwrap();
    }

    assert!(!backend.is_indexed());

    let opts = IndexOptions::default();
    backend.build_index(&opts).unwrap();

    assert!(backend.is_indexed());

    // Check that invariants were computed
    let invariants = backend.get_invariants();
    assert!(!invariants.is_empty());
}

#[test]
fn test_memory_backend_find_similar_triads() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    for (id, ellipse) in ellipses.iter() {
        backend
            .add_record(EllipseRecord {
                id: *id,
                ellipse: ellipse.clone(),
            })
            .unwrap();
    }

    // Build index
    let opts = IndexOptions::default();
    backend.build_index(&opts).unwrap();

    // Compute invariants for a known triad
    let query_invs = compute_invariants(
        &ellipses[0].1.ellipse,
        &ellipses[1].1.ellipse,
        &ellipses[2].1.ellipse,
    )
    .unwrap();

    // Search for similar triads
    let results = backend.find_similar_triads(&query_invs, 0.1, 10).unwrap();

    assert!(!results.is_empty());

    // The exact triad should be found with distance ≈ 0
    let best = &results[0];
    assert!(best.distance < 1e-10);

    // Check that the triad matches (in some order)
    let mut triad_ids = best.triad;
    triad_ids.sort();
    assert_eq!(triad_ids, [0, 1, 2]);
}

#[cfg(feature = "std")]
#[test]
fn test_file_backend_persistence() {
    use tempfile::tempdir;

    let dir = tempdir().unwrap();
    let path = dir.path().join("test_db.coplanar");

    // Create and populate database
    {
        let mut backend = FileBackend::<f64>::new(&path).unwrap();
        let ellipses = create_test_ellipses();

        for (id, ellipse) in ellipses {
            backend.add_record(EllipseRecord { id, ellipse }).unwrap();
        }

        assert_eq!(backend.len(), 5);

        // Explicitly save
        backend.save().unwrap();
    }

    // Load from file
    {
        let backend = FileBackend::<f64>::new(&path).unwrap();
        assert_eq!(backend.len(), 5);

        // Check that records were loaded correctly
        let record = backend.select(&0).unwrap();
        assert_eq!(record.id, 0);
        assert_relative_eq!(record.ellipse.center.x, 0.0, epsilon = 1e-10);
    }
}

#[cfg(feature = "std")]
#[test]
fn test_file_backend_auto_save() {
    use tempfile::tempdir;

    let dir = tempdir().unwrap();
    let path = dir.path().join("test_db_auto.coplanar");

    // Create and populate database
    {
        let mut backend = FileBackend::<f64>::new(&path).unwrap();
        let ellipses = create_test_ellipses();

        for (id, ellipse) in ellipses.into_iter().take(3) {
            backend.add_record(EllipseRecord { id, ellipse }).unwrap();
        }

        assert_eq!(backend.len(), 3);
        // Drop will trigger auto-save
    }

    // Load from file
    {
        let backend = FileBackend::<f64>::new(&path).unwrap();
        assert_eq!(backend.len(), 3);
    }
}

#[test]
fn test_database_api() {
    let backend = InMemoryBackend::<f64>::new();
    let mut db = EllipseTriadDatabase::new(backend);

    let ellipses = create_test_ellipses();

    // Add ellipses
    db.add_ellipses(ellipses).unwrap();
    assert_eq!(db.len(), 5);
    assert!(!db.is_empty());

    // Get ellipse
    let ellipse = db.get_ellipse(&0).unwrap();
    assert_relative_eq!(ellipse.center.x, 0.0, epsilon = 1e-10);

    // Build index
    let opts = IndexOptions::default();
    db.build_index(&opts).unwrap();
    assert!(db.is_indexed());

    // Find matches
    let query_invs = compute_invariants(
        &db.get_ellipse(&0).unwrap().ellipse,
        &db.get_ellipse(&1).unwrap().ellipse,
        &db.get_ellipse(&2).unwrap().ellipse,
    )
    .unwrap();

    let matches = db.find_matches(&query_invs, 0.1, 5).unwrap();
    assert!(!matches.is_empty());
}

#[test]
fn test_triad_matching_with_transformation() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    // Add original ellipses
    for (id, ellipse) in ellipses.iter() {
        backend
            .add_record(EllipseRecord {
                id: *id,
                ellipse: ellipse.clone(),
            })
            .unwrap();
    }

    // Build index
    backend.build_index(&IndexOptions::default()).unwrap();

    // Create a transformed version of the first triad
    let rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 4.0);
    let translation = na::Translation3::new(10.0, -5.0, 3.0);
    let transform = translation.to_homogeneous() * rotation.to_homogeneous();

    let transformed_ellipses = vec![
        ellipses[0].1.transform(&transform).unwrap(),
        ellipses[1].1.transform(&transform).unwrap(),
        ellipses[2].1.transform(&transform).unwrap(),
    ];

    // Compute invariants from transformed triad
    let query_invs = compute_invariants(
        &transformed_ellipses[0].ellipse,
        &transformed_ellipses[1].ellipse,
        &transformed_ellipses[2].ellipse,
    )
    .unwrap();

    // Should match the original triad
    let results = backend.find_similar_triads(&query_invs, 0.1, 5).unwrap();

    assert!(!results.is_empty());
    assert!(results[0].distance < 1e-6);
}

#[test]
fn test_index_invalidation() {
    let mut backend = InMemoryBackend::<f64>::new();
    let ellipses = create_test_ellipses();

    for (id, ellipse) in ellipses.iter().take(3) {
        backend
            .add_record(EllipseRecord {
                id: *id,
                ellipse: ellipse.clone(),
            })
            .unwrap();
    }

    // Build index
    backend.build_index(&IndexOptions::default()).unwrap();
    assert!(backend.is_indexed());

    // Add new ellipse - should invalidate index
    backend
        .add_record(EllipseRecord {
            id: 10,
            ellipse: ellipses[3].1.clone(),
        })
        .unwrap();
    assert!(!backend.is_indexed());

    // Rebuild
    backend.build_index(&IndexOptions::default()).unwrap();
    assert!(backend.is_indexed());

    // Remove ellipse - should invalidate
    backend.remove(&10).unwrap();
    assert!(!backend.is_indexed());
}
