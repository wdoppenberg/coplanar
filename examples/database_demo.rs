use coplanar::SpatialEllipse;
/// Demo of the ellipse triad database with rerun visualization
use coplanar::db::{InMemoryBackend, IndexOptions, IndexedStorageBackend};
use coplanar::math::invariants::compute_invariants;
use nalgebra as na;
use std::f64::consts::PI;

/// Demo of the ellipse triad database with rerun visualization
use coplanar::db::EllipseTriadDatabase;
#[cfg(feature = "rerun")]
use rerun::RecordingStream;

fn create_marker_pattern() -> Vec<(usize, SpatialEllipse<f64>)> {
    // Create a distinctive marker pattern with multiple ellipses
    vec![
        // Central cluster
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
                1.2,
                0.6,
                PI / 6.0,
            )
            .unwrap(),
        ),
        (
            2,
            SpatialEllipse::new(
                na::Point3::new(1.5, 2.6, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                0.8,
                0.4,
                0.0,
            )
            .unwrap(),
        ),
        // Additional ellipses
        (
            3,
            SpatialEllipse::new(
                na::Point3::new(5.0, 1.0, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                0.9,
                0.45,
                PI / 4.0,
            )
            .unwrap(),
        ),
        (
            4,
            SpatialEllipse::new(
                na::Point3::new(2.0, 4.0, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                1.1,
                0.55,
                -PI / 6.0,
            )
            .unwrap(),
        ),
        (
            5,
            SpatialEllipse::new(
                na::Point3::new(-2.0, 1.5, 0.0),
                na::Unit::new_normalize(na::Vector3::new(0.0, 0.0, 1.0)),
                0.7,
                0.35,
                PI / 3.0,
            )
            .unwrap(),
        ),
    ]
}

#[cfg(feature = "rerun")]
fn visualize_ellipses(
    rec: &RecordingStream,
    path: &str,
    ellipses: &[(usize, &SpatialEllipse<f64>)],
    color: [u8; 3],
) -> anyhow::Result<()> {
    for (id, ellipse) in ellipses {
        // Visualize as a point with a label
        let center = ellipse.center;
        rec.log(
            format!("{}/ellipse_{}", path, id),
            &rerun::Points3D::new([(center.x as f32, center.y as f32, center.z as f32)])
                .with_radii([0.1])
                .with_colors([rerun::Color::from_rgb(color[0], color[1], color[2])])
                .with_labels([format!("E{}", id)]),
        )?;
    }
    Ok(())
}

#[cfg(feature = "rerun")]
fn visualize_triad(
    rec: &RecordingStream,
    path: &str,
    ellipses: &[(usize, &SpatialEllipse<f64>)],
    triad_indices: [usize; 3],
    color: [u8; 3],
) -> anyhow::Result<()> {
    // Draw lines connecting the triad
    let e1 = ellipses
        .iter()
        .find(|(id, _)| *id == triad_indices[0])
        .unwrap()
        .1;
    let e2 = ellipses
        .iter()
        .find(|(id, _)| *id == triad_indices[1])
        .unwrap()
        .1;
    let e3 = ellipses
        .iter()
        .find(|(id, _)| *id == triad_indices[2])
        .unwrap()
        .1;

    let points = vec![
        [e1.center.x as f32, e1.center.y as f32, e1.center.z as f32],
        [e2.center.x as f32, e2.center.y as f32, e2.center.z as f32],
        [e3.center.x as f32, e3.center.y as f32, e3.center.z as f32],
        [e1.center.x as f32, e1.center.y as f32, e1.center.z as f32], // Close the triangle
    ];

    rec.log(
        path,
        &rerun::LineStrips3D::new([points])
            .with_colors([rerun::Color::from_rgb(color[0], color[1], color[2])]),
    )?;

    Ok(())
}

fn main() -> anyhow::Result<()> {
    println!("Ellipse Triad Database Demo");
    println!("============================\n");

    // Create the database with in-memory backend
    let backend = InMemoryBackend::<f64>::new();
    let mut db = EllipseTriadDatabase::new(backend);

    // Add marker pattern to database
    let markers = create_marker_pattern();
    println!("Adding {} ellipses to database...", markers.len());
    db.add_ellipses(markers.clone())?;

    // Build the invariant index
    println!("Building invariant index...");
    let opts = IndexOptions::default();
    db.build_index(&opts)?;
    println!(
        "Index built with {} invariant entries\n",
        db.backend().get_invariants().len()
    );

    // Simulate a "detected" triad (transformed version of first 3 markers)
    println!("Simulating camera detection with transformed ellipses...");
    let rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, PI / 8.0);
    let translation = na::Translation3::new(10.0, -5.0, 2.0);
    let transform = translation.to_homogeneous() * rotation.to_homogeneous();

    let detected_ellipses = vec![
        markers[0].1.transform(&transform)?,
        markers[1].1.transform(&transform)?,
        markers[2].1.transform(&transform)?,
    ];

    println!("Original triad positions:");
    for i in 0..3 {
        println!("  E{}: {:?}", i, markers[i].1.center);
    }
    println!("\nDetected (transformed) positions:");
    for (i, e) in detected_ellipses.iter().enumerate() {
        println!("  E{}: {:?}", i, e.center);
    }

    // Compute invariants for detected triad
    let query_invs = compute_invariants(
        &detected_ellipses[0].ellipse,
        &detected_ellipses[1].ellipse,
        &detected_ellipses[2].ellipse,
    )?;

    println!("\nQuery invariants:");
    println!("  i_ij: {:.6}", query_invs.i_ij);
    println!("  i_ji: {:.6}", query_invs.i_ji);
    println!("  i_ijk: {:.6}", query_invs.i_ijk);

    // Search for matches
    println!("\nSearching for similar triads (threshold=0.01)...");
    let matches = db.find_matches(&query_invs, 0.01, 5)?;

    println!("Found {} matches:", matches.len());
    for (i, result) in matches.iter().enumerate() {
        println!(
            "  Match {}: triad [{}, {}, {}], distance: {:.8}",
            i + 1,
            result.triad[0],
            result.triad[1],
            result.triad[2],
            result.distance
        );
    }

    if !matches.is_empty() {
        let best = &matches[0];
        println!(
            "\n✓ Successfully matched! Best match is triad [{}, {}, {}]",
            best.triad[0], best.triad[1], best.triad[2]
        );
        println!("  Distance: {:.10}", best.distance);
        println!(
            "  Transformation applied: rotation {:.2}° + translation {:?}",
            (PI / 8.0) * 180.0 / PI,
            translation.vector
        );
    }

    // Rerun visualization
    #[cfg(feature = "rerun")]
    {
        println!("\n🎨 Starting Rerun visualization...");
        let rec = rerun::RecordingStreamBuilder::new("ellipse_triad_db").spawn()?;

        // Visualize original database ellipses
        let all_ellipses: Vec<_> = db.all_ellipses().collect();
        visualize_ellipses(&rec, "database", &all_ellipses, [100, 150, 255])?;

        // Visualize the original triad in the database
        if !matches.is_empty() {
            visualize_triad(
                &rec,
                "database/matched_triad",
                &all_ellipses,
                matches[0].triad,
                [0, 255, 0],
            )?;
        }

        // Visualize detected ellipses
        let detected_with_ids: Vec<_> = detected_ellipses
            .iter()
            .enumerate()
            .map(|(i, e)| (i, e))
            .collect();
        visualize_ellipses(&rec, "detected", &detected_with_ids, [255, 100, 100])?;
        visualize_triad(
            &rec,
            "detected/query_triad",
            &detected_with_ids,
            [0, 1, 2],
            [255, 0, 0],
        )?;

        println!("✓ Visualization complete. Check the Rerun viewer!");
    }

    #[cfg(not(feature = "rerun"))]
    {
        println!("\n💡 Tip: Enable the 'rerun' feature to see a 3D visualization:");
        println!("   cargo run --example database_demo --features rerun");
    }

    Ok(())
}
