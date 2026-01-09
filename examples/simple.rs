use coplanar::{compute_invariants, PlanarEllipse};

fn main() -> Result<(), coplanar::Error> {
    let ellipses = [
        PlanarEllipse::from_parameters(2.0, 1.0, 0.0, 0., 0.),
        PlanarEllipse::from_parameters(1.5, 1.0, 0.0, 4., 0.),
        PlanarEllipse::from_parameters(2.0, 1.0, 0.0, -4., -1.),
    ];

    // Compute invariant features
    let invariants = compute_invariants(&ellipses[0], &ellipses[1], &ellipses[2])?;

    println!("I: {:#?}", invariants);

    Ok(())
}
