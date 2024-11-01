use coplanar::{compute_invariants, PlanarEllipse};

fn main() -> Result<(), coplanar::Error> {
    // Compute invariant features
    let invariants = compute_invariants(
        &PlanarEllipse::from_parameters(2.0, 1.0, 0.0, 0., 0.),
        &PlanarEllipse::from_parameters(1.5, 1.0, 0.0, 4., 0.),
        &PlanarEllipse::from_parameters(2.0, 1.0, 0.0, -4., -1.),
    )?;

    // Access individual features
    println!("I: {:?}", invariants);
    println!("i_ij: {}", invariants.i_ij);
    println!("i_ijk: {}", invariants.i_ijk);

    Ok(())
}
