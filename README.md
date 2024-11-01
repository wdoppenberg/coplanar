# `coplanar`

# Invariants for Coplanar Ellipses

A Rust library for computing translation and rotation invariant features from sets of coplanar ellipses. This library is
particularly useful for computer vision and geometric analysis applications where you need to compare ellipse
configurations regardless of their position or orientation in space.

## Features

- Compute invariant features from pairs and triads of ellipses
- Translation and rotation invariant measurements
- Stable numerical computations with proper matrix normalization
- Support for both planar and spatial ellipse representations
- Comprehensive error handling for singular cases

## Usage

```rust
use nalgebra as na;
use coplanar::{PlanarEllipse, compute_invariants};

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
```

WIP, so breaking changes will occur. Contributions are most welcome.

MSRV: 1.81.0
