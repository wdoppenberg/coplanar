/// Options for building the invariant index
#[derive(Debug, Clone)]
pub struct IndexOptions {
    /// Minimum distance between ellipses in a triad to be considered valid
    /// This helps filter out degenerate triads
    pub min_triad_distance: Option<f64>,
    /// Maximum number of triads to generate per ellipse
    /// None means generate all possible triads
    pub max_triads_per_ellipse: Option<usize>,
}

impl Default for IndexOptions {
    fn default() -> Self {
        Self {
            min_triad_distance: Some(0.1),
            max_triads_per_ellipse: None,
        }
    }
}
