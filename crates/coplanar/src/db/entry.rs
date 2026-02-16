use crate::EllipseIdx;
use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;

#[derive(Debug, Clone)]
pub struct InvariantEntry<F: Float> {
    pub triad: [EllipseIdx; 3],
    pub invariants: CoplanarInvariants<F>,
}
