use crate::float::Float;
use crate::math::invariants::CoplanarInvariants;
use crate::EllipseIdx;

pub struct InvariantEntry<F: Float> {
    pub(crate) triad: [EllipseIdx; 3],
    pub(crate) invariants: CoplanarInvariants<F>,
}
