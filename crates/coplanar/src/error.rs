use crate::geom::ellipse::PlanarEllipseError;
use crate::math::conic::EllipseMatrixError;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error(transparent)]
    EllipseMatrix(#[from] EllipseMatrixError),

    #[error(transparent)]
    PlanarEllipse(#[from] PlanarEllipseError),

    #[error("Unknown error: {0}")]
    Unknown(#[from] anyhow::Error),
}
