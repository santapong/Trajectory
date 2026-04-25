//! Heightmap surface (single-camera fallback) — stub for a future version.

use super::{Mesh, Surface, SurfaceError};

pub struct HeightMapSurface {
    _w: u32,
    _h: u32,
    _z: Vec<f64>,
}

impl HeightMapSurface {
    pub fn from_png(_path: &std::path::Path) -> Result<Self, SurfaceError> {
        Err(SurfaceError::InvalidMesh(
            "heightmap loading not yet implemented".into(),
        ))
    }
}

impl Surface for HeightMapSurface {
    fn mesh(&self) -> &Mesh {
        unimplemented!("heightmap → mesh not yet implemented")
    }
}
