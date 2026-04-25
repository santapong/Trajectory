//! Point-cloud surface (depth-camera input) — stub for a future version.

use super::{Mesh, Surface, SurfaceError};

pub struct PointCloudSurface {
    _xyz: Vec<[f64; 3]>,
}

impl PointCloudSurface {
    pub fn load_ply(_path: &std::path::Path) -> Result<Self, SurfaceError> {
        Err(SurfaceError::InvalidMesh(
            "point-cloud loading not yet implemented".into(),
        ))
    }
}

impl Surface for PointCloudSurface {
    fn mesh(&self) -> &Mesh {
        unimplemented!("point-cloud → mesh meshing not yet implemented")
    }
}
