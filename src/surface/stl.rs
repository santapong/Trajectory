//! STL loader. Reads ASCII or binary STL into a [`Mesh`] of [`Triangle`]s.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use crate::math3d::{Point3D, UnitVec3, Vec3};

use super::{Mesh, Surface, SurfaceError, Triangle};

#[derive(Debug, Clone)]
pub struct StlSurface {
    mesh: Mesh,
}

impl StlSurface {
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, SurfaceError> {
        let mut file = BufReader::new(File::open(path)?);
        let stl = stl_io::read_stl(&mut file)
            .map_err(|e| SurfaceError::InvalidMesh(format!("stl_io: {e}")))?;

        let mut tris = Vec::with_capacity(stl.faces.len());
        for face in &stl.faces {
            let v0 = stl.vertices[face.vertices[0]];
            let v1 = stl.vertices[face.vertices[1]];
            let v2 = stl.vertices[face.vertices[2]];
            let p0 = Point3D::new(v0[0] as f64, v0[1] as f64, v0[2] as f64);
            let p1 = Point3D::new(v1[0] as f64, v1[1] as f64, v1[2] as f64);
            let p2 = Point3D::new(v2[0] as f64, v2[1] as f64, v2[2] as f64);

            let raw = Vec3::new(face.normal[0] as f64, face.normal[1] as f64, face.normal[2] as f64);
            let normal = UnitVec3::try_new(raw, 1e-9);

            tris.push(Triangle::new([p0, p1, p2], normal));
        }
        if tris.is_empty() {
            return Err(SurfaceError::InvalidMesh("STL has no triangles".into()));
        }
        let mesh = Mesh::from_triangles(tris);

        let diag = mesh.aabb.diagonal();
        if diag < 1.0 || diag > 5000.0 {
            eprintln!(
                "[trajectory] warning: STL AABB diagonal is {:.3} (expected mm-scale 1..5000); \
                 check that the file is in millimeters.",
                diag
            );
        }

        Ok(Self { mesh })
    }

    pub fn from_mesh(mesh: Mesh) -> Self {
        Self { mesh }
    }
}

impl Surface for StlSurface {
    fn mesh(&self) -> &Mesh {
        &self.mesh
    }
}
