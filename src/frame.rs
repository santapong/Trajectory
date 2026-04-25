//! Workpiece frame: maps the workpiece (where the surface lives) into the
//! machine or robot base coordinate system.
//!
//! Either an explicit 4x4 matrix or XYZ + RPY (degrees) — both forms parse
//! from the same JSON file via `#[serde(untagged)]`.

use std::path::Path;

use nalgebra::{Isometry3, Matrix4};
use serde::{Deserialize, Serialize};

use crate::math3d::isometry_from_xyz_rpy;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FrameSpec {
    Matrix { matrix: [[f64; 4]; 4] },
    XyzRpy { xyz_mm: [f64; 3], rpy_deg: [f64; 3] },
}

#[derive(Debug, Clone)]
pub struct WorkpieceFrame {
    pub workpiece_to_machine: Isometry3<f64>,
}

impl WorkpieceFrame {
    pub fn identity() -> Self {
        Self {
            workpiece_to_machine: Isometry3::identity(),
        }
    }

    pub fn load_json<P: AsRef<Path>>(p: P) -> std::io::Result<Self> {
        let bytes = std::fs::read(p)?;
        let spec: FrameSpec = serde_json::from_slice(&bytes)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        Ok(Self::from_spec(&spec))
    }

    pub fn from_spec(spec: &FrameSpec) -> Self {
        let iso = match spec {
            FrameSpec::XyzRpy { xyz_mm, rpy_deg } => {
                let rpy_rad = [
                    rpy_deg[0].to_radians(),
                    rpy_deg[1].to_radians(),
                    rpy_deg[2].to_radians(),
                ];
                isometry_from_xyz_rpy(*xyz_mm, rpy_rad)
            }
            FrameSpec::Matrix { matrix } => {
                let m = Matrix4::new(
                    matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
                    matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
                    matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3],
                    matrix[3][0], matrix[3][1], matrix[3][2], matrix[3][3],
                );
                Isometry3::from_parts(
                    nalgebra::Translation3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]),
                    nalgebra::UnitQuaternion::from_matrix(&m.fixed_view::<3, 3>(0, 0).into()),
                )
            }
        };
        Self {
            workpiece_to_machine: iso,
        }
    }

    pub fn transform(&self) -> &Isometry3<f64> {
        &self.workpiece_to_machine
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_xyz_rpy() {
        let json = r#"{ "xyz_mm": [10, 20, 30], "rpy_deg": [0, 0, 0] }"#;
        let spec: FrameSpec = serde_json::from_str(json).unwrap();
        let f = WorkpieceFrame::from_spec(&spec);
        let p = nalgebra::Point3::new(0.0, 0.0, 0.0);
        let tp = f.transform().transform_point(&p);
        assert!((tp.x - 10.0).abs() < 1e-9);
        assert!((tp.y - 20.0).abs() < 1e-9);
        assert!((tp.z - 30.0).abs() < 1e-9);
    }

    #[test]
    fn identity_passthrough() {
        let f = WorkpieceFrame::identity();
        let p = nalgebra::Point3::new(1.0, 2.0, 3.0);
        let tp = f.transform().transform_point(&p);
        assert!((tp.x - 1.0).abs() < 1e-9);
        assert!((tp.y - 2.0).abs() < 1e-9);
        assert!((tp.z - 3.0).abs() < 1e-9);
    }
}
