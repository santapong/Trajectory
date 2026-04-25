//! 3D math primitives used by the surface, toolpath, and post-processor modules.
//!
//! Wraps `nalgebra` types under crate-local aliases and defines [`Pose6D`], the
//! canonical position + tool-axis representation used throughout the 3D pipeline.

use nalgebra::{Isometry3, Matrix3, Matrix4, Point3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use serde::{Deserialize, Serialize};

pub type Vec3 = Vector3<f64>;
pub type Point3D = Point3<f64>;
pub type UnitVec3 = UnitVector3<f64>;
pub type Mat4 = Matrix4<f64>;

/// 6-DOF pose used by the toolpath generator and post-processor.
///
/// Position is the TCP location on the workpiece surface (workpiece frame, mm).
/// Axis is a unit vector pointing FROM the workpiece INTO the tool shank
/// (i.e. it is the tool-Z direction in workpiece coordinates).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Pose6D {
    pub position: [f64; 3],
    pub axis: [f64; 3],
}

impl Pose6D {
    pub fn new(p: Point3D, axis: UnitVec3) -> Self {
        Self {
            position: [p.x, p.y, p.z],
            axis: [axis.x, axis.y, axis.z],
        }
    }

    pub fn position_p3(&self) -> Point3D {
        Point3D::new(self.position[0], self.position[1], self.position[2])
    }

    pub fn axis_v3(&self) -> Vec3 {
        Vec3::new(self.axis[0], self.axis[1], self.axis[2])
    }

    /// Quaternion that rotates +Z onto `self.axis` via the shortest arc
    /// (Rodrigues). Yaw around the tool axis is unconstrained — fine for
    /// 5-axis CNC; robots may want a yaw rule in a later version.
    pub fn orientation_quat(&self) -> UnitQuaternion<f64> {
        let z = Vec3::z_axis();
        let a = self.axis_v3();
        let a_unit = match UnitVec3::try_new(a, 1e-12) {
            Some(u) => u,
            None => return UnitQuaternion::identity(),
        };
        UnitQuaternion::rotation_between_axis(&z, &a_unit)
            .unwrap_or_else(UnitQuaternion::identity)
    }

    pub fn orientation_matrix(&self) -> Matrix3<f64> {
        self.orientation_quat().to_rotation_matrix().into_inner()
    }

    /// Apply a rigid transform (workpiece → machine/robot base).
    pub fn transformed(&self, t: &Isometry3<f64>) -> Pose6D {
        let new_pos = t.transform_point(&self.position_p3());
        let new_axis = t.transform_vector(&self.axis_v3());
        let new_axis_unit = UnitVec3::try_new(new_axis, 1e-12).unwrap_or(UnitVec3::new_normalize(Vec3::z()));
        Pose6D::new(new_pos, new_axis_unit)
    }
}

/// Build an Isometry3 from XYZ translation (mm) and roll/pitch/yaw (radians).
pub fn isometry_from_xyz_rpy(xyz: [f64; 3], rpy_rad: [f64; 3]) -> Isometry3<f64> {
    let translation = Translation3::new(xyz[0], xyz[1], xyz[2]);
    let rotation = UnitQuaternion::from_euler_angles(rpy_rad[0], rpy_rad[1], rpy_rad[2]);
    Isometry3::from_parts(translation, rotation)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_axis_plus_z_is_identity_quat() {
        let p = Pose6D::new(Point3D::new(0.0, 0.0, 0.0), UnitVec3::new_normalize(Vec3::z()));
        let q = p.orientation_quat();
        assert!((q.w - 1.0).abs() < 1e-9);
        assert!(q.i.abs() < 1e-9);
        assert!(q.j.abs() < 1e-9);
        assert!(q.k.abs() < 1e-9);
    }

    #[test]
    fn isometry_translation_only() {
        let t = isometry_from_xyz_rpy([10.0, 20.0, 30.0], [0.0, 0.0, 0.0]);
        let p = Point3D::new(1.0, 2.0, 3.0);
        let tp = t.transform_point(&p);
        assert!((tp.x - 11.0).abs() < 1e-9);
        assert!((tp.y - 22.0).abs() < 1e-9);
        assert!((tp.z - 33.0).abs() < 1e-9);
    }
}
