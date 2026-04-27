//! 3D surface representation used as input to the toolpath generator.
//!
//! A [`Surface`] exposes a triangle [`Mesh`] and a `project_down` operation
//! (cast a -Z ray from above to find the surface point). [`StlSurface`] is the
//! v1 implementation; point-cloud and heightmap stubs are present for future
//! versions.

use serde::{Deserialize, Serialize};

use crate::math3d::{Point3D, UnitVec3, Vec3};

pub mod stl;
#[cfg(feature = "pointcloud")]
pub mod pointcloud;
#[cfg(feature = "heightmap")]
pub mod heightmap;

pub use stl::StlSurface;
#[cfg(feature = "heightmap")]
pub use heightmap::{HeightMapOpts, HeightMapSurface};

#[derive(Debug, thiserror::Error)]
pub enum SurfaceError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid mesh: {0}")]
    InvalidMesh(String),
    #[error("ray miss")]
    RayMiss,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    pub fn from_points(points: &[Point3D]) -> Self {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for p in points {
            let coords = [p.x, p.y, p.z];
            for i in 0..3 {
                if coords[i] < min[i] {
                    min[i] = coords[i];
                }
                if coords[i] > max[i] {
                    max[i] = coords[i];
                }
            }
        }
        Self { min, max }
    }

    pub fn diagonal(&self) -> f64 {
        let dx = self.max[0] - self.min[0];
        let dy = self.max[1] - self.min[1];
        let dz = self.max[2] - self.min[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[derive(Debug, Clone)]
pub struct Triangle {
    pub v: [Point3D; 3],
    pub normal: UnitVec3,
}

impl Triangle {
    pub fn new(v: [Point3D; 3], normal: Option<UnitVec3>) -> Self {
        let computed = normal.unwrap_or_else(|| {
            let e1 = v[1] - v[0];
            let e2 = v[2] - v[0];
            let n = e1.cross(&e2);
            UnitVec3::try_new(n, 1e-12).unwrap_or(UnitVec3::new_normalize(Vec3::z()))
        });
        Self { v, normal: computed }
    }
}

#[derive(Debug, Clone)]
pub struct Mesh {
    pub triangles: Vec<Triangle>,
    pub aabb: Aabb,
}

impl Mesh {
    pub fn from_triangles(tris: Vec<Triangle>) -> Self {
        let mut all_pts = Vec::with_capacity(tris.len() * 3);
        for t in &tris {
            all_pts.extend_from_slice(&t.v);
        }
        let aabb = Aabb::from_points(&all_pts);
        Self {
            triangles: tris,
            aabb,
        }
    }

    /// Möller–Trumbore ray cast. Returns nearest (t, hit_point, surface_normal).
    pub fn ray_cast(&self, origin: Point3D, dir: Vec3) -> Option<(f64, Point3D, UnitVec3)> {
        let eps = 1e-9;
        let mut best: Option<(f64, Point3D, UnitVec3)> = None;
        for tri in &self.triangles {
            let v0 = tri.v[0];
            let v1 = tri.v[1];
            let v2 = tri.v[2];
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let h = dir.cross(&edge2);
            let a = edge1.dot(&h);
            if a.abs() < eps {
                continue;
            }
            let f = 1.0 / a;
            let s = origin - v0;
            let u = f * s.dot(&h);
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(&edge1);
            let v = f * dir.dot(&q);
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let t = f * edge2.dot(&q);
            if t > eps {
                let hit = origin + dir * t;
                let candidate = (t, hit, tri.normal);
                match best {
                    None => best = Some(candidate),
                    Some((bt, _, _)) if t < bt => best = Some(candidate),
                    _ => {}
                }
            }
        }
        best
    }
}

/// Pluggable surface source. Must produce a Mesh and a "drop a point onto the
/// surface from above" projection.
pub trait Surface: Send + Sync {
    fn mesh(&self) -> &Mesh;

    fn aabb(&self) -> Aabb {
        self.mesh().aabb
    }

    /// Cast a -Z ray from `(x, y, aabb.max.z + epsilon)`. Returns the hit point
    /// and the surface normal at the hit, or `None` if the ray misses.
    fn project_down(&self, x: f64, y: f64) -> Option<(Point3D, UnitVec3)> {
        let aabb = self.aabb();
        let z_start = aabb.max[2] + 1.0;
        let origin = Point3D::new(x, y, z_start);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        self.mesh()
            .ray_cast(origin, dir)
            .map(|(_, p, n)| (p, n))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_plane() -> Mesh {
        let n = UnitVec3::new_normalize(Vec3::z());
        let t1 = Triangle::new(
            [
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(100.0, 0.0, 0.0),
                Point3D::new(100.0, 100.0, 0.0),
            ],
            Some(n),
        );
        let t2 = Triangle::new(
            [
                Point3D::new(0.0, 0.0, 0.0),
                Point3D::new(100.0, 100.0, 0.0),
                Point3D::new(0.0, 100.0, 0.0),
            ],
            Some(n),
        );
        Mesh::from_triangles(vec![t1, t2])
    }

    #[test]
    fn aabb_covers_plane() {
        let m = flat_plane();
        assert_eq!(m.aabb.min, [0.0, 0.0, 0.0]);
        assert_eq!(m.aabb.max, [100.0, 100.0, 0.0]);
    }

    #[test]
    fn ray_cast_hits_plane_center() {
        let m = flat_plane();
        let hit = m.ray_cast(Point3D::new(50.0, 50.0, 100.0), Vec3::new(0.0, 0.0, -1.0));
        let (_, p, n) = hit.expect("ray should hit");
        assert!((p.x - 50.0).abs() < 1e-6);
        assert!((p.y - 50.0).abs() < 1e-6);
        assert!(p.z.abs() < 1e-6);
        assert!((n.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn ray_cast_misses_off_plane() {
        let m = flat_plane();
        let hit = m.ray_cast(Point3D::new(-10.0, -10.0, 100.0), Vec3::new(0.0, 0.0, -1.0));
        assert!(hit.is_none());
    }
}
