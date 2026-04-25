#![cfg(feature = "surface")]

use trajectory::math3d::{Point3D, Vec3};
use trajectory::surface::{Surface, StlSurface};

const FIXTURE: &str = "tests/fixtures/flat_plane.stl";

#[test]
fn test_load_flat_plane_stl() {
    let s = StlSurface::load(FIXTURE).expect("load STL");
    assert_eq!(s.mesh().triangles.len(), 2);
    assert_eq!(s.aabb().min, [0.0, 0.0, 0.0]);
    assert_eq!(s.aabb().max, [100.0, 100.0, 0.0]);
    for tri in &s.mesh().triangles {
        assert!((tri.normal.z - 1.0).abs() < 1e-6, "expected +Z normal, got {:?}", tri.normal);
    }
}

#[test]
fn test_ray_cast_hits_plane() {
    let s = StlSurface::load(FIXTURE).expect("load STL");
    let hit = s
        .mesh()
        .ray_cast(Point3D::new(50.0, 50.0, 100.0), Vec3::new(0.0, 0.0, -1.0))
        .expect("hit");
    assert!((hit.1.x - 50.0).abs() < 1e-6);
    assert!((hit.1.y - 50.0).abs() < 1e-6);
    assert!(hit.1.z.abs() < 1e-6);
    assert!((hit.2.z - 1.0).abs() < 1e-6);
}

#[test]
fn test_ray_cast_misses_off_plane() {
    let s = StlSurface::load(FIXTURE).expect("load STL");
    let hit = s
        .mesh()
        .ray_cast(Point3D::new(-10.0, -10.0, 100.0), Vec3::new(0.0, 0.0, -1.0));
    assert!(hit.is_none());
}

#[test]
fn test_project_down_returns_surface_point() {
    let s = StlSurface::load(FIXTURE).expect("load STL");
    let (p, n) = s.project_down(25.0, 75.0).expect("hit");
    assert!((p.z).abs() < 1e-6);
    assert!((n.z - 1.0).abs() < 1e-6);
}
