//! Zig-zag (boustrophedon) toolpath generator.
//!
//! Generates parallel passes along the rotated +U axis with step-over along
//! +V, projects each (u,v) sample down onto the surface via -Z ray cast, and
//! reverses every other pass. Between passes, emits retract → traverse →
//! plunge → approach moves at safe / feed heights, all tagged as travel.

use crate::math3d::{Point3D, Pose6D, UnitVec3, Vec3};
use crate::surface::Surface;
use crate::tool::Tool;

use super::{Toolpath, ToolPathStrategy, ToolpathParams};

pub struct ZigZagStrategy;

impl ToolPathStrategy for ZigZagStrategy {
    fn generate(&self, surface: &dyn Surface, _tool: &Tool, params: &ToolpathParams) -> Toolpath {
        let aabb = surface.aabb();
        let mut path = Toolpath::new();

        let theta = params.direction_deg.to_radians();
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        let u_dir = (cos_t, sin_t);
        let v_dir = (-sin_t, cos_t);

        let corners = [
            (aabb.min[0], aabb.min[1]),
            (aabb.max[0], aabb.min[1]),
            (aabb.max[0], aabb.max[1]),
            (aabb.min[0], aabb.max[1]),
        ];
        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for (x, y) in corners {
            let u = x * u_dir.0 + y * u_dir.1;
            let v = x * v_dir.0 + y * v_dir.1;
            u_min = u_min.min(u);
            u_max = u_max.max(u);
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }

        let stepover = params.stepover_mm.max(1e-6);
        let segment = params.max_segment_mm.max(1e-6);
        let n_v = ((v_max - v_min) / stepover).floor() as i64 + 1;
        let n_u = ((u_max - u_min) / segment).floor() as i64 + 1;

        let safe_z = params.safe_z_mm.max(aabb.max[2] + 1.0);
        let up_axis = UnitVec3::new_normalize(Vec3::z());

        let mut prev_pass_end: Option<(f64, f64, f64)> = None;

        for i in 0..n_v {
            let v = v_min + (i as f64) * stepover;
            let reverse = i % 2 == 1;

            let mut samples: Vec<(f64, f64)> = Vec::with_capacity(n_u as usize);
            for k in 0..n_u {
                let u = if reverse {
                    u_max - (k as f64) * segment
                } else {
                    u_min + (k as f64) * segment
                };
                let x = u * u_dir.0 + v * v_dir.0;
                let y = u * u_dir.1 + v * v_dir.1;
                samples.push((x, y));
            }

            let mut pass_poses: Vec<Pose6D> = Vec::with_capacity(samples.len());
            for (x, y) in &samples {
                if let Some((p, n)) = surface.project_down(*x, *y) {
                    let offset = params.offset_along_axis_mm;
                    let pos = if offset.abs() < 1e-12 {
                        p
                    } else {
                        Point3D::new(p.x + n.x * offset, p.y + n.y * offset, p.z + n.z * offset)
                    };
                    pass_poses.push(Pose6D::new(pos, n));
                }
            }
            if pass_poses.is_empty() {
                continue;
            }

            let first = *pass_poses.first().unwrap();
            let last = *pass_poses.last().unwrap();

            // Travel between passes: retract → traverse → plunge → approach
            // (all 4 poses tagged feed=false).
            if let Some((px, py, _)) = prev_pass_end {
                path.push(
                    Pose6D::new(Point3D::new(px, py, safe_z), up_axis),
                    false,
                );
                path.push(
                    Pose6D::new(
                        Point3D::new(first.position[0], first.position[1], safe_z),
                        up_axis,
                    ),
                    false,
                );
                let feed_z = first.position[2] + params.feed_height_mm.max(0.0);
                path.push(
                    Pose6D::new(
                        Point3D::new(first.position[0], first.position[1], feed_z),
                        up_axis,
                    ),
                    false,
                );
                path.push(
                    Pose6D::new(
                        Point3D::new(first.position[0], first.position[1], first.position[2]),
                        up_axis,
                    ),
                    false,
                );
            }

            // Cutting poses for this pass.
            for p in &pass_poses {
                path.push(*p, true);
            }

            prev_pass_end = Some((last.position[0], last.position[1], last.position[2]));
        }

        path
    }
}
