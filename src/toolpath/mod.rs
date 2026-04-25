//! Toolpath data structures and strategy trait.
//!
//! Strategies (zig-zag, scallop, waterline, ...) all consume a [`Surface`]
//! and a [`Tool`] and produce a [`Toolpath`] — a stream of [`Pose6D`]s
//! flagged as either cutting (MoveL/G1) or travel (MoveJ/G0).

use crate::math3d::Pose6D;
use crate::surface::Surface;
use crate::tool::Tool;

pub mod zigzag;

pub use zigzag::ZigZagStrategy;

#[derive(Debug, Clone)]
pub struct Toolpath {
    pub poses: Vec<Pose6D>,
    /// `true` = cutting / feed move (MoveL, G1).
    /// `false` = rapid / travel move (MoveJ, G0).
    pub feed_flags: Vec<bool>,
}

impl Toolpath {
    pub fn new() -> Self {
        Self {
            poses: Vec::new(),
            feed_flags: Vec::new(),
        }
    }

    pub fn push(&mut self, pose: Pose6D, feed: bool) {
        self.poses.push(pose);
        self.feed_flags.push(feed);
    }

    pub fn len(&self) -> usize {
        self.poses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }
}

impl Default for Toolpath {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ToolpathParams {
    pub stepover_mm: f64,
    /// Pass direction in degrees (0 = +X, 90 = +Y).
    pub direction_deg: f64,
    /// Retract Z above mesh AABB max for travel moves.
    pub safe_z_mm: f64,
    /// Approach Z above the surface before plunging into the next pass.
    pub feed_height_mm: f64,
    /// Maximum spacing between sample poses along a pass.
    pub max_segment_mm: f64,
    /// Optional offset along the tool axis (positive = away from surface).
    /// Set to `tool.radius_mm()` if your controller does not support RTCP and
    /// you want the contact point to remain on the surface for a ball-end.
    pub offset_along_axis_mm: f64,
}

impl Default for ToolpathParams {
    fn default() -> Self {
        Self {
            stepover_mm: 1.0,
            direction_deg: 0.0,
            safe_z_mm: 50.0,
            feed_height_mm: 5.0,
            max_segment_mm: 1.0,
            offset_along_axis_mm: 0.0,
        }
    }
}

pub trait ToolPathStrategy {
    fn generate(&self, surface: &dyn Surface, tool: &Tool, params: &ToolpathParams) -> Toolpath;
}
