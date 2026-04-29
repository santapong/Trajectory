//! Toolpath data structures and strategy trait.
//!
//! Strategies (zig-zag, scallop, waterline, ...) all consume a [`Surface`]
//! and a [`Tool`] and produce a [`Toolpath`] — a stream of [`Pose6D`]s
//! flagged as either cutting (MoveL/G1) or travel (MoveJ/G0).

use serde::{Deserialize, Serialize};

use crate::math3d::Pose6D;
use crate::surface::{Aabb, Surface};
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

    /// Compact JSON representation for the frontend viewer.
    /// Shape: `{"poses":[[x,y,z],...], "axes":[[ax,ay,az],...], "feeds":[bool,...], "stats":{...}}`.
    pub fn to_viewer_json(&self) -> String {
        let cuts = self.feed_flags.iter().filter(|b| **b).count();
        let travels = self.feed_flags.len() - cuts;
        let mut out = String::with_capacity(64 + self.poses.len() * 40);
        out.push_str("{\"poses\":[");
        for (i, p) in self.poses.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "[{:.4},{:.4},{:.4}]",
                p.position[0], p.position[1], p.position[2]
            ));
        }
        out.push_str("],\"axes\":[");
        for (i, p) in self.poses.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "[{:.4},{:.4},{:.4}]",
                p.axis[0], p.axis[1], p.axis[2]
            ));
        }
        out.push_str("],\"feeds\":[");
        for (i, f) in self.feed_flags.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(if *f { "true" } else { "false" });
        }
        out.push_str(&format!(
            "],\"stats\":{{\"poses\":{},\"cutting\":{},\"travel\":{}}}}}",
            self.poses.len(),
            cuts,
            travels
        ));
        out
    }
}

impl Default for Toolpath {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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

impl ToolpathParams {
    /// Cross-field safety check. Catches the foot-guns that show up when a
    /// human edits a JSON config and forgets that `stepover_mm` must be
    /// strictly less than the tool diameter, or that `safe_z_mm` must clear
    /// the highest point of the workpiece.
    pub fn validate(&self, tool: &Tool, aabb: &Aabb) -> Result<(), String> {
        if !self.stepover_mm.is_finite() || self.stepover_mm <= 0.0 {
            return Err(format!("stepover_mm must be > 0, got {}", self.stepover_mm));
        }
        if self.stepover_mm >= tool.diameter_mm {
            return Err(format!(
                "stepover_mm ({:.3}) must be strictly less than tool diameter ({:.3}); \
                 typical jewelry stepover is 10–50% of diameter",
                self.stepover_mm, tool.diameter_mm
            ));
        }
        if !self.max_segment_mm.is_finite() || self.max_segment_mm <= 0.0 {
            return Err(format!("max_segment_mm must be > 0, got {}", self.max_segment_mm));
        }
        if !self.feed_height_mm.is_finite() || self.feed_height_mm < 0.0 {
            return Err(format!("feed_height_mm must be >= 0, got {}", self.feed_height_mm));
        }
        if !self.safe_z_mm.is_finite() {
            return Err(format!("safe_z_mm must be finite, got {}", self.safe_z_mm));
        }
        let z_clearance = self.safe_z_mm - aabb.max[2];
        if z_clearance < 1.0 {
            return Err(format!(
                "safe_z_mm ({:.3}) is only {:.3} mm above workpiece peak ({:.3}); \
                 leave at least 1 mm clearance",
                self.safe_z_mm, z_clearance, aabb.max[2]
            ));
        }
        if !self.direction_deg.is_finite() {
            return Err(format!("direction_deg must be finite, got {}", self.direction_deg));
        }
        if !self.offset_along_axis_mm.is_finite() {
            return Err(format!(
                "offset_along_axis_mm must be finite, got {}",
                self.offset_along_axis_mm
            ));
        }
        Ok(())
    }
}

pub trait ToolPathStrategy {
    fn generate(&self, surface: &dyn Surface, tool: &Tool, params: &ToolpathParams) -> Toolpath;
}
