//! End-to-end relief job: PNG bytes (or path) + opts → validated, emits RAPID
//! plus a viewer-friendly toolpath JSON. Shared by the CLI and the web server
//! so both code paths run the *same* validation, tracing, and provenance.

use serde::{Deserialize, Serialize};
use tracing::{info, info_span};

use crate::frame::WorkpieceFrame;
use crate::post::{AbbRapidPost, JobMetadata, PostContext, PostError, PostProcessor};
use crate::surface::{HeightMapOpts, HeightMapSurface, Surface, SurfaceError};
use crate::tool::Tool;
use crate::toolpath::{ToolPathStrategy, Toolpath, ToolpathParams, ZigZagStrategy};

#[derive(Debug, thiserror::Error)]
pub enum JobError {
    #[error("surface: {0}")]
    Surface(#[from] SurfaceError),
    #[error("post: {0}")]
    Post(#[from] PostError),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid toolpath params: {0}")]
    InvalidParams(String),
    #[error("invalid request: {0}")]
    InvalidRequest(String),
}

/// Inputs to a relief job. Mirrors what the web frontend POSTs and what the
/// CLI assembles from flags.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliefRequest {
    pub program_name: String,
    pub heightmap: HeightMapOpts,
    pub toolpath: ToolpathParams,
    pub feedrate_mm_min: f64,
    pub rapid_speed: f64,
    /// Free-form note that ends up in the RAPID provenance comment block.
    #[serde(default)]
    pub note: Option<String>,
}

impl Default for ReliefRequest {
    fn default() -> Self {
        Self {
            program_name: "HeightmapRelief".into(),
            heightmap: HeightMapOpts::default(),
            toolpath: ToolpathParams {
                stepover_mm: 1.0,
                direction_deg: 0.0,
                safe_z_mm: 20.0,
                feed_height_mm: 2.0,
                max_segment_mm: 1.0,
                offset_along_axis_mm: 0.0,
            },
            feedrate_mm_min: 300.0,
            rapid_speed: 5000.0,
            note: None,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ReliefStats {
    pub triangles: usize,
    pub poses: usize,
    pub cutting: usize,
    pub travel: usize,
    pub aabb_min: [f64; 3],
    pub aabb_max: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct ReliefOutput {
    pub rapid: String,
    pub toolpath_json: String,
    pub stats: ReliefStats,
}

/// Run the full pipeline from in-memory PNG bytes. Source label is used in
/// provenance (typically a filename or "<upload>").
pub fn run_from_bytes(
    png_bytes: &[u8],
    source_label: &str,
    request: &ReliefRequest,
    tool: &Tool,
    frame: &WorkpieceFrame,
) -> Result<ReliefOutput, JobError> {
    if png_bytes.is_empty() {
        return Err(JobError::InvalidRequest("empty PNG payload".into()));
    }
    let span = info_span!("relief_job", program = %request.program_name, bytes = png_bytes.len());
    let _enter = span.enter();

    // Write to a temp path because the PNG decoder API in `from_png` is
    // path-based; we route through a tempfile to avoid duplicating decode
    // logic. Cleaned up on drop.
    let tmp_path = {
        let mut p = std::env::temp_dir();
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        p.push(format!("trajectory_relief_{stamp}.png"));
        std::fs::write(&p, png_bytes)?;
        p
    };
    let result = run_inner(&tmp_path, png_bytes, source_label, request, tool, frame);
    let _ = std::fs::remove_file(&tmp_path);
    result
}

fn run_inner(
    tmp_path: &std::path::Path,
    png_bytes: &[u8],
    source_label: &str,
    request: &ReliefRequest,
    tool: &Tool,
    frame: &WorkpieceFrame,
) -> Result<ReliefOutput, JobError> {
    let surface = HeightMapSurface::from_png(tmp_path, request.heightmap)?;
    let aabb = surface.aabb();

    request
        .toolpath
        .validate(tool, &aabb)
        .map_err(JobError::InvalidParams)?;

    let toolpath: Toolpath = {
        let _s = info_span!("toolpath.zigzag").entered();
        ZigZagStrategy.generate(&surface, tool, &request.toolpath)
    };
    let cuts = toolpath.feed_flags.iter().filter(|b| **b).count();
    let travel = toolpath.feed_flags.len() - cuts;
    info!(triangles = surface.mesh().triangles.len(), poses = toolpath.poses.len(), cuts, travel, "toolpath ready");

    let opts_json = serde_json::to_string(&request.heightmap).unwrap_or_default();
    let mut metadata = JobMetadata::new()
        .with_source_bytes(source_label, png_bytes)
        .with_opts_json(opts_json);
    if let Some(note) = &request.note {
        metadata = metadata.with_note(note.clone());
    }

    let ctx = PostContext {
        program_name: &request.program_name,
        tool,
        frame,
        feedrate_mm_min: request.feedrate_mm_min,
        rapid_speed: request.rapid_speed,
        metadata: Some(&metadata),
    };
    let rapid = AbbRapidPost.emit(&toolpath, &ctx)?;
    let toolpath_json = toolpath.to_viewer_json();

    let stats = ReliefStats {
        triangles: surface.mesh().triangles.len(),
        poses: toolpath.poses.len(),
        cutting: cuts,
        travel,
        aabb_min: aabb.min,
        aabb_max: aabb.max,
    };
    Ok(ReliefOutput { rapid, toolpath_json, stats })
}

/// Convenience wrapper for a path on disk.
pub fn run_from_path(
    png_path: &std::path::Path,
    request: &ReliefRequest,
    tool: &Tool,
    frame: &WorkpieceFrame,
) -> Result<ReliefOutput, JobError> {
    let bytes = std::fs::read(png_path)?;
    let label = png_path.display().to_string();
    run_from_bytes(&bytes, &label, request, tool, frame)
}
