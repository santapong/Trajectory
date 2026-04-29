//! End-to-end pipeline:
//!   PNG heightmap + tool.json + workpiece.json
//!     -> HeightMapSurface (pixel intensity -> Z, grid -> Mesh)
//!     -> ZigZagStrategy (parallel passes, -Z ray-cast onto surface)
//!     -> AbbRapidPost (MODULE with MoveJ travels and MoveL cuts)

use trajectory::frame::WorkpieceFrame;
use trajectory::post::{AbbRapidPost, PostContext, PostProcessor};
use trajectory::surface::{HeightMapOpts, HeightMapSurface, Surface};
use trajectory::tool::Tool;
use trajectory::toolpath::{ToolPathStrategy, ToolpathParams, ZigZagStrategy};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Trajectory: PNG heightmap -> ABB RAPID ===\n");

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 4.0,
        gamma: 1.0,
        blur_sigma: 0.5,
        invert: false,
        base_pad_mm: 2.0,
    };
    let surface = HeightMapSurface::from_png("tests/fixtures/relief_ramp.png", opts)?;
    let tool = Tool::load_json("tests/fixtures/tool_ball6.json")?;
    let frame = WorkpieceFrame::load_json("tests/fixtures/workpiece_identity.json")?;

    println!(
        "Loaded heightmap: {} triangles, AABB {:?} -> {:?}",
        surface.mesh().triangles.len(),
        surface.aabb().min,
        surface.aabb().max
    );
    println!("Tool: {} ({:?}, d={}mm, l={}mm)", tool.name, tool.kind, tool.diameter_mm, tool.length_mm);

    let params = ToolpathParams {
        stepover_mm: 1.0,
        direction_deg: 0.0,
        safe_z_mm: 20.0,
        feed_height_mm: 2.0,
        max_segment_mm: 1.0,
        offset_along_axis_mm: 0.0,
    };
    let toolpath = ZigZagStrategy.generate(&surface, &tool, &params);
    let cutting = toolpath.feed_flags.iter().filter(|b| **b).count();
    let travel = toolpath.feed_flags.iter().filter(|b| !**b).count();
    println!(
        "Generated toolpath: {} poses ({} cutting, {} travel)",
        toolpath.poses.len(),
        cutting,
        travel
    );

    let opts_json = serde_json::to_string(&opts).unwrap_or_default();
    let metadata = trajectory::post::JobMetadata::new()
        .with_source_file("tests/fixtures/relief_ramp.png")?
        .with_opts_json(opts_json)
        .with_tool_path("tests/fixtures/tool_ball6.json")
        .with_frame_path("tests/fixtures/workpiece_identity.json")
        .with_note("example: surface_heightmap_zigzag");

    let ctx = PostContext {
        program_name: "HeightmapRelief",
        tool: &tool,
        frame: &frame,
        feedrate_mm_min: 300.0,
        rapid_speed: 5000.0,
        metadata: Some(&metadata),
    };
    let rapid = AbbRapidPost.emit(&toolpath, &ctx)?;

    println!("\n--- RAPID (first 30 lines) ---");
    for line in rapid.lines().take(30) {
        println!("{line}");
    }
    println!("... ({} lines total)", rapid.lines().count());

    Ok(())
}
