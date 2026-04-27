//! End-to-end pipeline:
//!   STL surface + tool.json + workpiece.json
//!     -> ZigZagStrategy (parallel passes, -Z ray-cast onto surface)
//!     -> AbbRapidPost (MODULE with MoveJ travels and MoveL cuts)
//!     -> printed RAPID program

use trajectory::frame::WorkpieceFrame;
use trajectory::post::{AbbRapidPost, PostContext, PostProcessor};
use trajectory::surface::{StlSurface, Surface};
use trajectory::tool::Tool;
use trajectory::toolpath::{ToolPathStrategy, ToolpathParams, ZigZagStrategy};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Trajectory: Surface zig-zag -> ABB RAPID ===\n");

    let surface = StlSurface::load("tests/fixtures/flat_plane.stl")?;
    let tool = Tool::load_json("tests/fixtures/tool_ball6.json")?;
    let frame = WorkpieceFrame::load_json("tests/fixtures/workpiece_identity.json")?;

    println!(
        "Loaded surface: {} triangles, AABB {:?} -> {:?}",
        surface.mesh().triangles.len(),
        surface.aabb().min,
        surface.aabb().max
    );
    println!("Tool: {} ({:?}, d={}mm, l={}mm)", tool.name, tool.kind, tool.diameter_mm, tool.length_mm);

    let params = ToolpathParams {
        stepover_mm: 5.0,
        direction_deg: 0.0,
        safe_z_mm: 50.0,
        feed_height_mm: 5.0,
        max_segment_mm: 5.0,
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

    let ctx = PostContext {
        program_name: "ZigZagDemo",
        tool: &tool,
        frame: &frame,
        feedrate_mm_min: 600.0,
        rapid_speed: 5000.0,
        metadata: None,
    };
    let rapid = AbbRapidPost.emit(&toolpath, &ctx)?;

    println!("\n--- RAPID (first 30 lines) ---");
    for line in rapid.lines().take(30) {
        println!("{line}");
    }
    println!("... ({} lines total)", rapid.lines().count());

    Ok(())
}
