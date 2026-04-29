//! UAT-friendly CLI: turn a PNG into a RAPID program (and optionally a
//! viewer JSON) without touching any Rust code.
//!
//! Example:
//!   trajectory-cli relief \
//!     --image samples/portrait.png \
//!     --tool tests/fixtures/tool_ball6.json \
//!     --frame tests/fixtures/workpiece_identity.json \
//!     --pixel-size 0.1 --max-depth 4 --stepover 0.3 --safe-z 25 \
//!     --feedrate 300 --out portrait.mod --json portrait_path.json

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use tracing_subscriber::EnvFilter;

use trajectory::frame::WorkpieceFrame;
use trajectory::relief_job::{run_from_path, JobInputs, ReliefRequest};
use trajectory::surface::HeightMapOpts;
use trajectory::tool::Tool;
use trajectory::toolpath::ToolpathParams;

#[derive(Parser, Debug)]
#[command(name = "trajectory-cli", about = "CNC jewelry relief CLI", version)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Generate a relief toolpath from a grayscale PNG.
    Relief(ReliefArgs),
    /// Validate a tool/frame/opts combo without producing output. Exit 0 = ok.
    Validate(ValidateArgs),
}

#[derive(Parser, Debug)]
struct ReliefArgs {
    #[arg(long)]
    image: PathBuf,
    #[arg(long)]
    tool: PathBuf,
    #[arg(long)]
    frame: PathBuf,
    /// Output path for the RAPID program (default: <image>.mod).
    #[arg(long)]
    out: Option<PathBuf>,
    /// Optional toolpath viewer JSON output (for the web frontend).
    #[arg(long)]
    json: Option<PathBuf>,
    /// Free-form note recorded in the RAPID provenance header.
    #[arg(long)]
    note: Option<String>,

    #[arg(long, default_value_t = String::from("HeightmapRelief"))]
    program_name: String,

    // HeightMapOpts
    #[arg(long, default_value_t = 0.1)]
    pixel_size: f64,
    #[arg(long, default_value_t = 5.0)]
    max_depth: f64,
    #[arg(long, default_value_t = 1.0)]
    gamma: f64,
    #[arg(long, default_value_t = 0.5)]
    blur_sigma: f64,
    #[arg(long, default_value_t = false)]
    invert: bool,
    #[arg(long, default_value_t = 1.0)]
    base_pad: f64,

    // ToolpathParams
    #[arg(long, default_value_t = 0.5)]
    stepover: f64,
    #[arg(long, default_value_t = 0.0)]
    direction_deg: f64,
    #[arg(long, default_value_t = 25.0)]
    safe_z: f64,
    #[arg(long, default_value_t = 2.0)]
    feed_height: f64,
    #[arg(long, default_value_t = 1.0)]
    max_segment: f64,
    #[arg(long, default_value_t = 0.0)]
    offset_along_axis: f64,

    // Speeds
    #[arg(long, default_value_t = 300.0)]
    feedrate: f64,
    #[arg(long, default_value_t = 5000.0)]
    rapid_speed: f64,
}

#[derive(Parser, Debug)]
struct ValidateArgs {
    #[arg(long)]
    image: PathBuf,
    #[arg(long)]
    tool: PathBuf,
    #[arg(long)]
    frame: PathBuf,
    #[arg(long, default_value_t = 0.1)]
    pixel_size: f64,
    #[arg(long, default_value_t = 5.0)]
    max_depth: f64,
    #[arg(long, default_value_t = 0.5)]
    stepover: f64,
    #[arg(long, default_value_t = 25.0)]
    safe_z: f64,
}

fn init_tracing() {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(false)
        .with_timer(tracing_subscriber::fmt::time::uptime())
        .init();
}

fn main() -> ExitCode {
    init_tracing();
    let cli = Cli::parse();
    let result = match cli.cmd {
        Cmd::Relief(args) => cmd_relief(args),
        Cmd::Validate(args) => cmd_validate(args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

fn cmd_relief(args: ReliefArgs) -> Result<(), Box<dyn std::error::Error>> {
    let tool = Tool::load_json(&args.tool)?;
    let frame = WorkpieceFrame::load_json(&args.frame)?;
    let request = ReliefRequest {
        program_name: args.program_name,
        heightmap: HeightMapOpts {
            pixel_size_mm: args.pixel_size,
            max_depth_mm: args.max_depth,
            gamma: args.gamma,
            blur_sigma: args.blur_sigma,
            invert: args.invert,
            base_pad_mm: args.base_pad,
        },
        toolpath: ToolpathParams {
            stepover_mm: args.stepover,
            direction_deg: args.direction_deg,
            safe_z_mm: args.safe_z,
            feed_height_mm: args.feed_height,
            max_segment_mm: args.max_segment,
            offset_along_axis_mm: args.offset_along_axis,
        },
        feedrate_mm_min: args.feedrate,
        rapid_speed: args.rapid_speed,
        note: args.note,
    };

    let tool_path = args.tool.display().to_string();
    let frame_path = args.frame.display().to_string();
    let inputs = JobInputs::new(&tool, &frame)
        .with_tool_path(&tool_path)
        .with_frame_path(&frame_path);
    let out = run_from_path(&args.image, &request, &inputs)?;

    let rapid_path = args.out.unwrap_or_else(|| {
        let mut p = args.image.clone();
        p.set_extension("mod");
        p
    });
    std::fs::write(&rapid_path, &out.rapid)?;
    println!(
        "wrote {} ({} poses: {} cutting / {} travel, {} triangles)",
        rapid_path.display(),
        out.stats.poses,
        out.stats.cutting,
        out.stats.travel,
        out.stats.triangles
    );
    if let Some(json_path) = args.json {
        std::fs::write(&json_path, &out.toolpath_json)?;
        println!("wrote {}", json_path.display());
    }
    Ok(())
}

fn cmd_validate(args: ValidateArgs) -> Result<(), Box<dyn std::error::Error>> {
    let tool = Tool::load_json(&args.tool)?;
    let _frame = WorkpieceFrame::load_json(&args.frame)?;

    let opts = HeightMapOpts {
        pixel_size_mm: args.pixel_size,
        max_depth_mm: args.max_depth,
        ..HeightMapOpts::default()
    };
    opts.validate()?;

    // Toolpath validation needs the actual surface AABB, so we just sanity-check
    // the cross-field part the user is most likely to get wrong.
    if args.stepover >= tool.diameter_mm {
        return Err(format!(
            "stepover {} must be < tool diameter {}",
            args.stepover, tool.diameter_mm
        )
        .into());
    }
    if args.safe_z < args.max_depth + 1.0 {
        return Err(format!(
            "safe_z {} must be at least 1 mm above max_depth {}",
            args.safe_z, args.max_depth
        )
        .into());
    }
    if !args.image.exists() {
        return Err(format!("image not found: {}", args.image.display()).into());
    }
    println!("ok");
    Ok(())
}
