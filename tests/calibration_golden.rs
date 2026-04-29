//! UAT calibration golden test.
//!
//! Generates a 4-step pyramid PNG with known step heights, runs the full
//! relief pipeline, and asserts the emitted RAPID's MoveL Z values include
//! every expected step plateau within tolerance. If this test fails, the
//! intensity → Z math is broken and no UAT run can be trusted.

#![cfg(all(feature = "heightmap", feature = "abb"))]

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use trajectory::frame::WorkpieceFrame;
use trajectory::relief_job::{run_from_path, JobInputs, ReliefRequest};
use trajectory::surface::HeightMapOpts;
use trajectory::tool::Tool;
use trajectory::toolpath::ToolpathParams;

fn temp_png_path(tag: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    p.push(format!("trajectory_uat_{tag}_{stamp}.png"));
    p
}

fn write_step_pyramid_png(path: &PathBuf, w: u32, h: u32) -> Vec<u8> {
    // Concentric squares with intensities 0, 64, 128, 192, 255.
    // Center = 255, outermost ring = 0.
    let mut pixels = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let dx = (x as i32 - (w as i32) / 2).abs();
            let dy = (y as i32 - (h as i32) / 2).abs();
            let r = dx.max(dy) as f64;
            let half = (w.min(h) / 2) as f64;
            let frac = (1.0 - r / half).clamp(0.0, 1.0);
            // Quantize to 5 steps: 0, 64, 128, 192, 255
            let step = (frac * 4.0).round() as u8;
            let intensity = match step {
                0 => 0u8,
                1 => 64,
                2 => 128,
                3 => 192,
                _ => 255,
            };
            pixels.push(intensity);
        }
    }
    let file = File::create(path).expect("create png");
    let buf = BufWriter::new(file);
    let mut encoder = png::Encoder::new(buf, w, h);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().expect("png header");
    writer.write_image_data(&pixels).expect("png data");
    writer.finish().expect("png finish");
    pixels
}

fn fixture_path(rel: &str) -> PathBuf {
    let mut p = std::env::current_dir().expect("cwd");
    p.push(rel);
    p
}

#[test]
fn step_pyramid_produces_expected_z_plateaus() {
    let path = temp_png_path("pyramid");
    write_step_pyramid_png(&path, 32, 32);

    // Linear: max_depth 4 mm, so steps should land at exactly:
    // 0 -> 0.0, 64 -> ~1.004, 128 -> ~2.008, 192 -> ~3.012, 255 -> 4.0
    let expected_z = [0.0, 64.0 / 255.0 * 4.0, 128.0 / 255.0 * 4.0, 192.0 / 255.0 * 4.0, 4.0];

    let request = ReliefRequest {
        program_name: "Calibration".into(),
        heightmap: HeightMapOpts {
            pixel_size_mm: 1.0,
            max_depth_mm: 4.0,
            gamma: 1.0,
            blur_sigma: 0.0, // no blur — we want exact plateau heights
            invert: false,
            base_pad_mm: 0.0,
        },
        toolpath: ToolpathParams {
            stepover_mm: 1.0,
            direction_deg: 0.0,
            safe_z_mm: 20.0,
            feed_height_mm: 1.0,
            max_segment_mm: 1.0,
            offset_along_axis_mm: 0.0,
        },
        feedrate_mm_min: 300.0,
        rapid_speed: 5000.0,
        note: Some("calibration_golden".into()),
    };
    let tool_path_str = fixture_path("tests/fixtures/tool_ball6.json").display().to_string();
    let frame_path_str = fixture_path("tests/fixtures/workpiece_identity.json").display().to_string();
    let tool = Tool::load_json(&tool_path_str).expect("tool");
    let frame = WorkpieceFrame::load_json(&frame_path_str).expect("frame");
    let inputs = JobInputs::new(&tool, &frame)
        .with_tool_path(&tool_path_str)
        .with_frame_path(&frame_path_str);

    let out = run_from_path(&path, &request, &inputs).expect("relief job");

    // Provenance header is present and stable
    assert!(out.rapid.contains("! ----- trajectory provenance -----"));
    assert!(out.rapid.contains("git_sha       :"));
    assert!(out.rapid.contains("source_sha256"));
    assert!(out.rapid.contains("note          : calibration_golden"));
    assert!(
        out.rapid.contains(&format!("tool_path     : {}", tool_path_str)),
        "expected tool_path in header, got header:\n{}",
        out.rapid.lines().take(20).collect::<Vec<_>>().join("\n")
    );
    assert!(out.rapid.contains(&format!("frame_path    : {}", frame_path_str)));

    // Toolpath structure non-empty
    assert!(out.stats.poses > 0, "no poses generated");
    assert!(out.stats.cutting > 0, "no cutting moves");

    // Find every CONST robtarget Z and assert each expected plateau appears.
    let zs: Vec<f64> = out
        .rapid
        .lines()
        .filter(|l| l.contains("CONST robtarget"))
        .filter_map(|l| {
            // ...:= [[X.XXX,Y.YYY,Z.ZZZ],[...
            let start = l.find(":= [[")? + 5;
            let rest = &l[start..];
            let end = rest.find(']')?;
            let nums = &rest[..end];
            let parts: Vec<&str> = nums.split(',').collect();
            if parts.len() != 3 {
                return None;
            }
            parts[2].trim().parse::<f64>().ok()
        })
        .collect();
    assert!(zs.len() > 50, "too few robtargets parsed: {}", zs.len());

    for &want in &expected_z {
        let hit = zs.iter().any(|&z| (z - want).abs() < 0.05);
        assert!(
            hit,
            "expected a robtarget Z near {:.3} mm in emitted RAPID, got Zs in [{:.3}, {:.3}]",
            want,
            zs.iter().cloned().fold(f64::INFINITY, f64::min),
            zs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        );
    }

    // No NaNs anywhere; cutting Zs stay within the relief band.
    let cutting_zs: Vec<f64> = out
        .rapid
        .lines()
        .scan(Vec::<f64>::new(), |_acc, _l| Some(_l))
        .filter(|l| l.contains("CONST robtarget"))
        .filter_map(|l| {
            let start = l.find(":= [[")? + 5;
            let rest = &l[start..];
            let end = rest.find(']')?;
            let parts: Vec<&str> = rest[..end].split(',').collect();
            if parts.len() != 3 {
                return None;
            }
            parts[2].trim().parse::<f64>().ok()
        })
        .collect();
    for &z in &cutting_zs {
        assert!(z.is_finite(), "non-finite Z in RAPID");
        assert!(
            z >= -0.001 && z <= request.toolpath.safe_z_mm + 0.5,
            "Z out of expected range: {}",
            z
        );
    }

    // Write a copy alongside the test for human review when they pass.
    if let Ok(mut f) = File::create(std::env::temp_dir().join("trajectory_uat_pyramid.mod")) {
        let _ = f.write_all(out.rapid.as_bytes());
    }

    let _ = std::fs::remove_file(&path);
}

#[test]
fn validate_rejects_stepover_at_or_above_tool_diameter() {
    let path = temp_png_path("oversized");
    write_step_pyramid_png(&path, 16, 16);

    let request = ReliefRequest {
        program_name: "BadStepover".into(),
        heightmap: HeightMapOpts {
            pixel_size_mm: 1.0,
            max_depth_mm: 2.0,
            ..HeightMapOpts::default()
        },
        toolpath: ToolpathParams {
            stepover_mm: 6.0, // == tool diameter
            direction_deg: 0.0,
            safe_z_mm: 20.0,
            feed_height_mm: 1.0,
            max_segment_mm: 1.0,
            offset_along_axis_mm: 0.0,
        },
        feedrate_mm_min: 300.0,
        rapid_speed: 5000.0,
        note: None,
    };
    let tool = Tool::load_json(fixture_path("tests/fixtures/tool_ball6.json")).unwrap();
    let frame = WorkpieceFrame::load_json(fixture_path("tests/fixtures/workpiece_identity.json")).unwrap();
    let inputs = JobInputs::new(&tool, &frame);

    let err = run_from_path(&path, &request, &inputs).expect_err("must reject");
    let msg = format!("{err}");
    assert!(msg.contains("stepover"), "expected stepover error, got: {msg}");

    let _ = std::fs::remove_file(&path);
}

#[test]
fn validate_rejects_unsafe_safe_z() {
    let path = temp_png_path("lowsafe");
    write_step_pyramid_png(&path, 16, 16);

    let request = ReliefRequest {
        program_name: "BadSafeZ".into(),
        heightmap: HeightMapOpts {
            pixel_size_mm: 1.0,
            max_depth_mm: 4.0,
            ..HeightMapOpts::default()
        },
        toolpath: ToolpathParams {
            stepover_mm: 1.0,
            direction_deg: 0.0,
            safe_z_mm: 4.0, // peak relief is 4mm => zero clearance
            feed_height_mm: 1.0,
            max_segment_mm: 1.0,
            offset_along_axis_mm: 0.0,
        },
        feedrate_mm_min: 300.0,
        rapid_speed: 5000.0,
        note: None,
    };
    let tool = Tool::load_json(fixture_path("tests/fixtures/tool_ball6.json")).unwrap();
    let frame = WorkpieceFrame::load_json(fixture_path("tests/fixtures/workpiece_identity.json")).unwrap();
    let inputs = JobInputs::new(&tool, &frame);

    let err = run_from_path(&path, &request, &inputs).expect_err("must reject");
    let msg = format!("{err}");
    assert!(msg.contains("safe_z"), "expected safe_z error, got: {msg}");

    let _ = std::fs::remove_file(&path);
}
