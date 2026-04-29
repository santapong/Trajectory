#![cfg(feature = "heightmap")]

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;

use trajectory::math3d::{Point3D, Vec3};
use trajectory::surface::{HeightMapOpts, HeightMapSurface, Surface};
use trajectory::tool::Tool;
use trajectory::toolpath::{ToolPathStrategy, ToolpathParams, ZigZagStrategy};

fn write_png_grayscale8(path: &PathBuf, w: u32, h: u32, pixels: &[u8]) {
    assert_eq!(pixels.len(), (w * h) as usize);
    let file = File::create(path).expect("create png");
    let buf = BufWriter::new(file);
    let mut encoder = png::Encoder::new(buf, w, h);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().expect("png header");
    writer.write_image_data(pixels).expect("png data");
    writer.finish().expect("png finish");
}

fn unique_fixture_path(tag: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    p.push(format!("trajectory_heightmap_{tag}_{stamp}.png"));
    p
}

#[test]
fn from_png_flat_image_produces_expected_grid() {
    let path = unique_fixture_path("flat");
    let pixels = vec![128u8; 16 * 16];
    write_png_grayscale8(&path, 16, 16, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 10.0,
        gamma: 1.0,
        blur_sigma: 0.0,
        invert: false,
        base_pad_mm: 0.0,
    };
    let s = HeightMapSurface::from_png(&path, opts).expect("from_png");

    // 15x15 quads, 2 triangles per quad
    assert_eq!(s.mesh().triangles.len(), 2 * 15 * 15);

    // 128/255 * 10 ≈ 5.0196
    let expected_z = 128.0 / 255.0 * 10.0;
    assert_eq!(s.aabb().min, [0.0, 0.0, expected_z]);
    assert_eq!(s.aabb().max, [15.0, 15.0, expected_z]);

    for tri in &s.mesh().triangles {
        assert!(tri.normal.z > 0.99, "expected ~+Z normal, got {:?}", tri.normal);
    }

    let _ = std::fs::remove_file(&path);
}

#[test]
fn project_down_returns_expected_height() {
    let path = unique_fixture_path("project");
    let pixels = vec![255u8; 8 * 8];
    write_png_grayscale8(&path, 8, 8, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 0.5,
        max_depth_mm: 4.0,
        base_pad_mm: 0.0,
        ..HeightMapOpts::default()
    };
    let s = HeightMapSurface::from_png(&path, opts).expect("from_png");

    // White (1.0) * 4mm = 4mm, full grid is 7*0.5 = 3.5mm wide
    let (p, n) = s.project_down(1.5, 1.5).expect("hit");
    assert!((p.z - 4.0).abs() < 1e-6, "expected z=4, got {}", p.z);
    assert!(n.z > 0.99, "normal not pointing up: {:?}", n);

    let _ = std::fs::remove_file(&path);
}

#[test]
fn gradient_image_yields_monotonic_z_along_x() {
    let path = unique_fixture_path("gradient");
    // Horizontal ramp: column 0 = 0, column 15 = 255
    let mut pixels = Vec::with_capacity(16 * 16);
    for _y in 0..16 {
        for x in 0..16 {
            pixels.push(((x as u32 * 255) / 15) as u8);
        }
    }
    write_png_grayscale8(&path, 16, 16, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 8.0,
        base_pad_mm: 0.0,
        ..HeightMapOpts::default()
    };
    let s = HeightMapSurface::from_png(&path, opts).expect("from_png");

    let mut last = -1.0f64;
    for x in 0..=15 {
        let xf = x as f64;
        let (p, _) = s.project_down(xf, 7.5).expect("hit");
        assert!(p.z >= last - 1e-6, "expected monotonic Z along X: {} -> {}", last, p.z);
        last = p.z;
    }
    // Endpoints anchor: x=0 at Z=0, x=15 at Z=8
    let (p_low, _) = s.project_down(0.0, 7.5).unwrap();
    let (p_high, _) = s.project_down(15.0, 7.5).unwrap();
    assert!(p_low.z.abs() < 1e-6);
    assert!((p_high.z - 8.0).abs() < 1e-6);

    let _ = std::fs::remove_file(&path);
}

#[test]
fn base_pad_extends_aabb_and_is_hittable() {
    let path = unique_fixture_path("pad");
    let pixels = vec![200u8; 8 * 8];
    write_png_grayscale8(&path, 8, 8, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 5.0,
        base_pad_mm: 2.0,
        ..HeightMapOpts::default()
    };
    let s = HeightMapSurface::from_png(&path, opts).expect("from_png");

    assert_eq!(s.aabb().min[0], -2.0);
    assert_eq!(s.aabb().max[0], 9.0);

    // Ray-cast inside the pad ring should hit the z=0 pedestal.
    let hit = s
        .mesh()
        .ray_cast(Point3D::new(-1.0, 3.0, 100.0), Vec3::new(0.0, 0.0, -1.0))
        .expect("pedestal hit");
    assert!(hit.1.z.abs() < 1e-6);

    let _ = std::fs::remove_file(&path);
}

#[test]
fn invert_flag_swaps_high_low() {
    let path = unique_fixture_path("invert");
    let pixels = vec![0u8; 4 * 4];
    write_png_grayscale8(&path, 4, 4, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 3.0,
        invert: true,
        base_pad_mm: 0.0,
        ..HeightMapOpts::default()
    };
    let s = HeightMapSurface::from_png(&path, opts).expect("from_png");
    // Black inverted = full depth
    let (p, _) = s.project_down(1.0, 1.0).unwrap();
    assert!((p.z - 3.0).abs() < 1e-6, "expected z=3 after invert, got {}", p.z);

    let _ = std::fs::remove_file(&path);
}

#[test]
fn zigzag_strategy_consumes_heightmap_surface() {
    let path = unique_fixture_path("zigzag");
    // Small dome: brighter in the center
    let w = 16u32;
    let h = 16u32;
    let mut pixels = Vec::with_capacity((w * h) as usize);
    let cx = w as f64 / 2.0;
    let cy = h as f64 / 2.0;
    let r_max = (cx * cx + cy * cy).sqrt();
    for y in 0..h {
        for x in 0..w {
            let dx = x as f64 + 0.5 - cx;
            let dy = y as f64 + 0.5 - cy;
            let r = (dx * dx + dy * dy).sqrt() / r_max;
            pixels.push(((1.0 - r).max(0.0) * 255.0).round() as u8);
        }
    }
    write_png_grayscale8(&path, w, h, &pixels);

    let opts = HeightMapOpts {
        pixel_size_mm: 1.0,
        max_depth_mm: 3.0,
        base_pad_mm: 1.0,
        ..HeightMapOpts::default()
    };
    let surface = HeightMapSurface::from_png(&path, opts).expect("from_png");
    let tool = Tool::load_json("tests/fixtures/tool_ball6.json").expect("tool");
    let params = ToolpathParams {
        stepover_mm: 2.0,
        direction_deg: 0.0,
        safe_z_mm: 20.0,
        feed_height_mm: 2.0,
        max_segment_mm: 1.0,
        offset_along_axis_mm: 0.0,
    };

    let path_out = ZigZagStrategy.generate(&surface, &tool, &params);
    assert!(!path_out.poses.is_empty(), "zigzag produced no poses");
    let cutting = path_out.feed_flags.iter().filter(|b| **b).count();
    let travel = path_out.feed_flags.iter().filter(|b| !**b).count();
    assert!(cutting > 0, "no cutting moves");
    assert!(travel > 0, "no travel moves");

    let _ = std::fs::remove_file(&path);
}
