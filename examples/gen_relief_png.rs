//! Regenerate the demo heightmap fixture used by `surface_heightmap_zigzag`.
//!
//! Run with:
//!   cargo run --example gen_relief_png --features heightmap

use std::fs::File;
use std::io::BufWriter;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let w = 64u32;
    let h = 64u32;
    let mut buf = Vec::with_capacity((w * h) as usize);
    let cx = w as f64 / 2.0;
    let cy = h as f64 / 2.0;
    let r_max = (cx * cx + cy * cy).sqrt();
    for y in 0..h {
        for x in 0..w {
            let dx = x as f64 + 0.5 - cx;
            let dy = y as f64 + 0.5 - cy;
            let r = (dx * dx + dy * dy).sqrt() / r_max;
            let dome = (1.0 - r).max(0.0);
            let v = (dome * 255.0).round() as u8;
            buf.push(v);
        }
    }

    let path = "tests/fixtures/relief_ramp.png";
    let file = File::create(path)?;
    let w_buf = BufWriter::new(file);
    let mut encoder = png::Encoder::new(w_buf, w, h);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(&buf)?;
    writer.finish()?;
    println!("wrote {path} ({w}x{h} grayscale)");
    Ok(())
}
