//! Heightmap surface from a grayscale PNG.
//!
//! Pixel intensity is mapped to Z, the regular pixel grid is triangulated into
//! a [`Mesh`], and the surface plugs into the existing [`Surface`] trait so
//! `ZigZagStrategy` and `AbbRapidPost` consume it unchanged.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use serde::{Deserialize, Serialize};
use tracing::{debug, info_span};

use crate::math3d::Point3D;

use super::{Mesh, Surface, SurfaceError, Triangle};

/// Options for [`HeightMapSurface::from_png`].
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HeightMapOpts {
    /// Real-world XY size of one pixel, in millimeters.
    pub pixel_size_mm: f64,
    /// Maximum carved depth in millimeters (white pixel by default → this Z).
    pub max_depth_mm: f64,
    /// Gamma curve applied to the normalized intensity before scaling
    /// (`p' = p^gamma`). 1.0 = linear, < 1.0 boosts midtones for portrait relief.
    pub gamma: f64,
    /// Gaussian blur sigma in pixels (0.0 disables). Suppresses stair-step
    /// artifacts from 8-bit quantization.
    pub blur_sigma: f64,
    /// If true, black=high / white=low. Default false: white=high / black=low.
    pub invert: bool,
    /// Width of a flat z=0 perimeter added around the heightmap so edge
    /// ray-casts always hit and the relief sits on a pedestal. 0.0 disables.
    pub base_pad_mm: f64,
}

impl Default for HeightMapOpts {
    fn default() -> Self {
        Self {
            pixel_size_mm: 0.1,
            max_depth_mm: 5.0,
            gamma: 1.0,
            blur_sigma: 0.0,
            invert: false,
            base_pad_mm: 1.0,
        }
    }
}

impl HeightMapOpts {
    /// Reject obviously unsafe values before any pixel work happens.
    /// Bounds are tuned to the project's stated domain (CNC jewelry, mm-scale).
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if !self.pixel_size_mm.is_finite() || self.pixel_size_mm <= 0.0 || self.pixel_size_mm > 10.0 {
            return Err(SurfaceError::InvalidMesh(format!(
                "pixel_size_mm must be in (0, 10] mm, got {}",
                self.pixel_size_mm
            )));
        }
        if !self.max_depth_mm.is_finite() || self.max_depth_mm <= 0.0 || self.max_depth_mm > 50.0 {
            return Err(SurfaceError::InvalidMesh(format!(
                "max_depth_mm must be in (0, 50] mm, got {}",
                self.max_depth_mm
            )));
        }
        if !self.gamma.is_finite() || self.gamma < 0.1 || self.gamma > 5.0 {
            return Err(SurfaceError::InvalidMesh(format!(
                "gamma must be in [0.1, 5.0], got {}",
                self.gamma
            )));
        }
        if !self.blur_sigma.is_finite() || self.blur_sigma < 0.0 || self.blur_sigma > 10.0 {
            return Err(SurfaceError::InvalidMesh(format!(
                "blur_sigma must be in [0, 10] px, got {}",
                self.blur_sigma
            )));
        }
        if !self.base_pad_mm.is_finite() || self.base_pad_mm < 0.0 || self.base_pad_mm > 50.0 {
            return Err(SurfaceError::InvalidMesh(format!(
                "base_pad_mm must be in [0, 50] mm, got {}",
                self.base_pad_mm
            )));
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct HeightMapSurface {
    mesh: Mesh,
}

impl HeightMapSurface {
    /// Decode a PNG (8 or 16-bit, grayscale or RGB/RGBA) and build a relief
    /// surface. RGB inputs are converted to luminance via Rec.601:
    /// `0.299·R + 0.587·G + 0.114·B`.
    pub fn from_png<P: AsRef<Path>>(path: P, opts: HeightMapOpts) -> Result<Self, SurfaceError> {
        opts.validate()?;
        let span = info_span!("heightmap.from_png", path = %path.as_ref().display());
        let _enter = span.enter();

        let (buf, info) = {
            let _s = info_span!("decode").entered();
            let file = File::open(path)?;
            let decoder = png::Decoder::new(BufReader::new(file));
            let mut reader = decoder
                .read_info()
                .map_err(|e| SurfaceError::InvalidMesh(format!("png: {e}")))?;
            let info = reader.info().clone();
            let mut buf = vec![0u8; reader.output_buffer_size()];
            let frame = reader
                .next_frame(&mut buf)
                .map_err(|e| SurfaceError::InvalidMesh(format!("png frame: {e}")))?;
            buf.truncate(frame.buffer_size());
            (buf, info)
        };

        let w = info.width as usize;
        let h = info.height as usize;
        if w < 2 || h < 2 {
            return Err(SurfaceError::InvalidMesh(format!(
                "heightmap must be at least 2x2 pixels, got {w}x{h}"
            )));
        }
        debug!(width = w, height = h, color = ?info.color_type, depth = ?info.bit_depth, "decoded png");

        let intensity = {
            let _s = info_span!("normalize").entered();
            decode_to_normalized(&buf, w, h, info.color_type, info.bit_depth)?
        };
        let intensity = if opts.blur_sigma > 0.0 {
            let _s = info_span!("blur", sigma = opts.blur_sigma).entered();
            gaussian_blur(&intensity, w, h, opts.blur_sigma)
        } else {
            intensity
        };

        let z = {
            let _s = info_span!("intensity_to_z").entered();
            intensity_to_z(&intensity, &opts)
        };
        let mesh = {
            let _s = info_span!("triangulate", w, h).entered();
            build_mesh(&z, w, h, &opts)?
        };
        debug!(triangles = mesh.triangles.len(), "mesh built");
        Ok(Self { mesh })
    }

    /// Construct directly from a row-major Z grid (mm). Mostly useful for tests.
    pub fn from_grid(z_mm: &[f64], width: usize, height: usize, opts: HeightMapOpts) -> Result<Self, SurfaceError> {
        opts.validate()?;
        if z_mm.len() != width * height {
            return Err(SurfaceError::InvalidMesh(format!(
                "z_mm length {} != width*height {}",
                z_mm.len(),
                width * height
            )));
        }
        if width < 2 || height < 2 {
            return Err(SurfaceError::InvalidMesh(format!(
                "heightmap must be at least 2x2, got {width}x{height}"
            )));
        }
        let mesh = build_mesh(z_mm, width, height, &opts)?;
        Ok(Self { mesh })
    }
}

impl Surface for HeightMapSurface {
    fn mesh(&self) -> &Mesh {
        &self.mesh
    }
}

fn decode_to_normalized(
    buf: &[u8],
    w: usize,
    h: usize,
    color: png::ColorType,
    depth: png::BitDepth,
) -> Result<Vec<f64>, SurfaceError> {
    let pixels = w * h;
    let mut out = vec![0.0f64; pixels];

    match (color, depth) {
        (png::ColorType::Grayscale, png::BitDepth::Eight) => {
            for i in 0..pixels {
                out[i] = buf[i] as f64 / 255.0;
            }
        }
        (png::ColorType::GrayscaleAlpha, png::BitDepth::Eight) => {
            for i in 0..pixels {
                out[i] = buf[i * 2] as f64 / 255.0;
            }
        }
        (png::ColorType::Rgb, png::BitDepth::Eight) => {
            for i in 0..pixels {
                let r = buf[i * 3] as f64;
                let g = buf[i * 3 + 1] as f64;
                let b = buf[i * 3 + 2] as f64;
                out[i] = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0;
            }
        }
        (png::ColorType::Rgba, png::BitDepth::Eight) => {
            for i in 0..pixels {
                let r = buf[i * 4] as f64;
                let g = buf[i * 4 + 1] as f64;
                let b = buf[i * 4 + 2] as f64;
                out[i] = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0;
            }
        }
        (png::ColorType::Grayscale, png::BitDepth::Sixteen) => {
            for i in 0..pixels {
                let hi = buf[i * 2] as u16;
                let lo = buf[i * 2 + 1] as u16;
                out[i] = ((hi << 8) | lo) as f64 / 65535.0;
            }
        }
        (png::ColorType::Rgb, png::BitDepth::Sixteen) => {
            for i in 0..pixels {
                let off = i * 6;
                let r = (((buf[off] as u16) << 8) | buf[off + 1] as u16) as f64;
                let g = (((buf[off + 2] as u16) << 8) | buf[off + 3] as u16) as f64;
                let b = (((buf[off + 4] as u16) << 8) | buf[off + 5] as u16) as f64;
                out[i] = (0.299 * r + 0.587 * g + 0.114 * b) / 65535.0;
            }
        }
        other => {
            return Err(SurfaceError::InvalidMesh(format!(
                "unsupported PNG color/depth combo: {:?}",
                other
            )));
        }
    }
    Ok(out)
}

fn intensity_to_z(intensity: &[f64], opts: &HeightMapOpts) -> Vec<f64> {
    let gamma = opts.gamma.max(1e-6);
    intensity
        .iter()
        .map(|&p| {
            let p = p.clamp(0.0, 1.0);
            let curved = if (gamma - 1.0).abs() < 1e-9 { p } else { p.powf(gamma) };
            let v = if opts.invert { 1.0 - curved } else { curved };
            v * opts.max_depth_mm
        })
        .collect()
}

fn gaussian_blur(src: &[f64], w: usize, h: usize, sigma: f64) -> Vec<f64> {
    let radius = (sigma * 3.0).ceil() as isize;
    let two_sigma_sq = 2.0 * sigma * sigma;
    let kernel: Vec<f64> = (-radius..=radius)
        .map(|i| (-((i * i) as f64) / two_sigma_sq).exp())
        .collect();
    let sum: f64 = kernel.iter().sum();
    let kernel: Vec<f64> = kernel.iter().map(|k| k / sum).collect();

    let mut tmp = vec![0.0f64; w * h];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (k_idx, &k) in kernel.iter().enumerate() {
                let dx = k_idx as isize - radius;
                let xi = (x as isize + dx).clamp(0, w as isize - 1) as usize;
                acc += k * src[y * w + xi];
            }
            tmp[y * w + x] = acc;
        }
    }
    let mut out = vec![0.0f64; w * h];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (k_idx, &k) in kernel.iter().enumerate() {
                let dy = k_idx as isize - radius;
                let yi = (y as isize + dy).clamp(0, h as isize - 1) as usize;
                acc += k * tmp[yi * w + x];
            }
            out[y * w + x] = acc;
        }
    }
    out
}

fn build_mesh(z: &[f64], w: usize, h: usize, opts: &HeightMapOpts) -> Result<Mesh, SurfaceError> {
    let px = opts.pixel_size_mm;
    if px <= 0.0 {
        return Err(SurfaceError::InvalidMesh(format!(
            "pixel_size_mm must be > 0, got {px}"
        )));
    }

    let pad = opts.base_pad_mm.max(0.0);
    let x0 = -pad;
    let y0 = -pad;
    let x_max = (w as f64 - 1.0) * px + pad;
    let y_max = (h as f64 - 1.0) * px + pad;

    let mut tris: Vec<Triangle> =
        Vec::with_capacity(2 * (w - 1) * (h - 1) + if pad > 0.0 { 8 } else { 0 });

    for j in 0..h - 1 {
        for i in 0..w - 1 {
            let p00 = Point3D::new(i as f64 * px, j as f64 * px, z[j * w + i]);
            let p10 = Point3D::new((i + 1) as f64 * px, j as f64 * px, z[j * w + i + 1]);
            let p01 = Point3D::new(i as f64 * px, (j + 1) as f64 * px, z[(j + 1) * w + i]);
            let p11 = Point3D::new((i + 1) as f64 * px, (j + 1) as f64 * px, z[(j + 1) * w + i + 1]);
            push_quad_ccw(&mut tris, p00, p10, p11, p01);
        }
    }

    if pad > 0.0 {
        let surf_xmax = (w as f64 - 1.0) * px;
        let surf_ymax = (h as f64 - 1.0) * px;

        let c00 = Point3D::new(x0, y0, 0.0);
        let c10 = Point3D::new(x_max, y0, 0.0);
        let c11 = Point3D::new(x_max, y_max, 0.0);
        let c01 = Point3D::new(x0, y_max, 0.0);

        let s00 = Point3D::new(0.0, 0.0, 0.0);
        let s10 = Point3D::new(surf_xmax, 0.0, 0.0);
        let s11 = Point3D::new(surf_xmax, surf_ymax, 0.0);
        let s01 = Point3D::new(0.0, surf_ymax, 0.0);

        push_quad_ccw(&mut tris, c00, c10, s10, s00);
        push_quad_ccw(&mut tris, s10, c10, c11, s11);
        push_quad_ccw(&mut tris, s01, s11, c11, c01);
        push_quad_ccw(&mut tris, c00, s00, s01, c01);
    }

    if tris.is_empty() {
        return Err(SurfaceError::InvalidMesh("heightmap produced no triangles".into()));
    }
    Ok(Mesh::from_triangles(tris))
}

fn push_quad_ccw(tris: &mut Vec<Triangle>, p00: Point3D, p10: Point3D, p11: Point3D, p01: Point3D) {
    tris.push(Triangle::new([p00, p10, p11], None));
    tris.push(Triangle::new([p00, p11, p01], None));
}
