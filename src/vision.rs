//! Real-time vision module using OpenCV for workpiece detection.
//!
//! This module is only available when the `vision` feature is enabled.
//! It captures frames from a camera, detects workpiece contours, and
//! converts them into obstacles for the RRT planner.

use crate::config::RrtConfig;
use crate::geometry::{Obstacle, Point2D, Workspace};
use crate::path::{export_waypoints_json, smooth_path};
use crate::rrt::RrtPlanner;

use opencv::core::{self, Mat, Point, Scalar, Size, Vector};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::videoio::{self, VideoCapture, VideoCaptureAPIs};

/// Calibration parameters for converting pixel coordinates to world coordinates.
#[derive(Debug, Clone)]
pub struct Calibration {
    /// Pixels per millimeter in X direction.
    pub pixels_per_mm_x: f64,
    /// Pixels per millimeter in Y direction.
    pub pixels_per_mm_y: f64,
    /// Origin offset in pixels (top-left of workspace in image).
    pub origin_x: f64,
    pub origin_y: f64,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            pixels_per_mm_x: 10.0,
            pixels_per_mm_y: 10.0,
            origin_x: 0.0,
            origin_y: 0.0,
        }
    }
}

impl Calibration {
    /// Convert pixel coordinates to world (mm) coordinates.
    pub fn pixel_to_world(&self, px: f64, py: f64) -> Point2D {
        Point2D::new(
            (px - self.origin_x) / self.pixels_per_mm_x,
            (py - self.origin_y) / self.pixels_per_mm_y,
        )
    }
}

/// Camera source wrapping OpenCV VideoCapture.
pub struct CameraSource {
    capture: VideoCapture,
}

impl CameraSource {
    /// Open the default camera (index 0).
    pub fn open_default() -> Result<Self, opencv::Error> {
        let capture = VideoCapture::new(0, VideoCaptureAPIs::CAP_ANY as i32)?;
        if !capture.is_opened()? {
            return Err(opencv::Error::new(
                core::StsError,
                "Failed to open camera",
            ));
        }
        Ok(Self { capture })
    }

    /// Open a camera by index.
    pub fn open(index: i32) -> Result<Self, opencv::Error> {
        let capture = VideoCapture::new(index, VideoCaptureAPIs::CAP_ANY as i32)?;
        if !capture.is_opened()? {
            return Err(opencv::Error::new(
                core::StsError,
                "Failed to open camera",
            ));
        }
        Ok(Self { capture })
    }

    /// Capture a single frame.
    pub fn capture_frame(&mut self) -> Result<Mat, opencv::Error> {
        let mut frame = Mat::default();
        self.capture.read(&mut frame)?;
        Ok(frame)
    }
}

/// Detect workpiece obstacles from a camera frame.
pub fn detect_workpiece(
    frame: &Mat,
    calibration: &Calibration,
    min_contour_area: f64,
) -> Result<Vec<Obstacle>, opencv::Error> {
    // Convert to grayscale
    let mut gray = Mat::default();
    imgproc::cvt_color(frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

    // Apply Gaussian blur to reduce noise
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &gray,
        &mut blurred,
        Size::new(5, 5),
        1.5,
        1.5,
        core::BORDER_DEFAULT,
    )?;

    // Canny edge detection
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 50.0, 150.0, 3, false)?;

    // Find contours
    let mut contours: Vector<Vector<Point>> = Vector::new();
    imgproc::find_contours(
        &edges,
        &mut contours,
        imgproc::RETR_EXTERNAL,
        imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )?;

    let mut obstacles = Vec::new();

    for contour in contours.iter() {
        let area = imgproc::contour_area(&contour, false)?;
        if area < min_contour_area {
            continue;
        }

        // Approximate contour to polygon
        let mut approx = Vector::<Point>::new();
        let epsilon = 0.02 * imgproc::arc_length(&contour, true)?;
        imgproc::approx_poly_dp(&contour, &mut approx, epsilon, true)?;

        let points: Vec<Point2D> = approx
            .iter()
            .map(|p| calibration.pixel_to_world(p.x as f64, p.y as f64))
            .collect();

        // If contour approximates to a small number of points and is roughly circular,
        // fit a minimum enclosing circle
        if approx.len() >= 6 {
            let mut center = core::Point2f::new(0.0, 0.0);
            let mut radius = 0.0f32;
            imgproc::min_enclosing_circle(&contour, &mut center, &mut radius)?;

            let circularity = 4.0 * std::f64::consts::PI * area
                / (imgproc::arc_length(&contour, true)?).powi(2);

            if circularity > 0.8 {
                // Close enough to a circle
                let world_center =
                    calibration.pixel_to_world(center.x as f64, center.y as f64);
                let world_radius = radius as f64 / calibration.pixels_per_mm_x;
                obstacles.push(Obstacle::Circle {
                    center: world_center,
                    radius: world_radius,
                });
                continue;
            }
        }

        // Otherwise use polygon
        if points.len() >= 3 {
            obstacles.push(Obstacle::Polygon { vertices: points });
        }
    }

    Ok(obstacles)
}

/// Real-time planner that continuously captures frames and replans.
pub struct RealTimePlanner {
    pub camera: CameraSource,
    pub calibration: Calibration,
    pub config: RrtConfig,
    pub workspace_bounds: (Point2D, Point2D),
    pub start: Point2D,
    pub goal: Point2D,
    pub min_contour_area: f64,
}

impl RealTimePlanner {
    /// Run a single planning cycle: capture -> detect -> plan -> output.
    pub fn plan_cycle(&mut self) -> Result<Option<String>, opencv::Error> {
        let frame = self.camera.capture_frame()?;

        let obstacles =
            detect_workpiece(&frame, &self.calibration, self.min_contour_area)?;

        let mut workspace =
            Workspace::new(self.workspace_bounds.0, self.workspace_bounds.1);
        for obs in obstacles {
            workspace.add_obstacle(obs);
        }

        let mut planner = RrtPlanner::new(self.config.clone(), workspace.clone());
        match planner.plan_star(self.start, self.goal) {
            Some(path) => {
                let smoothed =
                    smooth_path(&path, &workspace, self.config.smoothing_iterations);
                Ok(Some(export_waypoints_json(&smoothed)))
            }
            None => Ok(None),
        }
    }

    /// Run continuous planning loop. Calls the callback with each planned path.
    pub fn run_loop<F>(&mut self, mut on_path: F) -> Result<(), opencv::Error>
    where
        F: FnMut(Option<&str>),
    {
        loop {
            match self.plan_cycle()? {
                Some(json) => on_path(Some(&json)),
                None => on_path(None),
            }
        }
    }
}
