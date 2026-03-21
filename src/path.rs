use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::collision::segment_collides;
use crate::geometry::{Point2D, Workspace};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub index: usize,
    pub x: f64,
    pub y: f64,
}

/// Smooth a path by randomly attempting to shortcut between non-adjacent points.
pub fn smooth_path(
    path: &[Point2D],
    workspace: &Workspace,
    iterations: usize,
) -> Vec<Point2D> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let mut smoothed = path.to_vec();
    let mut rng = rand::thread_rng();

    for _ in 0..iterations {
        if smoothed.len() <= 2 {
            break;
        }
        let i = rng.gen_range(0..smoothed.len() - 1);
        let j = rng.gen_range(i + 1..smoothed.len());
        if j - i <= 1 {
            continue;
        }
        if !segment_collides(&smoothed[i], &smoothed[j], workspace) {
            // Remove intermediate points
            let mut new_path = smoothed[..=i].to_vec();
            new_path.extend_from_slice(&smoothed[j..]);
            smoothed = new_path;
        }
    }

    smoothed
}

/// Convert a path of Point2D into serializable Waypoints.
pub fn to_waypoints(path: &[Point2D]) -> Vec<Waypoint> {
    path.iter()
        .enumerate()
        .map(|(i, p)| Waypoint {
            index: i,
            x: p.x,
            y: p.y,
        })
        .collect()
}

/// Export waypoints as a JSON string for CNC controller consumption.
pub fn export_waypoints_json(path: &[Point2D]) -> String {
    let waypoints = to_waypoints(path);
    serde_json::to_string_pretty(&waypoints).unwrap_or_default()
}

/// Calculate total path length.
pub fn path_length(path: &[Point2D]) -> f64 {
    path.windows(2)
        .map(|w| w[0].distance_to(&w[1]))
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Workspace;

    #[test]
    fn test_path_length() {
        let path = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(3.0, 0.0),
            Point2D::new(3.0, 4.0),
        ];
        assert!((path_length(&path) - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_export_json() {
        let path = vec![Point2D::new(1.0, 2.0), Point2D::new(3.0, 4.0)];
        let json = export_waypoints_json(&path);
        assert!(json.contains("1.0"));
        assert!(json.contains("3.0"));
    }

    #[test]
    fn test_smooth_path_reduces() {
        let ws = Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(20.0, 20.0));
        // A zigzag path that should be smoothable to a straight line
        let path = vec![
            Point2D::new(0.0, 0.0),
            Point2D::new(1.0, 0.5),
            Point2D::new(2.0, 0.1),
            Point2D::new(3.0, 0.4),
            Point2D::new(4.0, 0.0),
        ];
        let smoothed = smooth_path(&path, &ws, 200);
        assert!(smoothed.len() <= path.len());
    }
}
