use crate::geometry::{Obstacle, Point2D, Workspace};

/// Check if a point collides with any obstacle or is out of bounds.
pub fn point_collides(point: &Point2D, workspace: &Workspace) -> bool {
    if !workspace.is_within_bounds(point) {
        return true;
    }
    workspace
        .obstacles
        .iter()
        .any(|obs| obs.contains_point(point))
}

/// Check if a line segment from p1 to p2 collides with any obstacle.
/// Uses discrete sampling along the segment.
pub fn segment_collides(p1: &Point2D, p2: &Point2D, workspace: &Workspace) -> bool {
    let dist = p1.distance_to(p2);
    let check_resolution = 0.1;
    let steps = (dist / check_resolution).ceil() as usize;
    if steps == 0 {
        return point_collides(p1, workspace);
    }
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let sample = p1.interpolate(p2, t);
        if point_collides(&sample, workspace) {
            return true;
        }
    }
    false
}

/// Check if a segment intersects a specific obstacle (used internally).
pub fn segment_intersects_obstacle(p1: &Point2D, p2: &Point2D, obstacle: &Obstacle) -> bool {
    let dist = p1.distance_to(p2);
    let steps = (dist / 0.1).ceil() as usize;
    if steps == 0 {
        return obstacle.contains_point(p1);
    }
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let sample = p1.interpolate(p2, t);
        if obstacle.contains_point(&sample) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Obstacle;

    fn test_workspace() -> Workspace {
        let mut ws = Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(20.0, 20.0));
        ws.add_obstacle(Obstacle::Circle {
            center: Point2D::new(10.0, 10.0),
            radius: 3.0,
        });
        ws
    }

    #[test]
    fn test_point_outside_bounds() {
        let ws = test_workspace();
        assert!(point_collides(&Point2D::new(-1.0, 5.0), &ws));
    }

    #[test]
    fn test_point_in_obstacle() {
        let ws = test_workspace();
        assert!(point_collides(&Point2D::new(10.0, 10.0), &ws));
    }

    #[test]
    fn test_point_free() {
        let ws = test_workspace();
        assert!(!point_collides(&Point2D::new(1.0, 1.0), &ws));
    }

    #[test]
    fn test_segment_through_obstacle() {
        let ws = test_workspace();
        assert!(segment_collides(
            &Point2D::new(5.0, 10.0),
            &Point2D::new(15.0, 10.0),
            &ws
        ));
    }

    #[test]
    fn test_segment_clear() {
        let ws = test_workspace();
        assert!(!segment_collides(
            &Point2D::new(1.0, 1.0),
            &Point2D::new(1.0, 19.0),
            &ws
        ));
    }
}
