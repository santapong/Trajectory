use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: &Point2D) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn interpolate(&self, other: &Point2D, t: f64) -> Point2D {
        Point2D {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }

    /// Steer from self toward target, limited by max_distance.
    pub fn steer_toward(&self, target: &Point2D, max_distance: f64) -> Point2D {
        let dist = self.distance_to(target);
        if dist <= max_distance {
            *target
        } else {
            self.interpolate(target, max_distance / dist)
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Obstacle {
    Circle {
        center: Point2D,
        radius: f64,
    },
    Rectangle {
        min: Point2D,
        max: Point2D,
    },
    Polygon {
        vertices: Vec<Point2D>,
    },
}

impl Obstacle {
    pub fn contains_point(&self, point: &Point2D) -> bool {
        match self {
            Obstacle::Circle { center, radius } => center.distance_to(point) <= *radius,
            Obstacle::Rectangle { min, max } => {
                point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y
            }
            Obstacle::Polygon { vertices } => point_in_polygon(point, vertices),
        }
    }
}

/// Ray-casting algorithm for point-in-polygon test.
fn point_in_polygon(point: &Point2D, vertices: &[Point2D]) -> bool {
    let n = vertices.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let vi = &vertices[i];
        let vj = &vertices[j];
        if ((vi.y > point.y) != (vj.y > point.y))
            && (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Workspace {
    pub bounds_min: Point2D,
    pub bounds_max: Point2D,
    pub obstacles: Vec<Obstacle>,
}

impl Workspace {
    pub fn new(bounds_min: Point2D, bounds_max: Point2D) -> Self {
        Self {
            bounds_min,
            bounds_max,
            obstacles: Vec::new(),
        }
    }

    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        self.obstacles.push(obstacle);
    }

    pub fn is_within_bounds(&self, point: &Point2D) -> bool {
        point.x >= self.bounds_min.x
            && point.x <= self.bounds_max.x
            && point.y >= self.bounds_min.y
            && point.y <= self.bounds_max.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_distance() {
        let a = Point2D::new(0.0, 0.0);
        let b = Point2D::new(3.0, 4.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_circle_contains() {
        let circle = Obstacle::Circle {
            center: Point2D::new(5.0, 5.0),
            radius: 2.0,
        };
        assert!(circle.contains_point(&Point2D::new(5.0, 5.0)));
        assert!(circle.contains_point(&Point2D::new(6.0, 5.0)));
        assert!(!circle.contains_point(&Point2D::new(8.0, 5.0)));
    }

    #[test]
    fn test_rectangle_contains() {
        let rect = Obstacle::Rectangle {
            min: Point2D::new(0.0, 0.0),
            max: Point2D::new(4.0, 4.0),
        };
        assert!(rect.contains_point(&Point2D::new(2.0, 2.0)));
        assert!(!rect.contains_point(&Point2D::new(5.0, 5.0)));
    }

    #[test]
    fn test_polygon_contains() {
        let poly = Obstacle::Polygon {
            vertices: vec![
                Point2D::new(0.0, 0.0),
                Point2D::new(4.0, 0.0),
                Point2D::new(4.0, 4.0),
                Point2D::new(0.0, 4.0),
            ],
        };
        assert!(poly.contains_point(&Point2D::new(2.0, 2.0)));
        assert!(!poly.contains_point(&Point2D::new(5.0, 5.0)));
    }

    #[test]
    fn test_steer_toward() {
        let a = Point2D::new(0.0, 0.0);
        let b = Point2D::new(10.0, 0.0);
        let steered = a.steer_toward(&b, 3.0);
        assert!((steered.x - 3.0).abs() < 1e-10);
        assert!((steered.y - 0.0).abs() < 1e-10);
    }
}
