use trajectory::collision::{point_collides, segment_collides};
use trajectory::config::RrtConfig;
use trajectory::geometry::{Obstacle, Point2D, Workspace};
use trajectory::path::{path_length, smooth_path};
use trajectory::rrt::RrtPlanner;

fn default_config() -> RrtConfig {
    RrtConfig {
        step_size: 1.0,
        max_iterations: 10000,
        goal_bias: 0.1,
        goal_tolerance: 1.0,
        rewire_radius: 3.0,
        smoothing_iterations: 100,
    }
}

fn open_workspace() -> Workspace {
    Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(20.0, 20.0))
}

fn workspace_with_circle() -> Workspace {
    let mut ws = open_workspace();
    ws.add_obstacle(Obstacle::Circle {
        center: Point2D::new(10.0, 10.0),
        radius: 3.0,
    });
    ws
}

#[test]
fn test_rrt_finds_path_open_space() {
    let mut planner = RrtPlanner::new(default_config(), open_workspace());
    let path = planner.plan(Point2D::new(1.0, 1.0), Point2D::new(19.0, 19.0));
    assert!(path.is_some(), "RRT should find path in open space");
}

#[test]
fn test_rrt_star_finds_path_open_space() {
    let mut planner = RrtPlanner::new(default_config(), open_workspace());
    let path = planner.plan_star(Point2D::new(1.0, 1.0), Point2D::new(19.0, 19.0));
    assert!(path.is_some(), "RRT* should find path in open space");
}

#[test]
fn test_rrt_navigates_around_obstacle() {
    let ws = workspace_with_circle();
    let mut planner = RrtPlanner::new(default_config(), ws.clone());
    let path = planner
        .plan(Point2D::new(1.0, 10.0), Point2D::new(19.0, 10.0))
        .expect("Should find path around obstacle");

    // Verify no point in the path is inside the obstacle
    for point in &path {
        assert!(
            !ws.obstacles[0].contains_point(point),
            "Path point ({}, {}) is inside obstacle",
            point.x,
            point.y
        );
    }
}

#[test]
fn test_rrt_star_navigates_around_obstacle() {
    let ws = workspace_with_circle();
    let mut planner = RrtPlanner::new(default_config(), ws.clone());
    let path = planner
        .plan_star(Point2D::new(1.0, 10.0), Point2D::new(19.0, 10.0))
        .expect("RRT* should find path around obstacle");

    for point in &path {
        assert!(
            !ws.obstacles[0].contains_point(point),
            "Path point ({}, {}) is inside obstacle",
            point.x,
            point.y
        );
    }
}

#[test]
fn test_path_starts_and_ends_correctly() {
    let mut planner = RrtPlanner::new(default_config(), open_workspace());
    let start = Point2D::new(1.0, 1.0);
    let goal = Point2D::new(19.0, 19.0);
    let path = planner.plan(start, goal).expect("Should find path");

    assert_eq!(path[0].x, start.x);
    assert_eq!(path[0].y, start.y);

    let last = path.last().unwrap();
    assert!(last.distance_to(&goal) <= default_config().goal_tolerance);
}

#[test]
fn test_smoothing_does_not_increase_length() {
    let ws = workspace_with_circle();
    let mut planner = RrtPlanner::new(default_config(), ws.clone());
    let path = planner
        .plan(Point2D::new(1.0, 10.0), Point2D::new(19.0, 10.0))
        .expect("Should find path");

    let original_len = path_length(&path);
    let smoothed = smooth_path(&path, &ws, 200);
    let smooth_len = path_length(&smoothed);

    assert!(
        smooth_len <= original_len + 0.01,
        "Smoothed path ({:.2}) should not be longer than original ({:.2})",
        smooth_len,
        original_len
    );
}

#[test]
fn test_collision_detection_circle() {
    let ws = workspace_with_circle();
    assert!(point_collides(&Point2D::new(10.0, 10.0), &ws));
    assert!(!point_collides(&Point2D::new(1.0, 1.0), &ws));
}

#[test]
fn test_collision_detection_out_of_bounds() {
    let ws = open_workspace();
    assert!(point_collides(&Point2D::new(-1.0, 5.0), &ws));
    assert!(point_collides(&Point2D::new(5.0, 25.0), &ws));
}

#[test]
fn test_segment_collision() {
    let ws = workspace_with_circle();
    // Segment through the circle
    assert!(segment_collides(
        &Point2D::new(5.0, 10.0),
        &Point2D::new(15.0, 10.0),
        &ws
    ));
    // Segment that avoids the circle
    assert!(!segment_collides(
        &Point2D::new(1.0, 1.0),
        &Point2D::new(1.0, 19.0),
        &ws
    ));
}

#[test]
fn test_polygon_obstacle() {
    let mut ws = open_workspace();
    ws.add_obstacle(Obstacle::Polygon {
        vertices: vec![
            Point2D::new(8.0, 8.0),
            Point2D::new(12.0, 8.0),
            Point2D::new(12.0, 12.0),
            Point2D::new(8.0, 12.0),
        ],
    });
    assert!(point_collides(&Point2D::new(10.0, 10.0), &ws));
    assert!(!point_collides(&Point2D::new(5.0, 5.0), &ws));
}
