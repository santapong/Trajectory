//! Example: Plan a filing path around a jewelry ring.
//!
//! The ring is modeled as a circular obstacle (the ring body) with the
//! tool needing to navigate around it for filing/polishing operations.

use trajectory::config::RrtConfig;
use trajectory::geometry::{Obstacle, Point2D, Workspace};
use trajectory::path::{path_length, smooth_path};
use trajectory::rrt::RrtPlanner;

fn main() {
    println!("=== Jewelry Ring Filing Path ===\n");

    // Work area: 30mm x 30mm
    let mut workspace = Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(30.0, 30.0));

    // Ring body: circular obstacle at center
    workspace.add_obstacle(Obstacle::Circle {
        center: Point2D::new(15.0, 15.0),
        radius: 5.0,
    });

    // Ring setting/stone mount: small rectangular protrusion
    workspace.add_obstacle(Obstacle::Rectangle {
        min: Point2D::new(13.0, 19.5),
        max: Point2D::new(17.0, 22.0),
    });

    let config = RrtConfig {
        step_size: 0.6,
        max_iterations: 10000,
        goal_bias: 0.08,
        goal_tolerance: 0.6,
        rewire_radius: 3.0,
        smoothing_iterations: 300,
    };

    // Plan multiple filing passes around the ring
    let filing_points = vec![
        (Point2D::new(2.0, 15.0), Point2D::new(28.0, 15.0), "Left to Right"),
        (Point2D::new(15.0, 2.0), Point2D::new(15.0, 28.0), "Bottom to Top"),
        (Point2D::new(2.0, 2.0), Point2D::new(28.0, 28.0), "Diagonal"),
    ];

    for (start, goal, name) in &filing_points {
        println!("--- Filing pass: {} ---", name);
        println!("  Start: ({:.1}, {:.1})", start.x, start.y);
        println!("  Goal:  ({:.1}, {:.1})", goal.x, goal.y);

        // Compare RRT vs RRT*
        let mut planner_rrt = RrtPlanner::new(config.clone(), workspace.clone());
        let mut planner_star = RrtPlanner::new(config.clone(), workspace.clone());

        let rrt_result = planner_rrt.plan(*start, *goal);
        let star_result = planner_star.plan_star(*start, *goal);

        if let Some(path) = rrt_result {
            let smoothed = smooth_path(&path, &workspace, config.smoothing_iterations);
            println!(
                "  RRT:  {} pts -> {} smoothed, length: {:.2}mm -> {:.2}mm",
                path.len(),
                smoothed.len(),
                path_length(&path),
                path_length(&smoothed)
            );
        } else {
            println!("  RRT:  No path found");
        }

        if let Some(path) = star_result {
            let smoothed = smooth_path(&path, &workspace, config.smoothing_iterations);
            println!(
                "  RRT*: {} pts -> {} smoothed, length: {:.2}mm -> {:.2}mm",
                path.len(),
                smoothed.len(),
                path_length(&path),
                path_length(&smoothed)
            );
        } else {
            println!("  RRT*: No path found");
        }

        println!();
    }
}
