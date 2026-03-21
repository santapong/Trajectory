use trajectory::collision::point_collides;
use trajectory::config::RrtConfig;
use trajectory::geometry::{Obstacle, Point2D, Workspace};
use trajectory::path::{export_waypoints_json, path_length, smooth_path};
use trajectory::rrt::RrtPlanner;

fn main() {
    println!("=== Trajectory: CNC Jewelry Pathing ===\n");

    // Set up workspace: 20mm x 20mm work area
    let mut workspace = Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(20.0, 20.0));

    // Add a circular jewelry piece in the center (obstacle to path around)
    workspace.add_obstacle(Obstacle::Circle {
        center: Point2D::new(10.0, 10.0),
        radius: 3.0,
    });

    // Add a small rectangular fixture
    workspace.add_obstacle(Obstacle::Rectangle {
        min: Point2D::new(15.0, 2.0),
        max: Point2D::new(17.0, 4.0),
    });

    let start = Point2D::new(1.0, 10.0);
    let goal = Point2D::new(19.0, 10.0);

    println!("Workspace: 20mm x 20mm");
    println!("Obstacles: circular workpiece (center=10,10 r=3), fixture (15,2)-(17,4)");
    println!("Start: ({}, {})", start.x, start.y);
    println!("Goal:  ({}, {})", goal.x, goal.y);

    // Verify start and goal are not in collision
    assert!(
        !point_collides(&start, &workspace),
        "Start position is in collision!"
    );
    assert!(
        !point_collides(&goal, &workspace),
        "Goal position is in collision!"
    );

    let config = RrtConfig {
        step_size: 0.8,
        max_iterations: 10000,
        goal_bias: 0.1,
        goal_tolerance: 0.8,
        rewire_radius: 3.0,
        smoothing_iterations: 200,
    };

    // --- Basic RRT ---
    println!("\n--- RRT ---");
    let mut planner = RrtPlanner::new(config.clone(), workspace.clone());
    match planner.plan(start, goal) {
        Some(path) => {
            let len = path_length(&path);
            println!("Path found! {} waypoints, length: {:.2}mm", path.len(), len);

            let smoothed = smooth_path(&path, &workspace, config.smoothing_iterations);
            let smooth_len = path_length(&smoothed);
            println!(
                "After smoothing: {} waypoints, length: {:.2}mm",
                smoothed.len(),
                smooth_len
            );
        }
        None => println!("No path found with basic RRT."),
    }

    // --- RRT* ---
    println!("\n--- RRT* ---");
    let mut planner = RrtPlanner::new(config.clone(), workspace.clone());
    match planner.plan_star(start, goal) {
        Some(path) => {
            let len = path_length(&path);
            println!("Path found! {} waypoints, length: {:.2}mm", path.len(), len);

            let smoothed = smooth_path(&path, &workspace, config.smoothing_iterations);
            let smooth_len = path_length(&smoothed);
            println!(
                "After smoothing: {} waypoints, length: {:.2}mm",
                smoothed.len(),
                smooth_len
            );

            println!("\n--- Waypoints (JSON) ---");
            println!("{}", export_waypoints_json(&smoothed));
        }
        None => println!("No path found with RRT*."),
    }
}
