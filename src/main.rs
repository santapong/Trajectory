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

    #[cfg(all(feature = "surface", feature = "abb"))]
    surface_demo();
}

#[cfg(all(feature = "surface", feature = "abb"))]
fn surface_demo() {
    use trajectory::frame::WorkpieceFrame;
    use trajectory::post::{AbbRapidPost, PostContext, PostProcessor};
    use trajectory::surface::StlSurface;
    use trajectory::tool::Tool;
    use trajectory::toolpath::{ToolPathStrategy, ToolpathParams, ZigZagStrategy};

    println!("\n=== 3D Surface zig-zag -> ABB RAPID (mini demo) ===");
    let stl_path = "tests/fixtures/flat_plane.stl";
    let surface = match StlSurface::load(stl_path) {
        Ok(s) => s,
        Err(e) => {
            println!("(skipping 3D demo: {} not loadable: {})", stl_path, e);
            return;
        }
    };
    let tool = match Tool::load_json("tests/fixtures/tool_ball6.json") {
        Ok(t) => t,
        Err(_) => return,
    };
    let frame = WorkpieceFrame::identity();
    let params = ToolpathParams {
        stepover_mm: 20.0,
        direction_deg: 0.0,
        safe_z_mm: 50.0,
        feed_height_mm: 5.0,
        max_segment_mm: 20.0,
        offset_along_axis_mm: 0.0,
    };
    let toolpath = ZigZagStrategy.generate(&surface, &tool, &params);

    let ctx = PostContext {
        program_name: "ZigZagDemo",
        tool: &tool,
        frame: &frame,
        feedrate_mm_min: 600.0,
        rapid_speed: 5000.0,
    };
    let rapid = AbbRapidPost.emit(&toolpath, &ctx).expect("post");
    println!("--- First 10 lines of RAPID ---");
    for line in rapid.lines().take(10) {
        println!("{line}");
    }
    println!("... ({} total lines, {} poses)", rapid.lines().count(), toolpath.poses.len());
}
