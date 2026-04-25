#![cfg(feature = "surface")]

use trajectory::surface::StlSurface;
use trajectory::tool::{TcpOffset, Tool, ToolType};
use trajectory::toolpath::{ToolPathStrategy, ToolpathParams, ZigZagStrategy};

fn ball6() -> Tool {
    Tool {
        name: "Ball6".into(),
        kind: ToolType::Ball,
        diameter_mm: 6.0,
        length_mm: 80.0,
        tcp: TcpOffset {
            xyz: [0.0, 0.0, 80.0],
            rpy_deg: None,
        },
        cone_half_angle_deg: None,
    }
}

#[test]
fn test_zigzag_on_flat_plane() {
    let surface = StlSurface::load("tests/fixtures/flat_plane.stl").expect("STL load");
    let tool = ball6();
    let params = ToolpathParams {
        stepover_mm: 10.0,
        direction_deg: 0.0,
        safe_z_mm: 50.0,
        feed_height_mm: 5.0,
        max_segment_mm: 10.0,
        offset_along_axis_mm: 0.0,
    };

    let path = ZigZagStrategy.generate(&surface, &tool, &params);

    // 11 passes (v = 0,10,...,100) × 11 samples per pass (u = 0,10,...,100) = 121 cutting
    // 4 travel poses between each adjacent pair of passes × 10 gaps = 40 travel
    let cutting = path.feed_flags.iter().filter(|b| **b).count();
    let travel = path.feed_flags.iter().filter(|b| !**b).count();
    assert_eq!(cutting, 121, "expected 121 cutting poses, got {cutting}");
    assert_eq!(travel, 40, "expected 40 travel poses, got {travel}");
    assert_eq!(path.poses.len(), 161);

    // All cutting poses on z = 0 with axis = +Z (flat plane).
    for (pose, &is_feed) in path.poses.iter().zip(path.feed_flags.iter()) {
        if is_feed {
            assert!(pose.position[2].abs() < 1e-6, "cutting z must be 0, got {}", pose.position[2]);
            assert!((pose.axis[2] - 1.0).abs() < 1e-9);
            assert!(pose.axis[0].abs() < 1e-9);
            assert!(pose.axis[1].abs() < 1e-9);
        }
    }
}

#[test]
fn test_zigzag_boustrophedon_reverses_direction() {
    let surface = StlSurface::load("tests/fixtures/flat_plane.stl").expect("STL load");
    let tool = ball6();
    let params = ToolpathParams {
        stepover_mm: 10.0,
        direction_deg: 0.0,
        safe_z_mm: 50.0,
        feed_height_mm: 5.0,
        max_segment_mm: 10.0,
        offset_along_axis_mm: 0.0,
    };

    let path = ZigZagStrategy.generate(&surface, &tool, &params);

    // Find first and second cutting passes by scanning feed_flags.
    let mut row0_x: Vec<f64> = Vec::new();
    let mut row1_x: Vec<f64> = Vec::new();
    let mut current_row: Vec<f64> = Vec::new();
    let mut rows_seen = 0usize;
    for (pose, &is_feed) in path.poses.iter().zip(path.feed_flags.iter()) {
        if is_feed {
            current_row.push(pose.position[0]);
        } else if !current_row.is_empty() {
            if rows_seen == 0 {
                row0_x = current_row.clone();
            } else if rows_seen == 1 {
                row1_x = current_row.clone();
            }
            rows_seen += 1;
            current_row.clear();
            if rows_seen >= 2 {
                break;
            }
        }
    }
    if rows_seen < 2 && !current_row.is_empty() {
        row1_x = current_row;
    }
    assert!(row0_x.first().unwrap() < row0_x.last().unwrap(), "row 0 should go +X");
    assert!(row1_x.first().unwrap() > row1_x.last().unwrap(), "row 1 should reverse and go -X");
}
