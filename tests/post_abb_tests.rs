#![cfg(all(feature = "surface", feature = "abb"))]

use trajectory::frame::WorkpieceFrame;
use trajectory::math3d::{Point3D, Pose6D, UnitVec3, Vec3};
use trajectory::post::{AbbRapidPost, PostContext, PostProcessor};
use trajectory::tool::{TcpOffset, Tool, ToolType};
use trajectory::toolpath::Toolpath;

#[test]
fn test_rapid_program_snapshot() {
    let mut path = Toolpath::new();
    let plus_z = UnitVec3::new_normalize(Vec3::z());
    path.push(Pose6D::new(Point3D::new(0.0, 0.0, 10.0), plus_z), true);
    path.push(Pose6D::new(Point3D::new(10.0, 0.0, 10.0), plus_z), true);

    let tool = Tool {
        name: "Ball6".into(),
        kind: ToolType::Ball,
        diameter_mm: 6.0,
        length_mm: 80.0,
        tcp: TcpOffset {
            xyz: [0.0, 0.0, 80.0],
            rpy_deg: None,
        },
        cone_half_angle_deg: None,
    };
    let frame = WorkpieceFrame::identity();

    let ctx = PostContext {
        program_name: "ZigZagDemo",
        tool: &tool,
        frame: &frame,
        feedrate_mm_min: 600.0,
        rapid_speed: 5000.0,
    };

    let rapid = AbbRapidPost.emit(&path, &ctx).expect("emit");

    assert!(rapid.contains("MODULE ZigZagDemo"));
    assert!(rapid.contains("PERS tooldata Tool0"));
    assert!(rapid.contains("[[0.000,0.000,80.000]"));
    assert!(rapid.contains("CONST robtarget P0001 := [[0.000,0.000,10.000]"));
    assert!(rapid.contains("CONST robtarget P0002 := [[10.000,0.000,10.000]"));
    assert!(rapid.contains("MoveL P0001"));
    assert!(rapid.contains("MoveL P0002"));
    assert!(rapid.contains("ENDPROC"));
    assert!(rapid.contains("ENDMODULE"));
    // Quaternion for axis = +Z is identity (1,0,0,0)
    assert!(rapid.contains("[1.000000,0.000000,0.000000,0.000000]"));
}

#[test]
fn test_rapid_distinguishes_movej_and_movel() {
    let mut path = Toolpath::new();
    let plus_z = UnitVec3::new_normalize(Vec3::z());
    path.push(Pose6D::new(Point3D::new(0.0, 0.0, 50.0), plus_z), false); // travel
    path.push(Pose6D::new(Point3D::new(0.0, 0.0, 0.0), plus_z), true); // cutting

    let tool = Tool {
        name: "Ball6".into(),
        kind: ToolType::Ball,
        diameter_mm: 6.0,
        length_mm: 80.0,
        tcp: TcpOffset { xyz: [0.0, 0.0, 80.0], rpy_deg: None },
        cone_half_angle_deg: None,
    };
    let frame = WorkpieceFrame::identity();
    let ctx = PostContext {
        program_name: "T",
        tool: &tool,
        frame: &frame,
        feedrate_mm_min: 600.0,
        rapid_speed: 5000.0,
    };
    let rapid = AbbRapidPost.emit(&path, &ctx).unwrap();
    assert!(rapid.contains("MoveJ P0001"));
    assert!(rapid.contains("MoveL P0002"));
}
