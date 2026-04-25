# Trajectory

RRT/RRT* robot pathing algorithm for CNC jewelry manufacturing (filing and polishing).

## Features

- **RRT algorithm** - Rapidly-exploring Random Tree for collision-free path planning
- **RRT\* algorithm** - Optimal variant with rewiring for shorter paths
- **Collision detection** - Circle, rectangle, and polygon obstacles
- **Path smoothing** - Post-processing to create smooth CNC-friendly toolpaths
- **JSON waypoint export** - Serialized output for CNC controller consumption
- **3D surface toolpaths** - Load STL surface, generate zig-zag tool path with TCP + tool-axis vector
- **ABB RAPID post-processor** - Emit `MODULE` programs with `MoveJ`/`MoveL` and `tooldata` for ABB robots and 5-axis CNC (controller does RTCP)
- **OpenCV vision** (optional) - Real-time workpiece detection from camera

## Quick Start

```bash
# Run the demo
cargo run

# Run the jewelry ring example
cargo run --example jewelry_ring

# Run tests
cargo test
```

## 3D Surface Toolpaths

Generate a zig-zag tool path that follows a 3D surface (STL file) and emit
machine code for an ABB robot or 5-axis CNC controller.

```bash
# Run the end-to-end pipeline (STL -> zig-zag -> RAPID)
cargo run --example surface_zigzag_abb
```

Pipeline:

1. `StlSurface::load("model.stl")` - load triangle mesh, build AABB.
2. `Tool::load_json("tool.json")` - tool type, diameter, length, TCP offset.
3. `WorkpieceFrame::load_json("workpiece.json")` - 4x4 transform from
   workpiece origin to machine/robot base (XYZ + RPY in degrees).
4. `ZigZagStrategy.generate(...)` - parallel passes in XY, projected onto the
   surface via -Z ray-cast, with retract/traverse/plunge/approach travel
   moves between passes.
5. `AbbRapidPost.emit(...)` - emits a `MODULE` with `PERS tooldata`, one
   `CONST robtarget` per pose (quaternion = [w,x,y,z]), and a `PROC main()`
   body of `MoveJ` (travel) and `MoveL` (cutting) instructions.

5-axis CNC and 6-DOF robots share the same Cartesian + tool-axis pose stream;
the controller is expected to do RTCP/TCPM (e.g. ABB tooldata, Fanuc G43.4).

**Units:** all surfaces are assumed to be in millimeters. Convert STEP files
to STL externally (FreeCAD / Fusion / OnShape) before loading.

## With OpenCV Vision (Optional)

Requires OpenCV installed on your system.

```bash
# Build with vision support
cargo build --features vision
```

## Project Structure

```
src/
├── geometry.rs    - Point2D, obstacles (Circle/Rectangle/Polygon), workspace
├── collision.rs   - Point and segment collision detection
├── config.rs      - Algorithm configuration parameters
├── rrt.rs         - Core RRT and RRT* planner
├── path.rs        - Path smoothing and JSON waypoint export
├── math3d.rs      - Point3D, Pose6D, Isometry helpers (nalgebra)
├── surface/       - Surface trait, Mesh, ray-cast, STL loader
├── tool.rs        - Tool config (type, diameter, TCP offset) loaded from JSON
├── frame.rs       - WorkpieceFrame (4x4 transform) loaded from JSON
├── toolpath/      - ToolPathStrategy trait + ZigZagStrategy
├── post/          - PostProcessor trait + AbbRapidPost
├── vision.rs      - OpenCV real-time workpiece detection (feature-gated)
├── lib.rs         - Library root
└── main.rs        - CLI demo
```

## Cargo features

| Feature | Default | What it enables |
|---|---|---|
| `surface` | yes | 3D math (`nalgebra`), STL loading, surface and toolpath modules |
| `abb` | yes | ABB RAPID post-processor |
| `pointcloud` | no | Point-cloud surface stub (depth camera) |
| `heightmap` | no | Heightmap surface stub (single camera) |
| `vision` | no | OpenCV camera input for 2D obstacle detection |

`cargo build --no-default-features` produces the original 2D-only build.

## Algorithm

The RRT (Rapidly-exploring Random Tree) algorithm:
1. Samples random points in the workspace
2. Extends the tree toward samples while avoiding obstacles
3. Connects to the goal when close enough

RRT\* improves on RRT by:
- Choosing the best parent from nearby nodes (lowest cost)
- Rewiring the tree when a new node offers a shorter path to neighbors
