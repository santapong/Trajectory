# Trajectory

RRT/RRT* robot pathing algorithm for CNC jewelry manufacturing (filing and polishing).

## Features

- **RRT algorithm** - Rapidly-exploring Random Tree for collision-free path planning
- **RRT\* algorithm** - Optimal variant with rewiring for shorter paths
- **Collision detection** - Circle, rectangle, and polygon obstacles
- **Path smoothing** - Post-processing to create smooth CNC-friendly toolpaths
- **JSON waypoint export** - Serialized output for CNC controller consumption
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

## With OpenCV Vision (Optional)

Requires OpenCV installed on your system.

```bash
# Build with vision support
cargo build --features vision
```

## Project Structure

```
src/
├── geometry.rs   - Point2D, obstacles (Circle/Rectangle/Polygon), workspace
├── collision.rs  - Point and segment collision detection
├── config.rs     - Algorithm configuration parameters
├── rrt.rs        - Core RRT and RRT* planner
├── path.rs       - Path smoothing and JSON waypoint export
├── vision.rs     - OpenCV real-time workpiece detection (feature-gated)
├── lib.rs        - Library root
└── main.rs       - CLI demo
```

## Algorithm

The RRT (Rapidly-exploring Random Tree) algorithm:
1. Samples random points in the workspace
2. Extends the tree toward samples while avoiding obstacles
3. Connects to the goal when close enough

RRT\* improves on RRT by:
- Choosing the best parent from nearby nodes (lowest cost)
- Rewiring the tree when a new node offers a shorter path to neighbors
