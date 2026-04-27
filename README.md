# Trajectory

RRT/RRT* robot pathing algorithm for CNC jewelry manufacturing (filing and polishing).

## Features

- **RRT algorithm** - Rapidly-exploring Random Tree for collision-free path planning
- **RRT\* algorithm** - Optimal variant with rewiring for shorter paths
- **Collision detection** - Circle, rectangle, and polygon obstacles
- **Path smoothing** - Post-processing to create smooth CNC-friendly toolpaths
- **JSON waypoint export** - Serialized output for CNC controller consumption
- **3D surface toolpaths** - Load STL surface, generate zig-zag tool path with TCP + tool-axis vector
- **Heightmap relief from PNG** (optional) - Decode a grayscale image, map pixel intensity to Z, build a triangle mesh, run the same zig-zag + RAPID pipeline
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

## Heightmap Relief from a PNG (Optional)

Generate a relief toolpath directly from a grayscale image. White pixels become
high points, black pixels become low (configurable). The decoded heightmap is
turned into a triangle mesh that plugs into the same `Surface` trait used by
the STL pipeline, so `ZigZagStrategy` and `AbbRapidPost` consume it unchanged.

```bash
# Generate the demo PNG fixture (one-time)
cargo run --example gen_relief_png --features heightmap

# Run the end-to-end pipeline (PNG -> zig-zag -> RAPID)
cargo run --example surface_heightmap_zigzag --features heightmap
```

Pipeline:

1. `HeightMapSurface::from_png("relief.png", opts)` - decode 8/16-bit grayscale
   or RGB(A) PNG (RGB → luminance via Rec.601), apply gamma + optional Gaussian
   blur, scale intensity to depth, triangulate the regular pixel grid (two
   CCW triangles per quad), optionally add a flat z=0 pedestal ring.
2. `Tool::load_json(...)` and `WorkpieceFrame::load_json(...)` - same as the
   STL pipeline.
3. `ZigZagStrategy.generate(&surface, &tool, &params)` - identical call site
   to STL; the strategy is surface-agnostic.
4. `AbbRapidPost.emit(...)` - same RAPID emitter.

`HeightMapOpts` fields: `pixel_size_mm`, `max_depth_mm`, `gamma`, `blur_sigma`,
`invert`, `base_pad_mm`. Defaults assume a fine 0.1 mm pixel and 5 mm depth.

**Limitations:** heightmaps cannot represent undercuts. Feature size below
~2× tool diameter is unmachinable (Nyquist). Brute-force ray-cast on every
triangle is fine up to a few million triangles; a BVH is a future addition.

## CLI

A `trajectory-cli` binary turns a PNG into a RAPID program without touching
any Rust code — the entry point for UAT operators and shop-floor users.

```bash
cargo build --release --bin trajectory-cli --features cli

./target/release/trajectory-cli relief \
  --image samples/portrait.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 0.1 --max-depth 4 --stepover 0.3 --safe-z 25 \
  --feedrate 300 --note "UAT run 42" \
  --out portrait.mod --json portrait_path.json
```

The emitted RAPID `MODULE` carries a provenance comment block (version, UTC
timestamp, source-PNG SHA-256, options JSON, tool/frame paths, free-form
note) so any program can be traced back to its inputs. Cross-field validation
rejects unsafe combinations (stepover ≥ tool diameter, safe Z below relief
peak, out-of-range opts).

`trajectory-cli validate ...` runs the same checks without producing output.

## Web frontend

A single Rust binary serves a small SPA + an HTTP API. No Node or `npm`.

```bash
cargo run --release --bin trajectory-web --features web -- \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --addr 127.0.0.1:8080
# open http://127.0.0.1:8080
```

The page lets you drag-drop a PNG, tweak heightmap and toolpath options,
preview the resulting toolpath in 3D (cutting moves green, travel blue), and
download the RAPID program. API endpoints:

- `GET  /api/health` → `{"status":"ok","version":"…"}`
- `GET  /api/defaults` → default `ReliefRequest` JSON
- `POST /api/relief` (multipart: `image=<png>`, `request=<json>`) → `{rapid, toolpath, stats}`

## UAT

See [`docs/UAT.md`](docs/UAT.md) for the acceptance test procedure: build/test
gates, CLI and web functional checks, provenance verification, dry-run on a
real machine, and the sign-off form. The calibration golden test
(`tests/calibration_golden.rs`) is the load-bearing gate — it asserts a known
step-pyramid PNG produces a RAPID program whose Z values include every
expected plateau.

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
| `heightmap` | no | Grayscale-PNG → heightmap → relief surface (pulls in `png` crate) |
| `cli` | no | `trajectory-cli` binary (clap + tracing-subscriber); enables `heightmap` + `abb` |
| `web` | no | `trajectory-web` Axum server + embedded SPA (enables `cli`) |
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
