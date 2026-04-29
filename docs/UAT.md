# Trajectory — UAT (User Acceptance Testing) Guide

This document is the operator-facing **checklist** + sign-off form. For the
full structured plan with test case IDs, defect classification, entry/exit
criteria, and a risk register, see:

- [`UAT_TEST_PLAN.md`](UAT_TEST_PLAN.md) — the binding plan
- [`UAT_TEST_CASES.md`](UAT_TEST_CASES.md) — every executable test case

If every item below passes, the build is **accepted for production CNC use**
for relief jewelry jobs.

UAT covers the PNG → ABB RAPID relief pipeline only. Other features
(RRT planner, OpenCV vision) have their own acceptance criteria.

---

## 0. What this build *is* and *is not*

**Is:**
- A library + CLI + web GUI that turn a grayscale PNG into a triangle mesh
  and an ABB RAPID `MODULE` of `MoveJ` (travel) and `MoveL` (cutting) moves.
- Cross-field-validated against safety footguns: stepover ≥ tool diameter,
  safe Z below relief peak, out-of-range opts.
- Self-describing: every emitted RAPID file carries a provenance comment
  block (version, UTC timestamp, source-PNG SHA-256, options JSON, tool/frame
  paths, free-form note).

**Is not:**
- A simulator. It does not check for tool-shank collisions, holder clearance,
  unmachinable undercuts, or feedrate/RPM limits of your spindle.
- A 5-axis solver. Tool axis is reported but yaw is unconstrained — fine for
  3-axis CNC and ABB controllers that do RTCP, not for general 6-DOF kinematic
  feasibility.
- Multi-tool aware. Roughing-then-finishing sequences must be assembled by
  hand from two separate runs.

UAT operators must read the **Limitations** section before signing off.

---

## 1. Prerequisites

| Item | How to verify |
|---|---|
| Rust toolchain (stable, ≥ 1.75) | `cargo --version` |
| Repo cloned and `cargo test` green at HEAD | `cargo test && cargo test --features cli` |
| `tests/fixtures/tool_ball6.json` reflects your real tool, OR a custom JSON in the same shape | open in editor, compare diameter/length/TCP to physical tool measurement |
| `tests/fixtures/workpiece_identity.json` (or your custom frame) matches the workpiece origin you've taught the controller | dry-run on machine in step 6 |
| ABB IRC5 / equivalent controller available, in **manual reduced-speed** mode | local procedure |

---

## 2. Build & test gates

Run these in order. **Stop on the first failure.**

```bash
# 1. Library + default features
cargo test

# 2. Heightmap feature
cargo test --features heightmap

# 3. CLI feature (includes the calibration golden test)
cargo test --features cli

# 4. Web feature
cargo test --features web

# 5. Build the release binaries
cargo build --release --bin trajectory-cli --features cli
cargo build --release --bin trajectory-web --features web
```

The **calibration golden test** (`tests/calibration_golden.rs`) is the
load-bearing one — it asserts that a known step-pyramid PNG produces a RAPID
program whose Z values include every expected plateau. If this passes, the
intensity → Z math is correct.

---

## 3. Functional acceptance — CLI

**Test 3.1: happy path**

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --feed-height 2.0 --max-segment 1.0 --base-pad 2.0 --blur-sigma 0.5 \
  --note "UAT 3.1" \
  --out /tmp/uat_3_1.mod
```

✅ Expected:
- Exit code 0
- `/tmp/uat_3_1.mod` exists
- First lines contain `MODULE HeightmapRelief` and a provenance block:
  - `! version       : 0.2.0`
  - `! source_sha256 : <64 hex chars>`
  - `! note          : UAT 3.1`
- File contains `PROC main()`, `MoveJ`, `MoveL`, `ENDPROC`, `ENDMODULE`

**Test 3.2: validation rejects unsafe stepover**

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 6.0 --safe-z 20.0 \
  --out /tmp/uat_3_2.mod
```

✅ Expected: non-zero exit code; stderr says
`stepover_mm (6.000) must be strictly less than tool diameter (6.000)`.
File `/tmp/uat_3_2.mod` is **not** created.

**Test 3.3: validation rejects unsafe safe_z**

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 4.0 \
  --out /tmp/uat_3_3.mod
```

✅ Expected: non-zero exit; error mentions `safe_z_mm` and clearance.

---

## 4. Functional acceptance — Web frontend

```bash
./target/release/trajectory-web \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --addr 127.0.0.1:8080
```

Open `http://127.0.0.1:8080`.

| Step | Action | Expected |
|---|---|---|
| 4.1 | Top-right shows `v<version> · ok` | health check live |
| 4.2 | Drop `tests/fixtures/relief_ramp.png` on the upload zone | filename + size shown |
| 4.3 | Leave defaults except set `pixel_size_mm=1`, `max_depth_mm=4`, `safe_z_mm=20`, `stepover_mm=1`, `base_pad_mm=2`. Add a note like "UAT 4". | form accepts |
| 4.4 | Click **Generate toolpath** | stats panel shows `4892 poses · 4624 cutting / 268 travel` (numbers match the CLI run) |
| 4.5 | 3D viewer shows green (cutting) lines forming a dome shape with blue (travel) lines connecting passes | visually correct |
| 4.6 | Drag/orbit/zoom in the 3D viewer | OrbitControls responsive |
| 4.7 | Click **Download .mod** | browser downloads `HeightmapRelief.mod` (or whatever `program_name` is set to) |
| 4.8 | Open the downloaded `.mod` in a text editor | provenance block present, `MoveL`/`MoveJ` lines present |
| 4.9 | Set `stepover_mm=7` and click **Generate** | stats unchanged; red error box shows `stepover_mm (7.000) must be strictly less than tool diameter (6.000)` |

---

## 5. Provenance acceptance

Open any `.mod` produced by either CLI or web. The header **must** contain:

```
  ! ----- trajectory provenance -----
  ! version       : <semver>
  ! timestamp_utc : <unix seconds>
  ! source_path   : <path or upload label>
  ! source_sha256 : <64 hex chars>
  ! tool_path     : <path>      (web only sets this if you wire it; CLI omits)
  ! frame_path    : <path>
  ! opts          : {"pixel_size_mm":...,"max_depth_mm":...,...}
  ! note          : <free-form>
  ! ----------------------------------
```

If the `source_sha256` of a stored UAT artifact matches the SHA-256 of the
PNG you re-ran the job against, **the program is reproducible.** Record
this hash in the sign-off form.

---

## 6. Hardware acceptance (dry-run)

**Required for first sign-off, optional for subsequent UATs.**

1. Load `/tmp/uat_3_1.mod` (or the web download) onto the controller.
2. Mount the **same** physical tool described in `tool_ball6.json`. Measure
   the actual diameter with calipers; abort if it differs by more than 0.05 mm.
3. Set the workpiece origin **per the frame JSON** (identity = machine origin).
4. Run the program in:
   - **Step mode**, single-stepping the first 10 instructions: confirm the tool
     reaches `safe_z_mm` then descends to the first relief Z without crashing.
   - **Manual reduced speed** continuously: confirm zig-zag pattern, no surprise
     retracts, smooth direction changes at pass ends.
5. Mill a sacrificial wax/foam blank (do NOT use precious metal in UAT).
6. Measure the carved depth at the dome center with calipers.
   ✅ Expected: within 0.10 mm of `max_depth_mm`.

---

## 7. Limitations to call out before sign-off

- **No collision check** — the planner does not verify the tool shank or holder
  clears the relief on retracts. If your tool is short relative to relief depth,
  set `safe_z_mm` generously (≥ 2× relief peak) and verify in step 6.
- **No undercuts** — heightmaps cannot represent overhangs. Inputs must
  represent monotonic-Z surfaces.
- **Nyquist limit** — feature size below ~2× tool diameter cannot be machined;
  fine PNG detail will be smoothed by the tool, not the algorithm.
- **No simulator** — there is no integrated G-code/RAPID simulator. Use a
  third-party sim (RobotStudio, NCViewer) for the first run on a new
  workpiece.
- **3-axis only** — although tool-axis vectors are emitted, yaw around the
  tool axis is unconstrained. Acceptable for ABB RTCP and 5-axis CNC; not
  validated for free 6-DOF kinematics.
- **Single tool, single pass** — combine roughing+finishing programs by hand.

---

## 8. Sign-off form

Copy this block, fill it out, and store it alongside the accepted `.mod`.

```
TRAJECTORY UAT SIGN-OFF
=======================
Date              : YYYY-MM-DD
Operator          : ____________________
Reviewer          : ____________________
Build version     : ____________________  (from `! version` in MODULE header)
Build commit      : ____________________  (git rev-parse HEAD)

Source PNG path   : ____________________
Source SHA-256    : ____________________  (must match `! source_sha256`)
Options JSON      : ____________________  (paste from `! opts`)
Tool config       : ____________________  (path + measured diameter mm)
Frame config      : ____________________

Test results:
  [ ] cargo test                                       PASS / FAIL
  [ ] cargo test --features heightmap                  PASS / FAIL
  [ ] cargo test --features cli  (incl. calibration)   PASS / FAIL
  [ ] cargo test --features web                        PASS / FAIL
  [ ] CLI 3.1 (happy path)                             PASS / FAIL
  [ ] CLI 3.2 (rejects unsafe stepover)                PASS / FAIL
  [ ] CLI 3.3 (rejects unsafe safe_z)                  PASS / FAIL
  [ ] Web 4.1–4.9                                      PASS / FAIL
  [ ] Provenance fields (§5)                           PRESENT / MISSING
  [ ] Dry-run depth at dome center (§6.6)              ____ mm  (target: max_depth ± 0.10)

Limitations acknowledged (§7) — operator initials: ____

Decision: [ ] ACCEPTED FOR PRODUCTION   [ ] REJECTED   [ ] CONDITIONAL: ____

Operator signature : ____________________
Reviewer signature : ____________________
```
