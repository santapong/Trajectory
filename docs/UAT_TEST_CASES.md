# Trajectory — UAT Test Cases

Companion to [`UAT_TEST_PLAN.md`](UAT_TEST_PLAN.md). Each case is
self-contained: copy the command, run it, fill in **Actual** and **Status**.

Status values: `PASS`, `FAIL` (with defect ID), `N/A` (with justification),
`BLOCKED` (with reason).

> Conventions: paths assume repo root as working directory. `<HEX64>` means
> any 64-character hex string. `<UNIX>` means any non-zero positive integer.

---

## Build & integration (TP-BLD)

### TP-BLD-001 — Clean default build

**Objective:** Confirm the library and demo CLI build with default features.

**Preconditions:** Fresh `cargo clean` not required; clean working tree at branch HEAD.

**Steps:**
```bash
cargo build
```

**Expected:** Exit code 0. No errors. Warnings allowed but flagged in actual.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-BLD-002 — Default test suite

```bash
cargo test
```

**Expected:** All tests pass. The summary line at the bottom must read
`test result: ok. <N> passed; 0 failed`. Total `<N>` ≥ 27 across the lib /
integration tests.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-BLD-003 — Heightmap test suite

```bash
cargo test --features heightmap
```

**Expected:** All tests pass; includes 6 heightmap tests
(`from_png_flat_image_produces_expected_grid`,
`project_down_returns_expected_height`,
`gradient_image_yields_monotonic_z_along_x`,
`base_pad_extends_aabb_and_is_hittable`,
`invert_flag_swaps_high_low`,
`zigzag_strategy_consumes_heightmap_surface`).

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-BLD-004 — CLI test suite (calibration golden)

```bash
cargo test --features cli
```

**Expected:** All tests pass; includes 3 calibration golden tests
(`step_pyramid_produces_expected_z_plateaus`,
`validate_rejects_stepover_at_or_above_tool_diameter`,
`validate_rejects_unsafe_safe_z`). The first is the load-bearing one — its
failure invalidates **every** subsequent test case.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-BLD-005 — Web build

```bash
cargo build --release --bin trajectory-web --features web
cargo build --release --bin trajectory-cli --features cli
```

**Expected:** Both binaries produced under `target/release/`. Exit code 0.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## CLI functional (TP-CLI)

### TP-CLI-001 — Happy path

**Objective:** End-to-end PNG → RAPID using the CLI.

**Preconditions:** TP-BLD-005 passed.

**Steps:**
```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --feed-height 2.0 --max-segment 1.0 --base-pad 2.0 --blur-sigma 0.5 \
  --note "TP-CLI-001" \
  --out /tmp/uat_cli_001.mod
```

**Expected:**
- Exit 0
- stdout contains `wrote /tmp/uat_cli_001.mod (4892 poses: 4624 cutting / 268 travel, 7946 triangles)`
- File exists, > 500 KB
- File contains `MODULE HeightmapRelief`, `PROC main()`, `MoveL`, `MoveJ`, `ENDPROC`, `ENDMODULE`

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-CLI-002 — Validate subcommand (happy)

```bash
./target/release/trajectory-cli validate \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0
```

**Expected:** Exit 0. stdout `ok`. No file produced.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-CLI-003 — Tracing output present

**Objective:** Structured logs are emitted to stderr.

```bash
RUST_LOG=info ./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --base-pad 2.0 --note "TP-CLI-003" \
  --out /tmp/uat_cli_003.mod 2>&1 | grep -E "(toolpath ready|relief_job)"
```

**Expected:** At least one matching line containing `relief_job` and one with
`toolpath ready triangles=... poses=... cuts=... travel=...`.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-CLI-004 — Toolpath JSON export

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --base-pad 2.0 --note "TP-CLI-004" \
  --out /tmp/uat_cli_004.mod --json /tmp/uat_cli_004.json
python3 -c "import json,sys; d=json.load(open('/tmp/uat_cli_004.json')); print(d['stats']); print('poses:', len(d['poses']))"
```

**Expected:** Both files present. JSON parses. `poses` count matches the
stats line emitted by the CLI.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Web API functional (TP-API)

### TP-API-001 — Health endpoint

**Preconditions:** Start server in a separate terminal:

```bash
./target/release/trajectory-web \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --addr 127.0.0.1:8080
```

**Steps:**
```bash
curl -fsS http://127.0.0.1:8080/api/health
```

**Expected:** Body is `{"status":"ok","version":"<semver>"}`. Exit 0.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-API-002 — Defaults endpoint

```bash
curl -fsS http://127.0.0.1:8080/api/defaults | python3 -m json.tool
```

**Expected:** Valid JSON with keys `program_name`, `heightmap`, `toolpath`,
`feedrate_mm_min`, `rapid_speed`, `note`. Heightmap and toolpath are nested
objects. Numeric values are finite.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-API-003 — Relief happy path

```bash
curl -fsS -X POST http://127.0.0.1:8080/api/relief \
  -F "image=@tests/fixtures/relief_ramp.png" \
  -F 'request={"program_name":"TP-API-003","heightmap":{"pixel_size_mm":1.0,"max_depth_mm":4.0,"gamma":1.0,"blur_sigma":0.5,"invert":false,"base_pad_mm":2.0},"toolpath":{"stepover_mm":1.0,"direction_deg":0.0,"safe_z_mm":20.0,"feed_height_mm":2.0,"max_segment_mm":1.0,"offset_along_axis_mm":0.0},"feedrate_mm_min":300.0,"rapid_speed":5000.0,"note":"TP-API-003"};type=application/json' \
  | python3 -c "import sys,json; d=json.load(sys.stdin); print('rapid bytes:', len(d['rapid'])); print('stats:', d['stats']); assert 'MODULE TP-API-003' in d['rapid']; print('module name ok')"
```

**Expected:**
- `rapid bytes:` ≥ 500000
- `stats:` shows `triangles=7946`, `poses=4892`, `cutting=4624`, `travel=268`,
  AABB Z max ≈ 3.92
- `module name ok` printed (assertion passed)

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-API-004 — Relief rejects invalid params (HTTP 400)

```bash
curl -s -o /tmp/api_004_body -w "%{http_code}\n" -X POST http://127.0.0.1:8080/api/relief \
  -F "image=@tests/fixtures/relief_ramp.png" \
  -F 'request={"program_name":"Bad","heightmap":{"pixel_size_mm":1.0,"max_depth_mm":2.0,"gamma":1.0,"blur_sigma":0.0,"invert":false,"base_pad_mm":0.0},"toolpath":{"stepover_mm":7.0,"direction_deg":0.0,"safe_z_mm":20.0,"feed_height_mm":2.0,"max_segment_mm":1.0,"offset_along_axis_mm":0.0},"feedrate_mm_min":300.0,"rapid_speed":5000.0};type=application/json'
cat /tmp/api_004_body
```

**Expected:** First line of stdout = `400`. Body contains
`stepover_mm (7.000) must be strictly less than tool diameter (6.000)`.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Web UI functional (TP-UI)

> Use a real browser. Chrome / Firefox / Safari acceptable. Server from TP-API
> precondition still running.

### TP-UI-001 — Page loads, health visible

**Steps:** Open `http://127.0.0.1:8080/` in browser.

**Expected:**
- Page renders with dark theme
- Title bar shows `Trajectory · PNG → ABB RAPID toolpath`
- Top-right shows `v<semver> · ok` within 2 seconds
- Sidebar fieldsets 1–4 are visible
- 3D canvas area shows a grid + RGB axes

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-UI-002 — Defaults populate

**Expected:** All numeric inputs (pixel_size_mm, max_depth_mm, gamma,
blur_sigma, base_pad_mm, stepover_mm, direction_deg, safe_z_mm,
feed_height_mm, max_segment_mm, feedrate_mm_min) are pre-filled with values
matching `GET /api/defaults`. `program_name` is `HeightmapRelief`.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-UI-003 — Drag-drop PNG and generate

**Steps:**
1. Drag `tests/fixtures/relief_ramp.png` onto the upload zone
2. Set: pixel_size_mm=1, max_depth_mm=4, blur_sigma=0.5, base_pad_mm=2,
   stepover_mm=1, safe_z_mm=20
3. Set note = `TP-UI-003`
4. Click **Generate toolpath**

**Expected:**
- Upload zone shows filename and KB size
- Button text changes to `generating…` then back to `Generate toolpath`
- Stats panel populates with `4892 poses · 4624 cutting / 268 travel`
- 3D viewer renders **green** lines forming a dome shape with **blue**
  travel lines bridging passes
- Camera auto-fits the AABB

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-UI-004 — 3D camera controls

**Steps:** With a generated toolpath visible:
- Left-drag → orbits around the model
- Mouse wheel → zooms
- Right-drag → pans

**Expected:** All three interactions feel responsive. Z axis is up
(legend reads `Z up`).

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-UI-005 — Download .mod and inspect

**Steps:** Click **Download .mod**. Save the file. Open in any text editor.

**Expected:**
- Downloaded filename = `<program_name>.mod`
- File starts with `MODULE TP-UI-003` (or whatever program_name was set)
- Provenance block immediately follows (see TP-PROV-001 for fields)
- File ends with `ENDMODULE`

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-UI-006 — Validation surfaces error in red

**Steps:** Change `stepover_mm` to `7`. Click **Generate toolpath**.

**Expected:**
- Stats panel does **not** update with new numbers
- A red error box appears below the button containing
  `stepover_mm (7.000) must be strictly less than tool diameter (6.000)`
- Existing 3D preview (if any) remains; no JS exceptions in the dev console

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Validation / safety (TP-VAL)

### TP-VAL-001 — Stepover ≥ diameter rejected (CLI)

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 6.0 --safe-z 20.0 \
  --out /tmp/uat_val_001.mod
echo "exit=$?"
test -f /tmp/uat_val_001.mod && echo "FILE LEAKED" || echo "no file (good)"
```

**Expected:** Non-zero exit code. stderr says `stepover_mm (6.000) must be
strictly less than tool diameter (6.000)`. **No file** is created.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-VAL-002 — Safe Z below relief peak rejected (CLI)

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 4.0 \
  --out /tmp/uat_val_002.mod
echo "exit=$?"
```

**Expected:** Non-zero exit. stderr mentions `safe_z_mm` and clearance.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-VAL-003 — Out-of-range opts rejected

```bash
./target/release/trajectory-cli relief \
  --image tests/fixtures/relief_ramp.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --out /tmp/uat_val_003.mod
echo "exit=$?"
```

**Expected:** Non-zero exit. stderr mentions `pixel_size_mm must be in (0, 10] mm`.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-VAL-004 — Empty / corrupt PNG handled

```bash
echo -n "" > /tmp/empty.png
./target/release/trajectory-cli relief \
  --image /tmp/empty.png \
  --tool tests/fixtures/tool_ball6.json \
  --frame tests/fixtures/workpiece_identity.json \
  --pixel-size 1.0 --max-depth 4.0 --stepover 1.0 --safe-z 20.0 \
  --out /tmp/uat_val_004.mod
echo "exit=$?"
```

**Expected:** Non-zero exit. Error message references PNG / decode failure.
No panic, no stack trace.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Provenance (TP-PROV)

### TP-PROV-001 — All required header fields present

**Steps:** Inspect a `.mod` produced by TP-CLI-001:

```bash
head -15 /tmp/uat_cli_001.mod
```

**Expected:** Header contains, in order:
```
MODULE HeightmapRelief
  ! ----- trajectory provenance -----
  ! version       : <semver>
  ! git_sha       : <12 hex chars>     (or "unknown" if built outside a checkout)
  ! timestamp_utc : <UNIX> (unix)
  ! source_path   : <path or upload label>
  ! source_sha256 : <HEX64>
  ! tool_path     : <path>
  ! frame_path    : <path>
  ! opts          : {"pixel_size_mm":...}
  ! note          : <free-form>
  ! ----------------------------------
```

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-PROV-002 — Source hash reproducible

**Objective:** Independent verification that `source_sha256` matches the
input PNG byte-for-byte.

```bash
# Extract sha from the .mod
HEADER_SHA=$(grep -m1 "source_sha256" /tmp/uat_cli_001.mod | awk '{print $4}')
# Recompute independently
FILE_SHA=$(sha256sum tests/fixtures/relief_ramp.png | awk '{print $1}')
echo "header=$HEADER_SHA"
echo "file  =$FILE_SHA"
test "$HEADER_SHA" = "$FILE_SHA" && echo MATCH || echo MISMATCH
```

**Expected:** `MATCH`. The two hashes are identical.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-PROV-003 — Git SHA matches HEAD

```bash
HEADER_SHA=$(grep -m1 "git_sha" /tmp/uat_cli_001.mod | awk '{print $4}')
HEAD_SHA=$(git rev-parse --short=12 HEAD)
echo "header=$HEADER_SHA  head=$HEAD_SHA"
test "$HEADER_SHA" = "$HEAD_SHA" && echo MATCH || echo MISMATCH
```

**Expected:** `MATCH`. (If `unknown`, this is `FAIL` — investigate why
build.rs didn't pick up the SHA.)

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-PROV-004 — Tool config matches physical tool

**Objective:** Force a physical-vs-config check before any cut.

**Steps:**
1. Extract tool diameter from JSON: `python3 -c "import json; print(json.load(open('tests/fixtures/tool_ball6.json'))['diameter_mm'])"`
2. Measure the physical tool with calipers (3 places, average).
3. Difference must be ≤ 0.05 mm.

**Expected:** Difference ≤ 0.05 mm. Record both values.

**Actual:**  JSON: ____ mm   Measured: ____ mm   Δ: ____ mm   **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Hardware dry-run (TP-HW)

> **Required only for first sign-off.** Subsequent UATs may mark as N/A if
> nothing in the relief pipeline (validation, provenance, RAPID emit) has
> changed since the last passing TP-HW cycle. Justify any N/A in writing.

### TP-HW-001 — E-stop verified

**Expected:** E-stop has been tested in the last 24 hours, recorded by the
machine supervisor.

**Actual:** Last test: __________  By: __________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-HW-002 — Single-step the first 10 instructions

**Steps:** Load `/tmp/uat_cli_001.mod` on the controller. Step-mode through
`PROC main()` for the first 10 instructions.

**Expected:**
- Tool reaches `safe_z_mm = 20` first (rapid)
- Descends to a Z within the relief band (≤ 4.5 mm) without crashing
- No alarms or unexpected stops

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-HW-003 — Continuous run on sacrificial blank (manual reduced speed)

**Steps:** Mount a wax/foam blank (≥ 70×70×10 mm). Set workpiece origin per
the frame JSON. Run the program continuously at manual reduced speed.

**Expected:**
- Zig-zag pattern visible
- No surprise retracts mid-pass
- Smooth direction changes at pass ends
- Run completes; controller returns to start

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-HW-004 — No tool-shank or holder collision

**Steps:** During TP-HW-003, observe (or pause and verify) that the tool
shank and holder clear the relief during all retracts and traverses.

**Expected:** No contact between non-cutting parts of the tool/holder and
the workpiece.

**Actual:** _____________________  **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

### TP-HW-005 — Carved depth measurement

**Steps:** With calipers (or depth gauge), measure the carved depth at the
dome center of the milled blank.

**Expected:** Within `max_depth_mm ± 0.10 mm` of the value used in
TP-CLI-001. For TP-CLI-001 that means **3.90 – 4.10 mm**.

**Actual:** Measured: ____ mm    Spec: 3.90 – 4.10 mm    **Status:** ☐ PASS ☐ FAIL ☐ N/A

---

## Test Environment Record

Fill once at the start of the UAT cycle.

```
Operator           : ________________________
Reviewer           : ________________________
Machine supervisor : ________________________
Date / time start  : ________________________

Host OS            : ________________________
Rust version       : ________________________   (cargo --version)
Branch             : claude/research-image-path-creation-SHDJj
Commit (HEAD)      : ________________________   (git rev-parse HEAD)
Header git_sha     : ________________________   (! git_sha from any .mod)
Match?             : ☐ YES   ☐ NO  (must be YES)

Tool config        : tests/fixtures/tool_ball6.json
  JSON diameter    : ____ mm
  Measured diameter: ____ mm   Δ ≤ 0.05? ☐
Frame config       : tests/fixtures/workpiece_identity.json

Controller         : ________________________
Speed mode         : ☐ Manual reduced speed (required)
E-stop tested in last 24 h: ☐ YES (TP-HW-001)
Sacrificial blank  : ________________________
```

## Result Summary

```
TP-BLD-001 ____   TP-CLI-001 ____   TP-API-001 ____   TP-UI-001 ____
TP-BLD-002 ____   TP-CLI-002 ____   TP-API-002 ____   TP-UI-002 ____
TP-BLD-003 ____   TP-CLI-003 ____   TP-API-003 ____   TP-UI-003 ____
TP-BLD-004 ____   TP-CLI-004 ____   TP-API-004 ____   TP-UI-004 ____
TP-BLD-005 ____                                       TP-UI-005 ____
                                                      TP-UI-006 ____

TP-VAL-001 ____   TP-PROV-001 ____   TP-HW-001 ____
TP-VAL-002 ____   TP-PROV-002 ____   TP-HW-002 ____
TP-VAL-003 ____   TP-PROV-003 ____   TP-HW-003 ____
TP-VAL-004 ____   TP-PROV-004 ____   TP-HW-004 ____
                                     TP-HW-005 ____

Open S1 / S2 defects: ____   (must be 0 to sign off)
Open S3 defects     : ____   (each needs a remediation plan)

Decision: ☐ ACCEPTED   ☐ REJECTED   ☐ CONDITIONAL: ________________

Operator signature : ________________________   Date: __________
Reviewer signature : ________________________   Date: __________
```

---

After completing this document, transcribe the final decision and the
header `git_sha` / `source_sha256` into [`UAT.md`](UAT.md) §8 Sign-off.

