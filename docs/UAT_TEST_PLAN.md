# Trajectory — UAT Test Plan

| Field | Value |
|---|---|
| Document | UAT Test Plan |
| System under test | `trajectory` (PNG → ABB RAPID relief pipeline) |
| Branch | `claude/research-image-path-creation-SHDJj` |
| Test cases | [`docs/UAT_TEST_CASES.md`](UAT_TEST_CASES.md) |
| Operator checklist | [`docs/UAT.md`](UAT.md) |
| Owner | _operator name + reviewer name_ |
| Version | 1.0 |
| Date | _fill on execution_ |

---

## 1. Purpose

Prove that the `trajectory` build at the branch HEAD is fit for production use
on the target ABB IRC5 (or 5-axis CNC) controller for grayscale-PNG relief
jewelry jobs. UAT passes only when **every test case in
[`UAT_TEST_CASES.md`](UAT_TEST_CASES.md) passes** and the sign-off form in
[`UAT.md`](UAT.md) §8 is countersigned.

This plan is the binding contract between engineering and the operator.
Anything not covered here is out of scope for this UAT cycle and must be
re-tested before production use.

## 2. Scope

**In scope**
- Library + `trajectory-cli` + `trajectory-web` for the relief pipeline
- Cross-field validation, provenance, calibration golden test
- Web frontend (drag-drop, opts form, 3D preview, .mod download)
- Hardware dry-run on the target controller using a sacrificial blank

**Out of scope (require their own UAT)**
- RRT/RRT* 2D path planner (`src/rrt.rs`)
- OpenCV vision module (`src/vision.rs`)
- Point-cloud surface stub
- Multi-tool roughing+finishing sequencing
- 6-DOF kinematic feasibility outside ABB RTCP
- Production runs on precious metal (forbidden during UAT)

## 3. Test approach

| Layer | Method | Owner |
|---|---|---|
| Unit / integration | `cargo test` across feature configs | engineering |
| Calibration math | golden test (`tests/calibration_golden.rs`) | engineering |
| CLI functional | scripted black-box runs against fixtures | operator |
| Web API functional | `curl` runs against a live server | operator |
| Web UI functional | manual exploratory in a real browser | operator |
| Validation / safety | inject known-bad inputs, assert rejection | operator |
| Provenance | inspect emitted `.mod` headers, compare hashes | reviewer |
| Hardware dry-run | sacrificial blank on real controller, manual reduced speed | operator + machine supervisor |

Black-box at every layer above unit tests. The operator does not need to read
Rust — every CLI/Web/Hardware case is a copy-pasteable command and a
checkable expected output.

## 4. Test environment

| Item | Value (fill in) | Verified by |
|---|---|---|
| Host OS | _e.g. Ubuntu 24.04 / macOS 14_ | `uname -a` |
| Rust toolchain | _≥ 1.75.0 stable_ | `cargo --version` |
| Trajectory commit | _git rev-parse HEAD_ | matches `! git_sha` in emitted `.mod` |
| Tool config | _path to JSON_ | `Tool::load_json` ok |
| Frame config | _path to JSON_ | `WorkpieceFrame::load_json` ok |
| Tool diameter (measured) | _mm with calipers_ | within ±0.05 mm of JSON |
| Controller | _e.g. ABB IRC5 RW6.x / Fanuc R-30iB / Haas NGC_ | manufacturer label |
| Network for web tests | localhost only (`127.0.0.1:8080`) | `curl /api/health` |

The exact values fill the **Test Environment Record** at the end of
`UAT_TEST_CASES.md` and feed into the §8 sign-off in `UAT.md`.

## 5. Test case ID schema

```
TP-<AREA>-<NNN>
```

| Area | Meaning |
|---|---|
| `BLD` | Build / install / dependency |
| `CLI` | `trajectory-cli` binary |
| `API` | `trajectory-web` HTTP API |
| `UI`  | `trajectory-web` browser UI |
| `VAL` | Cross-field validation / safety |
| `PROV` | Provenance / traceability |
| `HW`  | Hardware dry-run |

Every test case has: ID, objective, preconditions, steps, expected,
actual, status, evidence, defect ID (if any).

## 6. Defect classification

| Severity | Definition | Example | UAT impact |
|---|---|---|---|
| **S1** Blocker | Wrong CNC output reaches machine; safety / data integrity | Z values off by mm; provenance hash mismatches input | Halt UAT immediately. No partial sign-off. |
| **S2** Critical | Feature completely unusable; no workaround | Web UI fails to start; CLI panics on a valid PNG | Halt the affected area; other areas can continue. |
| **S3** Major | Feature works but with a clear flaw | Validation rejects a valid combination; preview misrenders | Document, continue UAT, must fix before sign-off. |
| **S4** Minor | UX / cosmetic / docs | Wrong default value in defaults endpoint; typo in error | Document, may sign off conditionally. |

### Defect log template

```
DEF-<NNN>
  Test case   : TP-<AREA>-<NNN>
  Severity    : S1 / S2 / S3 / S4
  Reproducer  : <command or steps>
  Expected    : <from the test case>
  Actual      : <observed behavior + evidence path>
  Build       : <git_sha from MODULE header or `cargo --version` output>
  Status      : Open / Fixed / Won't fix / Not a defect
  Resolution  : <fix commit SHA, or rationale>
```

Every S1 / S2 must reference a defect ID before this UAT cycle can re-run.

## 7. Entry criteria

UAT may begin only when **all** of these hold:

1. `cargo test` is green on the branch HEAD (default features).
2. `cargo test --features cli` is green (includes calibration golden).
3. `cargo test --features web` is green.
4. The `tool.json` and `frame.json` configs have been physically verified
   against the real tool and workpiece origin.
5. The hardware target is in **manual reduced speed** mode and has had its
   E-stop tested in the last 24 hours.
6. A sacrificial wax/foam blank is available for §6 hardware dry-run.
7. This document and `UAT_TEST_CASES.md` have been read by the operator and
   the reviewer.

## 8. Exit criteria

Sign-off in `UAT.md` §8 may be issued only when:

1. **Every** test case in `UAT_TEST_CASES.md` is in status **PASS** or
   **N/A (with documented justification)**.
2. There are **zero** open S1 or S2 defects.
3. Open S3 defects each have an agreed remediation plan with a target date.
4. The hardware dry-run depth measurement (TP-HW-005) is within the spec
   tolerance of `max_depth_mm ± 0.10 mm`.
5. The operator and reviewer have countersigned the form in `UAT.md` §8.
6. The accepted `.mod` artifact and the source PNG are archived together
   with their SHA-256 hashes recorded in the sign-off.

## 9. Risk register

| ID | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | Tool-shank or holder collision during retract | Medium | High (machine damage) | Set `safe_z_mm` ≥ 2× relief peak; first run on sacrificial blank; spot-check by manual retract |
| R2 | Stepover too aggressive for material → tool snap | Medium | Medium | Validation guards `< tool diameter`; operator picks 10–50% of diameter for jewelry |
| R3 | PNG with undercut intent silently produces flat zone | Low | Medium | Document limitation in §7 of `UAT.md`; UAT inputs are heightmap-only |
| R4 | Provenance corruption (wrong hash recorded) | Low | High (audit trail) | Calibration golden asserts header presence; reviewer recomputes SHA-256 of source |
| R5 | Web binary bound to non-loopback by accident | Low | Medium (network exposure) | Default `127.0.0.1`; UAT uses loopback only |
| R6 | Tool config out of sync with physical tool | Medium | Medium | TP-PROV-004 forces caliper measurement check before run |
| R7 | three.js CDN unreachable during UAT | Low | Low | Cache `index.html` page in browser; offline replay is acceptable |

## 10. Schedule template

| Phase | Estimated effort | Dependencies |
|---|---|---|
| Build & test gates (TP-BLD) | 30 min | clean checkout |
| CLI functional (TP-CLI) | 30 min | TP-BLD pass |
| Web API + UI (TP-API, TP-UI) | 60 min | TP-BLD pass |
| Validation + provenance (TP-VAL, TP-PROV) | 30 min | TP-CLI pass |
| Hardware dry-run (TP-HW) | 60–120 min | controller + sacrificial blank ready |
| Sign-off | 15 min | all above PASS |
| **Total** | **3.5–4.5 hours** | — |

## 11. Roles

| Role | Responsibility |
|---|---|
| **Operator** | Executes test cases; loads `.mod` on controller; runs blank cut; records actuals |
| **Reviewer** | Independent verifier; recomputes hashes; cross-checks defects; co-signs |
| **Machine supervisor** | Authorizes manual-reduced-speed runs; oversees E-stop readiness |

The same person **must not** play both Operator and Reviewer.

## 12. Re-test policy

If any S1/S2 defect is found:

1. UAT cycle is suspended.
2. Engineering fixes; new commit lands on the branch.
3. New `git_sha` is recorded.
4. **Full UAT** is re-executed — partial re-runs are not accepted, because
   regressions are easy to introduce in shared paths (`relief_job`, `JobMetadata`,
   validation).

S3/S4 defects do not require re-test if the fix is gated by a unit test that
demonstrates the failure mode and its resolution.

---

See [`UAT_TEST_CASES.md`](UAT_TEST_CASES.md) for the executable test cases.
