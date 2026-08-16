# PiP PyBullet Stationary-Turn Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the shared PiP turn-parameter contract with deterministic PyBullet simulation, objective measurement/scoring, and a bounded reproducible sweep.

**Architecture:** Focused `sim` modules separate immutable result contracts, surfaces, kinematics, scoring, and an injected Bullet runner. CLI tools use the same `TurnParameters`, copy one verified historical URDF, run DIRECT/headless simulations, and produce compact deterministic ranked output.

**Tech Stack:** Python 3.12, standard library, PyBullet, pytest, URDF, Git.

## Global Constraints

- Modify only `C:\Users\paul_\Documents\pip-clean`; `C:\Users\paul_\Documents\virtual-pip` is strictly read-only.
- Simulation consumes `pip_robot.turn.models.TurnParameters`; no second turn-parameter model.
- Never import or execute PiDog/Robot HAT/SH3001 hardware, servo, network-to-robot, or physical locomotion paths.
- Never rotate or translate the Bullet base after initial spawn; measured yaw comes only from Bullet base orientation.
- Use DIRECT/headless mode by default, fixed timestep/solver state, fixed initial pose, deterministic ordering, and no uncontrolled randomness.
- Shared turn values stay only in `config/turn_right_baseline.json`; simulator properties stay in `config/simulation.json`.
- Copy only verified `virtual-pip\pip.urdf`, preserving hash and provenance; copy no historical code, logs, caches, or repositories.
- Genuine red-green-refactor for every behavior; small local commits; no rebase/amend/push.

---

### Task 1: Simulation result contracts, surfaces, yaw, fall detection, and scoring

**Files:**
- Create: `sim/__init__.py`
- Create: `sim/model.py`
- Create: `sim/surfaces.py`
- Create: `sim/score.py`
- Create: `tests/sim/test_model.py`
- Create: `tests/sim/test_surfaces.py`
- Create: `tests/sim/test_score.py`

**Interfaces:**
- Consumes: `TurnParameters` and `HeadingObservation` from `pip_robot.turn.models`, `signed_heading_delta_deg` from `pip_robot.turn.heading`.
- Produces frozen `FootContact(foot, in_contact, normal_force_n)`, `FinalPose(position_xyz, quaternion_xyzw, roll_deg, pitch_deg, yaw_deg)`, `SimulationResult(parameters, surface_name, friction, yaw_delta_deg, translation_x_m, translation_y_m, translation_m, max_roll_deviation_deg, max_pitch_deviation_deg, fell, contact_instability, elapsed_sim_s, final_pose, foot_contacts, aborted=False, invalid_reason=None)`, `Surface`, `required_surfaces()`, `yaw_from_quaternion()`, `detect_fall(...)`, `score_result()`, and `aggregate_candidate_score()`.
- Fixed surfaces: low 0.45, nominal 0.70, high 0.95 in that order.
- Safe score: `10*yaw - 1000*translation - 2*roll - 2*pitch - 5*contact_instability`; fall/abort/invalid/non-positive yaw is `-math.inf`.

- [ ] Write literal tests proving frozen/result validation, finite metrics, quaternion yaw/wrap, fall by corrected roll >10, pitch >12, height <0.075, torso contact, sustained <2 feet for 0.10 s, surface order/friction validation, every required scoring comparison, and worst-surface aggregation.
- [ ] Run `python -m pytest tests/sim/test_model.py tests/sim/test_surfaces.py tests/sim/test_score.py -v -p no:cacheprovider`; confirm missing-module failures.
- [ ] Implement minimal pure contracts/functions with deterministic validation and tie-break data.
- [ ] Rerun focused and full tests green; commit `feat:add-simulation-contracts-and-scoring`.

### Task 2: Verified calibrated model artifact and clean geometric IK

**Files:**
- Create: `models/pip.urdf` as exact copy
- Create: `models/README.md`
- Create: `sim/kinematics.py`
- Create: `tests/sim/test_kinematics.py`

**Interfaces:**
- Produces `Leg` enum ordered FL/FR/RL/RR, `factory_stance_mm()`, `leg_angles_deg(leg, endpoint_mm)`, and `stance_joint_targets_rad(endpoints)` keyed by the eight URDF joint names.
- Uses upper/lower lengths 40/80 mm. Literal stance expectations: front hip/knee approximately `45.692012/-78.737020` degrees; rear `65.722071/-88.880871` degrees. Front/rear knee signs preserve corrected bend orientation.
- Model source hash must equal `920EF8F9045726FC3EE0E7919EEC029F9FC1D9F79812FD9A64E408C82BE51CE1` before and after copy.

- [ ] Write failing kinematic tests for factory coordinates, eight named targets, numeric stance angles, unreachable endpoints, and deterministic ordering.
- [ ] Run focused tests and confirm missing implementation.
- [ ] Implement fresh geometric IK without importing historical Python.
- [ ] Copy only the verified URDF, calculate matching SHA-256, and document exact source, rationale, mass/geometry, roll-zero, and alternatives rejected.
- [ ] Run focused/full tests and XML parse/hash checks; commit `feat:add-calibrated-pip-simulation-model`.

### Task 3: Injected phase runner and objective measurement

**Files:**
- Create: `sim/pivot_runner.py`
- Create: `tests/sim/test_pivot_runner.py`

**Interfaces:**
- Produces `SimulationSettings.from_mapping`, `load_turn_parameters(path)`, `load_simulation_settings(path)`, `phase_endpoint_targets(parameters)`, `contact_snapshot(client, body_id, plane_id, foot_links)`, `run_candidate(parameters, surface, settings, client_factory) -> SimulationResult`, and CLI `main(argv=None)`.
- Phase endpoint sequence is STAND, SETTLE, SHIFT_UNLOAD, DRIVE_TURN, REPLANT, RECOVER and consumes `unload_mm`, `tangential_mm`, `hold_s`, `settle_s`, `replant_s`, cycles, and speed. `FL_RR` remains the diagonal vertical unload pair; tangential offsets are side-opposed FL/RL versus FR/RR.
- Bullet boundary methods are injected. Initial spawn may reset base/joints; no base pose reset occurs after simulation begins.

- [ ] Write fake-client tests for shared `TurnParameters` identity, phase order/targets, motor-only drive, no post-spawn base reset, measured quaternion yaw with wrap, XY translation, baseline-relative roll/pitch maxima, contact force/index aggregation, fall abort, and absence of hardware imports.
- [ ] Run focused tests and confirm missing runner.
- [ ] Implement minimal runner with fixed-step interpolation, measurement aggregation, fail-closed invalid result, and JSON CLI output.
- [ ] Run focused/full tests; commit `feat:add-deterministic-pivot-runner`.

### Task 4: Real DIRECT smoke simulation and environment configuration

**Files:**
- Create: `config/simulation.json`
- Create: `requirements-sim.txt`
- Create: `tests/sim/test_direct_smoke.py`
- Modify: `sim/pivot_runner.py`

**Interfaces:**
- Simulation config: timestep `0.004166666666666667`, solver iterations `80`, gravity `[0,0,-9.81]`, identity-orientation spawn height `0.14`, initial settle `1.5`, measured settled roll/pitch zero, fall roll/pitch `10/12`, min height `0.075`, support-loss duration `0.10`, model `models/pip.urdf`, DIRECT default.
- CLI: `python -m sim.pivot_runner --config config/turn_right_baseline.json --smoke` prints one structured JSON result and exits 0 for valid simulation, even when physical yaw is small; GUI is optional and never used by tests.

- [ ] Install PyBullet only into the Codex bundled local runtime, record the installed version exactly in `requirements-sim.txt`, and confirm import/version.
- [ ] Write a failing real DIRECT smoke test that loads the copied URDF, returns finite structured metrics, and has no GUI/hardware dependency.
- [ ] Implement the real PyBullet client adapter and configuration loading needed for the test, without artificial base rotation.
- [ ] Run focused/full tests and the exact smoke CLI; commit `feat:add-headless-pybullet-smoke-run`.

### Task 5: Bounded deterministic sweep, ranking, and compact summary

**Files:**
- Create: `tools/__init__.py`
- Create: `tools/sweep_turn.py`
- Create: `tools/summarize_turn.py`
- Create: `tests/sim/test_sweep_turn.py`
- Create: `tests/sim/test_summarize_turn.py`

**Interfaces:**
- `candidate_parameters(baseline)` yields exactly 125 combinations in product order for unload `[2,3,4,5,6]`, tangential `[1,2,3,4,5]`, hold `[0.25,0.30,0.35,0.40,0.45]`; all other values come from baseline.
- `run_sweep(...)` evaluates every candidate on low/nominal/high, ranks by worst-surface score then deterministic safety/translation/orientation/parameter tie-breaks, and emits schema `pip-sim-turn-sweep/v1`.
- `write_ranked_json` and `render_summary` produce byte-stable output with separate model/turn/simulation hashes, PyBullet version, candidate/safe/fall counts, top ten, best score/yaw, consistency and rationale. No-safe output includes fallback parameters, worst yaw, disqualification, and a no-promotion decision. No timestamps or raw steps.
- CLI defaults to ignored `runs/sim/latest-ranked.json` and `runs/sim/latest-summary.md` and supports `--limit` for test/smoke diagnostics without changing full-search definition.

- [ ] Write failing tests for exact candidate set/order, shared contract, cross-surface evaluation, falls never ranking, deterministic tie-breaks, stable JSON bytes, concise summary fields, and output directories.
- [ ] Run focused tests and confirm missing tools.
- [ ] Implement deterministic generation, aggregation, JSON normalization, and Markdown rendering.
- [ ] Run focused/full tests and two small injected sweeps byte-compare equal; commit `feat:add-deterministic-turn-sweep`.

### Task 6: Full real sweep, determinism check, audit, and final review

**Files:**
- Modify only files required by test-first fixes.

**Interfaces:**
- Full 125-candidate × 3-surface sweep writes ignored ranked JSON and summary; repeated identical execution must produce identical bytes.

- [ ] Run fresh `python -m pytest -v -p no:cacheprovider` and record exact totals.
- [ ] Run exact smoke CLI and capture metrics.
- [ ] Run the bounded full sweep once; if runtime is excessive, diagnose and use an explicitly documented smaller bounded run rather than silently changing definitions.
- [ ] Run an identical second sweep and compare SHA-256 of both outputs.
- [ ] Audit imports and source to prove no hardware dependency, remote connection, physical command, post-initialization base pose reset, duplicate turn model, or tracked `runs/` output.
- [ ] Run `git diff --check`, `git status`, task/final review. For confirmed defects add a failing regression first, minimal fix, covering/full tests, and one focused fix commit. Do not push.

### Final review fix wave

- [x] Side-opposed tangential geometry with diagonal FL/RR vertical unload retained.
- [x] Identity spawn and measurement-only roll/pitch zero.
- [x] Separate turn/simulation hashes plus PyBullet version.
- [x] Complete no-safe summary evidence and decision.
- [x] Runner use of the shared configurable fall helper.
