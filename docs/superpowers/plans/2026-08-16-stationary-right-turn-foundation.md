# PiP Stationary Right-Turn Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify the hardware-independent software foundation for PiP's future repeatable stationary right turn.

**Architecture:** Small standard-library modules expose immutable typed contracts, pure state/safety/heading logic, a lazily constructed and injected SH3001 adapter, and append-only JSONL telemetry. No module commands locomotion, and all tests use pure code or fakes.

**Tech Stack:** Python 3 standard library, dataclasses, enum, json, pytest, Git.

## Global Constraints

- Work only in `C:\Users\paul_\Documents\pip-clean`; keep `C:\Users\paul_\Documents\virtual-pip` strictly read-only.
- The project is PiP and the Python package is exactly `pip_robot`, never `pip`.
- Use standard-library modules and dependency injection; do not add Pydantic or another framework.
- Never command servos, construct a PiDog movement controller, execute locomotion, run PyBullet sweeps, or perform physical testing.
- Never use observer-status files for live balance or safety; direct SH3001 readings only.
- Commanded yaw is never measured heading; absent physical heading produces `None` and JSON `null`.
- Baseline configuration values are not evidence of physical turning.
- Use genuine red-green-refactor cycles and focused local commits. Do not push.

---

### Task 1: Repository foundation and typed models

**Files:**
- Create: `.gitignore`
- Create: `pip_robot/__init__.py`
- Create: `pip_robot/turn/__init__.py`
- Create: `pip_robot/turn/models.py`
- Create: `config/turn_right_baseline.json`
- Create: `config/robot_calibration.json`
- Create: `tests/turn/test_models.py`

**Interfaces:**
- Produces `TurnParameters.from_mapping(data)`, `ImuSample`, `PoseObservation`, `HeadingObservation`, `SafetyDecision`, and `TurnResult` as frozen dataclasses.
- `TurnParameters` fields exactly match the approved baseline; direction is `right`, unload pair is `FL_RR`, distances/timings are positive finite numbers, cycles is positive, and speed is 1..100.

- [ ] Write tests using literal valid/invalid mappings for the baseline, zero/negative/non-finite timing and distance values, invalid cycles/speed, and invalid direction/pair. Verify frozen instances reject mutation.
- [ ] Run `python -m pytest tests/turn/test_models.py -v` and confirm failure because `pip_robot.turn.models` is absent.
- [ ] Implement the frozen dataclasses and explicit `ValueError` validation. Keep serialization as standard dataclass data.
- [ ] Create the two JSON configs. `robot_calibration.json` uses `roll_error_limit_deg: 8.0`, `pitch_error_limit_deg: 8.0`, and a warning that limits require physical validation before use.
- [ ] Run `python -m pytest tests/turn/test_models.py -v` and related tests until green.
- [ ] Commit as `chore:establish-turn-parameter-contract`.

### Task 2: Explicit state machine

**Files:**
- Create: `pip_robot/turn/state_machine.py`
- Create: `tests/turn/test_state_machine.py`

**Interfaces:**
- Produces `TurnState` enum, `TransitionError`, and `TurnStateMachine(state=TurnState.PRECHECK)` with `transition(next_state)` and `abort()`.
- Normal path: `PRECHECK, STAND, SETTLE, SHIFT_UNLOAD, DRIVE_TURN, REPLANT, RECOVER, VERIFY`.
- Abort path: `ABORT, SAFE_REPLANT, RECOVER, VERIFY`; abort is legal before VERIFY and prevents return to intended movement states.

- [ ] Write tests for the complete canonical path, illegal transitions leaving state unchanged, abort from movement preparation/drive, safe replant/recovery, and rejection of movement after abort.
- [ ] Run `python -m pytest tests/turn/test_state_machine.py -v` and confirm missing-module failure.
- [ ] Implement an explicit transition table, fail-closed error, and abort behavior without hardware callbacks.
- [ ] Run focused and model tests until green.
- [ ] Commit as `feat:add-stationary-turn-state-machine`.

### Task 3: Baseline-relative safety and measured heading

**Files:**
- Create: `pip_robot/turn/safety.py`
- Create: `pip_robot/turn/heading.py`
- Create: `tests/turn/test_safety.py`
- Create: `tests/turn/test_heading.py`

**Interfaces:**
- Produces `ImuBaseline`, `settled_baseline(samples)`, and `evaluate_imu_safety(sample, baseline, roll_limit_deg, pitch_limit_deg) -> SafetyDecision`.
- Stable reason codes are `ok`, `imu_missing`, `imu_invalid`, `roll_error_limit`, and `pitch_error_limit`.
- Produces `signed_heading_delta_deg(before_deg, after_deg) -> float | None` and `is_useful_turn(before_deg, after_deg, minimum_abs_delta_deg) -> bool`; inputs are physical measurements only.

- [ ] Write safety tests proving standing bias and small jitter are allowed, large roll/pitch deviations reject, missing samples reject, and invalid/non-finite samples reject.
- [ ] Write heading tests with hand-derived literals for ordinary positive/negative deltas, `358 -> 3 == 5`, `3 -> 358 == -5`, absent measurement, and useful-turn dependence on measurement.
- [ ] Run both focused files and confirm failures because behavior is absent.
- [ ] Implement median baseline, finite validation, deterministic reason precedence, shortest signed angular difference, and missing-heading behavior.
- [ ] Run `python -m pytest tests/turn/test_safety.py tests/turn/test_heading.py -v` and related tests until green.
- [ ] Commit as `feat:add-baseline-safety-and-heading`.

### Task 4: Fresh SH3001 hardware boundary

**Files:**
- Create: `pip_robot/turn/hardware.py`
- Create: `tests/turn/test_hardware.py`

**Interfaces:**
- Produces `ImuReadError`, `Sh3001ImuAdapter(sensor=None, sensor_factory=None, monotonic=time.monotonic, utc_now=...)`, `read_sample(batch_size=1) -> ImuSample`, and `diagnose_freshness(duration_s=2.0, minimum_samples=20) -> ImuFreshnessReport`.
- Injected sensors expose `_sh3001_getimudata()` returning `(acc_xyz, gyro_xyz)`. Lazy default construction imports `pidog.sh3001.Sh3001` only when read.
- Report fields: sample count, duration, estimated Hz, first/last monotonic timestamps, and `accepted` requiring count >= minimum and timestamp advance.

- [ ] Write fake-sensor tests for injection, averaged valid batches, advancing timestamps, malformed/non-finite readings, empty batches, lazy factory behavior, and two-second diagnostic acceptance logic using an injected clock.
- [ ] Run `python -m pytest tests/turn/test_hardware.py -v` and confirm missing behavior.
- [ ] Implement direct reads, validation, historical roll/pitch convention, lazy construction, optional Robot HAT scan compatibility, and diagnostic. Do not import or read observer status.
- [ ] Run focused and related tests until green.
- [ ] Commit as `feat:add-fresh-sh3001-imu-path`.

### Task 5: Append-only structured telemetry

**Files:**
- Create: `pip_robot/turn/telemetry.py`
- Create: `tests/turn/test_telemetry.py`

**Interfaces:**
- Produces `TurnTelemetryWriter(path, git_revision, utc_now=..., monotonic=...)`, `new_run_id()`, and `record_event(run_id, parameters, state, event_type, **optional_fields) -> dict`.
- Schema is exactly `pip-turn-run/v1`; event types include `before_phase`, `after_phase`, `abort`, `safe_replant`, `recovery`, and `final_result`.
- Each record always includes schema, run ID, UTC timestamp, monotonic timestamp, Git revision, parameters, state, and event type; optional structured fields remain JSON values and absent heading is written as null.

- [ ] Write tests for multiple appended records, no overwrite, distinct run IDs, required fields, before/after events, abort reason, recovery outcome, and null heading serialization.
- [ ] Run `python -m pytest tests/turn/test_telemetry.py -v` and confirm missing behavior.
- [ ] Implement one-line append writes with dataclass/enum-safe JSON normalization and no truncating path.
- [ ] Run focused and related tests until green.
- [ ] Commit as `feat:record-turn-telemetry`.

### Task 6: Curated historical evidence and repository hygiene

**Files:**
- Create: `archive/selected-evidence/README.md`
- Copy exact approved proof documents into `archive/selected-evidence/`.

**Interfaces:**
- README maps each copied filename to its exact absolute `virtual-pip` source path, known date, preservation reason, and historical-not-validated warning.
- Preserve at most four human-readable proof documents covering diagonal unload/right pivot/diagonal-pair results; exclude JSON status, logs, patches, and executable code.

- [ ] Select the strongest documents by reading only targeted candidates and record SHA-256 checksums before and after copying.
- [ ] Copy exact files and verify identical hashes.
- [ ] Confirm `.gitignore` excludes `.venv/`, `venv/`, Python/test/coverage caches, `runs/`, logs, simulation output, runtime telemetry, `.worktrees/`, `.superpowers/`, and nested vendor `.git` content without ignoring the curated archive/docs.
- [ ] Run `git status --short` and inspect the exact archive scope.
- [ ] Commit as `docs:preserve-selected-turn-evidence`.

### Task 7: Full verification and requirement audit

**Files:**
- Modify only files required by test-proven defect fixes.

**Interfaces:**
- The complete repository must satisfy the committed design specification and every Global Constraint.

- [ ] Run `python -m pytest -v` and record exact pass/fail/warning counts.
- [ ] Run `git diff --check` and require exit 0.
- [ ] Audit source for physical movement calls, observer-status reads, package-name conflict, commanded-yaw substitution, and unignored nested Git content.
- [ ] Run task-level and whole-branch reviews; for each confirmed defect, add a failing regression test, verify the failure, implement the minimal fix, and rerun covering/full tests.
- [ ] Run fresh final `python -m pytest -v`, `git diff --check`, and `git status`.
- [ ] Commit any verified review fixes as one focused local commit. Do not push.
