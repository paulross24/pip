# PiP Stationary Right-Turn Foundation Design

## Purpose

Build a hardware-independent software foundation for a future repeatable stationary right turn. This stage defines typed contracts, explicit sequencing, fresh IMU acquisition, baseline-relative safety, measured-heading mathematics, append-only telemetry, and curated historical evidence. It must not move physical hardware or claim that baseline parameters produce a physical turn.

## Constraints

- The project is PiP; its Python package is `pip_robot`, never `pip`.
- Only `C:\Users\paul_\Documents\pip-clean` is writable.
- `C:\Users\paul_\Documents\virtual-pip` is immutable reference material.
- Use Python's standard library and dependency injection; add no framework.
- Do not issue servo, gait, stand, sit, walk, or turn commands.
- Do not run PyBullet sweeps or physical tests.
- Do not push.

## Architecture

The turn subsystem is split into focused modules with immutable dataclasses at its boundaries. Pure models, state transitions, safety decisions, and heading calculations are independent of hardware. The SH3001 adapter lazily imports or accepts a sensor dependency and reads the live device directly. Telemetry accepts structured values and appends one JSON object per line.

### Models and configuration

`TurnParameters` validates direction, unload pair, positive distances and timings, positive cycle count, and speed in the inclusive range 1 through 100. The baseline JSON is:

```json
{
  "direction": "right",
  "unload_pair": "FL_RR",
  "unload_mm": 4.0,
  "tangential_mm": 3.0,
  "hold_s": 0.35,
  "settle_s": 0.35,
  "replant_s": 0.35,
  "cycles": 1,
  "speed": 20
}
```

These values are configuration only and are not evidence of physical turning. Other immutable models represent an IMU sample, pose and heading observations, structured safety decisions, and final turn results. `config/robot_calibration.json` stores explicit initial roll/pitch safety tolerances and documents that they require validation before physical use.

### State machine

The normal path is `PRECHECK -> STAND -> SETTLE -> SHIFT_UNLOAD -> DRIVE_TURN -> REPLANT -> RECOVER -> VERIFY`.

The abort path is `ABORT -> SAFE_REPLANT -> RECOVER -> VERIFY`. Abort is permitted from any pre-verification movement phase. Once aborted, intended turn phases cannot resume. Illegal transitions raise a dedicated transition error and leave state unchanged. The state machine has no locomotion dependency.

### Fresh IMU adapter

The adapter derives only from the direct SH3001 read pattern in the historical observer. It never reads `observer_status` or any status file. Vendor imports and device construction are lazy, an injected fake sensor is supported, readings are validated as finite three-axis accelerometer and gyroscope sequences, and fresh samples receive wall-clock UTC and monotonic timestamps at acquisition.

Batch reads average valid raw readings before calculating roll and pitch using the historical PiDog orientation convention. An optional compatibility hook provides the historical Robot HAT I2C scan workaround without constructing PiDog or commanding hardware. A two-second freshness diagnostic reports sample count, elapsed duration, estimated rate, and first/last monotonic timestamps; acceptance requires at least 20 samples and advancing timestamps.

### Safety

A settled baseline is calculated from a non-empty sample window using median roll and pitch, reducing bias and jitter sensitivity. Each current sample is compared with that baseline. Missing, non-finite, or explicitly invalid samples fail closed. Decisions contain `allowed`, a stable reason code, roll/pitch errors, and configured limits. No active balancing is implemented.

### Heading

Heading is an optional physical measurement behind a small adapter boundary. Signed delta uses the shortest angular difference in `[-180, 180)`, so `358 -> 3` is `+5` and `3 -> 358` is `-5`. If either measurement is absent, the delta is `None`. A useful-turn decision requires measured heading; commanded yaw is never accepted as a measurement.

### Telemetry

Telemetry schema `pip-turn-run/v1` is append-only JSONL. Every event includes schema, run ID, UTC timestamp, monotonic timestamp, Git revision, serialized turn parameters, state, and event type. Optional fields cover requested target, fresh IMU, safety, battery, pose, measured heading, heading delta, abort phase/reason, safe replant, recovery, and final result. Missing headings serialize as JSON `null`. Phase orchestration will eventually emit before/after events; this stage supplies and tests the event contract without performing motion.

### Evidence archive

Only a very small set of strong human-readable proof documents will be copied exactly. `archive/selected-evidence/README.md` records each original absolute path, date, rationale, and a warning that the material is historical evidence rather than validated current behavior. Status dumps, logs, patch scripts, executable code, stale observer data, vendor trees, and nested repositories are excluded.

## Error handling and safety boundaries

Validation failures are explicit exceptions. Hardware import, construction, malformed reading, and empty-batch failures produce typed adapter errors rather than fabricated values. Missing IMU data denies safety. Telemetry appends one complete line per call and does not truncate an existing file. No module contains a runnable physical-turn entry point.

## Testing strategy

Use genuine red-green-refactor cycles. Tests cover parameter validation, canonical and abort transitions, movement blocking after abort, baseline bias/jitter and roll/pitch limits, missing/invalid IMU rejection, fake sensor injection and freshness diagnostics, heading wrap-around and missing measurements, and JSONL append/schema/abort/recovery/null-heading behavior. All tests use fakes or pure functions and run without PiDog hardware.

## Delivery and verification

Commit the specification, plan, and implementation in small focused local commits. Before completion, run `python -m pytest -v`, `git diff --check`, `git status`, and a final review against this specification and the governing instruction. Do not push.
