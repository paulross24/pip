# PiP PyBullet Stationary-Turn Integration Design

## Purpose

Milestone 2 adds a deterministic, headless PyBullet layer that consumes the existing `pip_robot.turn.models.TurnParameters`, actuates the calibrated PiP leg joints through the canonical turn phases, measures actual simulated motion, scores safety and usefulness, and performs a bounded repeatable sweep. It never imports PiDog hardware libraries, commands physical hardware, or substitutes commanded motion for measured yaw.

## Chosen approach

Use a small `sim` package with an injected Bullet client boundary. This keeps result/scoring/surface logic independently testable and lets smoke/integration tests use DIRECT mode. Two alternatives were rejected: placing simulation inside `pip_robot.turn` would couple physical contracts to PyBullet, while copying the historical trainer wholesale would preserve legacy tuning state, logs, and a separate parameter model.

## Historical model and provenance

Copy only `C:\Users\paul_\Documents\virtual-pip\pip.urdf` to `models/pip.urdf`, preserving bytes and recording SHA-256 `920EF8F9045726FC3EE0E7919EEC029F9FC1D9F79812FD9A64E408C82BE51CE1` in `models/README.md`.

This model is selected because both `pip_pivot_trainer_v7.py` and `pip_contact_diagnostic.py` load it directly. Its total mass is 0.641000002 kg; geometry uses 40 mm upper and 80 mm lower legs and agrees with the latest IK. `pip-CALIBRATED-BASELINE.urdf` has matching structure but an obsolete 1.428 kg mass, while `pip-STABLE-BACKUP.urdf` has incompatible older geometry and 1.06 kg mass.

The adopted stance is FL/FR `[-15, 95]` mm and RL/RR `[5, 90]` mm, producing approximately front hip/knee `+45.692/-78.737` degrees and rear `+65.722/-88.881` degrees. Calibrated standing roll offset is `-2.397` degrees. These values are simulator calibration, not evidence of physical turning.

## Components

### `sim/model.py`

Defines frozen `SimulationResult`, `FootContact`, `FinalPose`, `SurfaceResult`, and validation errors. Results always carry the exact `TurnParameters`, measured yaw delta, planar translation vector/distance, baseline-relative maximum roll/pitch deviations, fall/invalid state, contact metrics, simulated elapsed time, surface data, abort reason, and final pose. Non-finite or inconsistent results are rejected.

### `sim/surfaces.py`

Defines an ordered immutable surface family: `low` friction 0.45, `nominal` 0.70, and `high` 0.95. Friction must be positive and finite. Ordering is fixed low/nominal/high for reproducible output.

### `sim/kinematics.py`

Contains the clean 40/80 mm geometric IK and maps leg endpoint coordinates to the eight URDF joint targets. Front knees bend forward and rear knees backward. This is a fresh focused port verified against historical factory-stance angles.

### `sim/score.py`

Scores one `SimulationResult` explicitly. Falls, aborts, invalid results, non-positive measured right yaw, or non-finite metrics are disqualified with negative infinity. Safe score is:

`10*yaw_deg - 1000*translation_m - 2*max_roll_deviation_deg - 2*max_pitch_deviation_deg - 5*contact_instability`

`contact_instability` is the fraction of phase samples with fewer than three feet in contact, clamped to `[0, 1]`. Cross-surface candidate score is the minimum surface score, with deterministic tie-breaks favoring fewer falls, lower translation, lower orientation excursion, then lexicographic parameter order. Weights are visible configuration constants, not hidden tuning.

### `sim/pivot_runner.py`

Loads `config/turn_right_baseline.json` through `TurnParameters.from_mapping`, loads `config/simulation.json`, and uses DIRECT mode by default. An injected client/factory supports unit tests. The runner loads the selected URDF and plane, applies surface friction, sets deterministic engine properties, establishes the stance and settled baseline, then executes `STAND`, `SETTLE`, `SHIFT_UNLOAD`, `DRIVE_TURN`, `REPLANT`, and `RECOVER` through joint motor targets only.

The phase mapping uses `FL_RR` only as the diagonal vertical unload pair. Tangential drive is side-opposed: FL/RL move together opposite FR/RR by `tangential_mm`; hold for `hold_s`; replant over `replant_s`; settle/recover over configured durations. The base spawns at identity orientation, settled roll/pitch zero is measured from Bullet, and no post-initialization base reset or artificial rotation occurs.

At every step it samples base pose and `getContactPoints`. Contacts are assigned to FL/FR/RL/RR lower links, normal forces come from tuple index 9, and unexpected support loss is fewer than three feet contacting. Yaw is measured from base quaternion before/after and passed through the existing sourced-heading wrap calculation. Translation is final minus initial planar base position. Roll/pitch maxima are deviations from the settled baseline.

Fall detection is deterministic and centralized in the shared configurable helper: baseline-relative roll, pitch, height, torso contact, and support-loss observations are evaluated against `SimulationSettings`. A fall aborts remaining intended turn phases but still records a result.

### `tools/sweep_turn.py`

Generates exactly 125 deterministic candidates from unload `[2,3,4,5,6]` mm, tangential `[1,2,3,4,5]` mm, and hold `[0.25,0.30,0.35,0.40,0.45]` seconds. Every candidate runs on all three surfaces. Other `TurnParameters` values come from the shared baseline config. Output order, JSON key ordering, float rounding, and tie-breaks are fixed.

Writes ignored `runs/sim/latest-ranked.json` and `runs/sim/latest-summary.md`. The JSON contains separate model, turn-config, and simulation-config hashes, PyBullet version, and per-surface metrics. The summary contains counts, best metrics, top ten candidates, cross-surface consistency, and recommendation rationale; a no-safe result still includes fallback parameters, worst yaw, disqualification, and an explicit no-promotion decision.

### `tools/summarize_turn.py`

Purely summarizes an existing ranked JSON file and cannot run simulation. This keeps deterministic aggregation separate from costly execution.

## Configuration and dependency

`config/turn_right_baseline.json` remains unchanged and shared. `config/simulation.json` stores only environment/model properties: DIRECT mode, fixed 1/240 s timestep, 80 solver iterations, gravity `[0,0,-9.81]`, spawn height 0.14 m, 1.5 s initial settle, friction family, phase sampling, fall thresholds, and model path. Roll/pitch zero is measurement-only and is never injected as spawn orientation.

PyBullet is the only new runtime dependency and is recorded in `requirements-sim.txt` with a compatible pinned version after installation verification. No virtual environment is committed.

## Determinism and error handling

Simulation resets the world for every candidate/surface, disables real-time simulation, uses fixed timestep/solver settings, applies identical initial joint/base state, and introduces no randomness. Configuration, URDF, invalid IK, missing joints, non-finite Bullet data, or unavailable PyBullet produce explicit invalid/abort results or clear CLI errors. Machine output never fabricates motion.

Two identical bounded sweeps must produce byte-identical ranked JSON apart from no timestamps being included; Markdown summaries must also match exactly.

## Testing

TDD covers result validation, score ordering/disqualification, deterministic surfaces, yaw quaternion extraction/wrap, fall detection, kinematic stance, phase mapping, contact aggregation, injected smoke runner behavior, shared `TurnParameters` identity, absence of hardware imports, deterministic candidate generation/ranking, and summary formatting. A real DIRECT smoke integration test is marked only by dependency availability and remains headless.

## Safety boundaries

The simulator never imports `pidog`, `robot_hat`, or the real SH3001 adapter; never connects remotely; never commands servos; and never runs physical stand/turn/recovery. Historical files are read-only. No Git push occurs during this milestone.

## Verification

Run the full suite, `git diff --check`, a real DIRECT smoke command, the bounded sweep, and a second identical sweep for byte comparison. Final review must confirm the base is never artificially rotated and measured yaw is derived only from Bullet base orientation.
