# PiP Turn Contact Mechanics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Diagnose the Milestone 2 yaw mechanism and identify a deterministic, fall-free, mechanically plausible right-turn primitive family in PyBullet.

**Architecture:** A generic deterministic phase executor consumes typed, pure turn primitives. Independent contact-mechanics and diagnostic modules reconstruct Bullet ground reactions, calculate yaw moments, serialize traces, and feed a family comparison layer without coupling measurement to phase generation.

**Tech Stack:** Python 3 standard library, PyBullet 3.2.7 in DIRECT mode, pytest, frozen dataclasses and protocols, JSON/Markdown runtime reports.

## Global Constraints

- Work only inside `C:\Users\paul_\Documents\pip-clean`; `C:\Users\paul_\Documents\virtual-pip` is strictly read-only.
- No physical PiP access, SSH, servo commands, gait calls, camera/IMU access, or hardware adapter execution.
- Never create yaw through base-pose/base-velocity mutation; retain only the initial spawn reset.
- Preserve the current `pip_robot` package name and the shared `TurnParameters` model.
- Use only low, nominal, and high canonical surfaces for promotion evidence.
- Use 3–8 conservative hand-designed variants per family; do not run broad parameter sweeps.
- Runtime diagnostics belong under ignored `runs/sim/`; do not commit generated outputs.
- Use red-green-refactor TDD, request review after each independently testable task, and create small local commits.
- Do not push, rebase, amend, squash, or rewrite existing history.

---

### Task 1: Contact force decoding and yaw torque

**Files:**
- Create: `sim/contact_mechanics.py`
- Create: `tests/sim/test_contact_mechanics.py`

**Interfaces:**
- Produces: `ContactForce`, `FootMechanics`, `reconstruct_contact_force(point)`, `yaw_moment_z_nm(base_xy, point_xy, force_xy)`, and `aggregate_foot_mechanics(points, foot_links, base_position)`.
- Sign contract: `bullet_tau_z_nm = rx * fy - ry * fx`; `right_yaw_torque_nm = -bullet_tau_z_nm`.

- [ ] **Step 1: Write failing geometry and Bullet-field tests**

  Add synthetic contact tuples proving normal-force direction, both lateral
  components, a known clockwise/right force couple, multiple contacts per foot,
  torso separation, and `None` tangential force when lateral fields are absent.

- [ ] **Step 2: Run the focused RED suite**

  Run: `python -m pytest tests/sim/test_contact_mechanics.py -v -p no:cacheprovider`
  Expected: collection fails because `sim.contact_mechanics` does not exist.

- [ ] **Step 3: Implement immutable validated mechanics values and decoders**

  Validate every consumed scalar/vector as finite, document tuple indexes
  3/5/7/9/10/11/12/13, reconstruct `F = n*N + d1*f1 + d2*f2`, and keep optional
  tangential force unavailable rather than fabricated.

- [ ] **Step 4: Run focused GREEN and regression suites**

  Run focused command above, then `python -m pytest tests/sim/test_pivot_runner.py -v -p no:cacheprovider`.

- [ ] **Step 5: Review and commit**

  Inspect the diff for sign/index errors and commit as `feat:add-contact-mechanics-diagnostics`.

### Task 2: Deterministic diagnostic trace and summaries

**Files:**
- Create: `sim/diagnostics.py`
- Create: `tests/sim/test_diagnostics.py`

**Interfaces:**
- Consumes: `FootMechanics` from Task 1 and shared heading wrap helpers.
- Produces: frozen `FootTraceSample`, `PhaseTraceSample`, `DiagnosticTrace`, `PhaseSummary`; `trace_json_bytes(trace)`, `write_trace_atomic(path, trace)`, and `summarize_trace(trace)`.

- [ ] **Step 1: Write failing trace schema tests**

  Assert all required pose/velocity/contact/force/slip/torque fields, stable leg
  ordering, deterministic bytes, rejection of NaN/infinity, atomic failure, and
  correct phase yaw/slip/attitude/dominant-foot summaries.

- [ ] **Step 2: Run focused RED**

  Run: `python -m pytest tests/sim/test_diagnostics.py -v -p no:cacheprovider`
  Expected: missing-module collection failure.

- [ ] **Step 3: Implement minimal trace types and serialization**

  Serialize via `json.dumps(..., sort_keys=True, separators=(",", ":"), allow_nan=False)`;
  validate before opening the destination; use sibling temporary file plus
  `Path.replace`; compute wrapped phase yaw from first/last samples.

- [ ] **Step 4: Run focused GREEN and full pure-simulation tests**

  Run focused command, then `python -m pytest tests/sim -v -p no:cacheprovider`.

- [ ] **Step 5: Review and commit**

  Commit as `feat:add-deterministic-phase-traces`.

### Task 3: Typed primitive contract and Milestone 2 control

**Files:**
- Create: `sim/turn_primitives/__init__.py`
- Create: `sim/turn_primitives/base.py`
- Create: `sim/turn_primitives/diagonal_unload.py`
- Create: `tests/sim/turn_primitives/test_contract.py`
- Create: `tests/sim/turn_primitives/test_diagonal_unload.py`

**Interfaces:**
- Produces: `FootTarget(x_mm, down_mm)`, `PhaseAction(name, targets, duration_s, expected_support, unloaded_feet)`, `PrimitiveCandidate`, `TurnPrimitive` protocol, `validate_actions`, and `DiagonalUnloadPrimitive.build_actions(candidate)`.
- Invariant: every action targets exactly FL/FR/RL/RR; expected support and unloaded sets are disjoint.

- [ ] **Step 1: Write failing protocol/invariant/control-equivalence tests**

  Assert immutable actions, canonical complete targets, deterministic phase
  tuples, invalid support contradictions, and exact equality with existing
  `phase_endpoint_targets()` endpoints/durations.

- [ ] **Step 2: Run focused RED**

  Run: `python -m pytest tests/sim/turn_primitives -v -p no:cacheprovider`
  Expected: missing package failure.

- [ ] **Step 3: Implement the pure contract and control adapter**

  Keep all client/Bullet operations out of the package and preserve the existing
  diagonal target geometry exactly.

- [ ] **Step 4: Run focused GREEN plus runner regressions**

  Run primitive tests and `python -m pytest tests/sim/test_pivot_runner.py tests/sim/test_direct_smoke.py -v -p no:cacheprovider`.

- [ ] **Step 5: Review and commit**

  Commit as `refactor:extract-turn-primitive-contract`.

### Task 4: Generic instrumented turn executor

**Files:**
- Create: `sim/turn_runner.py`
- Modify: `sim/pivot_runner.py`
- Create: `tests/sim/test_turn_runner.py`
- Modify: `tests/sim/test_pivot_runner.py`

**Interfaces:**
- Consumes: primitive actions, diagnostics, contact mechanics, current settings/model/safety helpers.
- Produces: `InstrumentedRun(result, trace)` and `run_primitive(candidate, surface, settings, client_factory, capture_trace=True)`; compatibility `run_candidate` delegates to the diagonal primitive.

- [ ] **Step 1: Write failing injected-client safety and sampling tests**

  Prove one initial base reset, no later base reset/velocity command, action-only
  joint motor targets, per-step pose/velocity/link/contact sampling, cumulative
  contacted-foot slip, torso contact, fall abort, and compatibility result parity.

- [ ] **Step 2: Run focused RED**

  Run: `python -m pytest tests/sim/test_turn_runner.py -v -p no:cacheprovider`
  Expected: missing-module failure.

- [ ] **Step 3: Extract the generic executor minimally**

  Reuse model setup and interpolation, attach current phase to each sample, and
  make diagnostic capture observational: it cannot change commands or safety.

- [ ] **Step 4: Run focused and full GREEN**

  Run runner tests, direct smoke, then `python -m pytest -v -p no:cacheprovider`.

- [ ] **Step 5: Review and commit**

  Commit as `feat:add-instrumented-turn-runner`.

### Task 5: Baseline diagnostic command and root-cause report data

**Files:**
- Create: `tools/diagnose_turn.py`
- Create: `tests/sim/test_diagnose_turn.py`
- Modify: `.gitignore` only if `runs/sim/diagnostics/` is not already covered.

**Interfaces:**
- Produces CLI flags `--surface`, `--all-surfaces`, `--output-dir`; one trace JSON and one compact summary JSON per deterministic run.

- [ ] **Step 1: Write failing CLI/output tests with an injected run function**

  Assert canonical surface order, deterministic names/content, no raw timestep
  stdout dump, ignored default output, and nonzero exit on invalid simulations.

- [ ] **Step 2: Run focused RED**

  Run: `python -m pytest tests/sim/test_diagnose_turn.py -v -p no:cacheprovider`.

- [ ] **Step 3: Implement the compact diagnostic command**

  Keep the module import-safe and expose `run_diagnostics(...)` separately from
  `main()` for deterministic tests.

- [ ] **Step 4: Run focused GREEN, then real DIRECT baseline on all surfaces**

  Run focused tests, then `python -m tools.diagnose_turn --all-surfaces` and
  inspect phase summaries to identify yaw acquisition/cancellation, load, slip,
  and dominant torque.

- [ ] **Step 5: Record evidence and commit**

  Put measured conclusions into a temporary ignored work report for later docs;
  commit source/tests as `feat:add-baseline-contact-diagnostics`.

### Task 6: Same-side shear and differential fore/aft families

**Files:**
- Create: `sim/turn_primitives/same_side_shear.py`
- Create: `sim/turn_primitives/differential_fore_aft.py`
- Create: `tests/sim/turn_primitives/test_same_side_shear.py`
- Create: `tests/sim/turn_primitives/test_differential_fore_aft.py`

**Interfaces:**
- Produces: `SameSideShearPrimitive`, `DifferentialForeAftPrimitive`, each with 3–8 stable candidates and deterministic `build_actions`.

- [ ] **Step 1: Write failing mechanics/sequence tests**

  Assert conservative prewind before loaded shear, recovery that does not simply
  reverse under identical load, left/right fore-aft directions consistent with
  the right-positive force-couple geometry test, and complete support metadata.

- [ ] **Step 2: Run focused RED**

  Run both new test files with `-v -p no:cacheprovider`.

- [ ] **Step 3: Implement minimal pure generators and candidate sets**

  Derive targets from calibrated factory stance; do not copy historical servo
  coordinates or import historical code.

- [ ] **Step 4: Run focused GREEN and one DIRECT diagnostic per family**

  Use nominal surface first. On unexpected sign, follow the phase trace in order:
  torque, contact, slip, trajectory; alter one design assumption only through a
  new failing test.

- [ ] **Step 5: Review and commit**

  Commit as `feat:add-shear-and-force-couple-primitives`.

### Task 7: Conservative staged stepping pivot

**Files:**
- Create: `sim/turn_primitives/staged_pivot.py`
- Create: `tests/sim/turn_primitives/test_staged_pivot.py`

**Interfaces:**
- Produces: `StagedPivotPrimitive` with explicit transfer, lift, reposition, replant, complementary transfer/reposition, and recovery actions.

- [ ] **Step 1: Write failing support alternation tests**

  Assert only conservative lift/travel, every lifted foot excluded from expected
  support, replant before complementary transfer, stable deterministic order,
  and neutral recovery.

- [ ] **Step 2: Run focused RED**

  Run: `python -m pytest tests/sim/turn_primitives/test_staged_pivot.py -v -p no:cacheprovider`.

- [ ] **Step 3: Implement the minimal staged family**

  Use one-foot steps with explicit durations and no arbitrary sleeps.

- [ ] **Step 4: Run focused GREEN and nominal DIRECT diagnosis**

  Inspect support/torque/slip before changing any candidate assumption.

- [ ] **Step 5: Review and commit**

  Commit as `feat:add-staged-pivot-primitive`.

### Task 8: Candidate and family comparison

**Files:**
- Create: `sim/family_comparison.py`
- Create: `tools/compare_turn_families.py`
- Create: `tests/sim/test_family_comparison.py`
- Create: `tests/sim/test_compare_turn_families.py`

**Interfaces:**
- Produces: immutable `CandidateComparison`, `FamilyComparison`, `ComparisonReport`; `promotion_decision`, `rank_candidates`, `rank_families`, deterministic JSON/Markdown writers, and a DIRECT CLI.

- [ ] **Step 1: Write failing promotion/ranking/serialization tests**

  Assert wrong-sign rejection, fall rejection, sign-reversing high yaw below
  consistent low positive yaw, required three surfaces, phase-mechanism evidence,
  slip penalty, family-level separation, and byte-stable outputs.

- [ ] **Step 2: Run focused RED**

  Run the two comparison test files with `-v -p no:cacheprovider`.

- [ ] **Step 3: Implement minimal comparison and CLI**

  Enumerate only registered family candidate sets and store runtime artifacts
  beneath ignored `runs/sim/comparison/`.

- [ ] **Step 4: Run focused GREEN and the first full small comparison**

  Run all family candidates on low/nominal/high and inspect compact output. Any
  unexpected yaw follows systematic-debugging evidence order, not amplitude
  widening.

- [ ] **Step 5: Review and commit**

  Commit as `feat:add-turn-family-comparison`.

### Task 9: Engineering findings and regression evidence

**Files:**
- Create: `docs/turning/contact-mechanics-findings.md`
- Modify: integration tests only where measured tolerant regression bounds are needed.

**Interfaces:**
- Consumes: baseline and family comparison outputs.
- Produces: evidence-backed diagnosis, exact reference paths, family matrix, selection/rejection rationale, and physics sanity check.

- [ ] **Step 1: Add a tolerant Milestone 2 DIRECT regression test**

  Assert the control remains fall-free and in a measured near-zero/left range
  broad enough for solver tolerance, without asserting one exact float.

- [ ] **Step 2: Run RED if the new diagnostic result surface is absent, then GREEN**

  Run the focused integration test and confirm it exercises real DIRECT Bullet.

- [ ] **Step 3: Write findings from generated evidence only**

  Record baseline phase, dominant foot, load transfer, slip, cancellation,
  coordinate convention, full comparison, selected family or honest failure,
  surface results, and exact historical paths inspected.

- [ ] **Step 4: Self-review claims against machine-readable outputs**

  Remove unsupported language and verify that right-positive torque and measured
  yaw signs agree for the promoted phase.

- [ ] **Step 5: Review and commit**

  Commit as `docs:record-turn-contact-mechanics-findings`.

### Task 10: Final review, fixes, and verification

**Files:**
- Modify: only files implicated by verified review findings.

**Interfaces:**
- Produces: locally verified milestone and consolidated final report; no push.

- [ ] **Step 1: Request independent requirements and quality review**

  Review all commits after `80cd8cf632da831d459101b04cedb59b4c1539a1` against the Milestone 3 brief and this plan.

- [ ] **Step 2: Fix Critical/Important findings through TDD**

  For each verified issue, reproduce RED, implement one root-cause fix, run
  focused GREEN, and commit a small fix. Re-review until no Critical/Important
  findings remain.

- [ ] **Step 3: Run fresh full verification**

  Run: `python -m pytest -v -p no:cacheprovider`
  Run: `git diff --check`
  Run: `git status --short --branch`

- [ ] **Step 4: Run final real diagnostics and repeat comparison twice**

  Run baseline diagnostics across all surfaces. Run complete family comparison
  twice into separate runtime directories and compare JSON bytes/hashes. If not
  identical, quantify per-metric variance.

- [ ] **Step 5: Audit prohibited behavior and report**

  Search committed simulation/tool code for hardware/network imports and base
  reset/velocity methods; distinguish the one spawn reset. Report status,
  diagnosis, traces, families, selected candidate, mechanics, exact tests,
  commits, clean Git state, no push, and zero physical access.
