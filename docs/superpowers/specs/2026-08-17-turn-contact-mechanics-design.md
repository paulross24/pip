# PiP Turn Contact Mechanics and Primitive Redesign

**Date:** 2026-08-17  
**Status:** Approved for implementation by the Milestone 3 brief

## Purpose

Milestone 2 established a deterministic, fall-free PyBullet model whose current
diagonal-unload primitive produces near-zero or leftward yaw. Milestone 3 first
explains that result from measured contact mechanics, then compares a small set
of mechanically distinct turn families. It does not widen the old sweep, touch
physical hardware, or manufacture yaw by resetting or rotating the base.

## Design options

1. **Generic executor, typed primitives, independent diagnostics (selected).**
   Extract phase generation from execution. A single executor drives joint
   targets, samples physics, applies safety, and optionally records a trace.
   Primitive objects only create deterministic phase actions. Contact decoding,
   summaries, and family comparison stay independent.
2. **Instrument the Milestone 2 runner in place.** This is smaller initially,
   but primitive-specific branches would make the runner and measurements hard
   to validate independently.
3. **Post-process a separate high-rate replay.** This minimizes runner changes,
   but duplicates simulations and risks comparing diagnostics from a different
   execution than the scored result.

The selected design gives every family identical physics and measurement while
keeping the control family backward-compatible.

## Coordinate and torque convention

The URDF/Bullet world uses a right-handed frame: +X is the robot's forward
direction at spawn, +Y is its left direction, and +Z is upward. Positive Bullet
yaw is counter-clockwise when viewed from above, which is a physical left turn.
PiP's product-facing `yaw_delta_deg` retains the established convention where a
right turn is positive. Therefore:

- `bullet_tau_z_nm = r_x F_y - r_y F_x` is positive for counter-clockwise/left;
- `right_yaw_torque_nm = -bullet_tau_z_nm` is positive for clockwise/right;
- measured right-yaw delta is the negated wrapped Bullet yaw delta.

Every trace stores both the native Bullet moment and the right-positive moment
so sign conversion is explicit rather than implicit.

## Architecture

### Primitive contract

`sim.turn_primitives.base` defines immutable `FootTarget`, `PhaseAction`, and
`PrimitiveParameters` values plus a `TurnPrimitive` protocol. A primitive
returns a non-empty tuple of named actions. Each action contains an endpoint
target for every canonical leg, a positive duration, and explicit expected
support and unloaded-foot sets. It contains no client, sleeps, servo calls, or
base-pose operations.

The implemented families are:

- `diagonal_unload`: an exact control representation of the Milestone 2 target
  sequence;
- `same_side_shear`: a conservative prewind/load/shear sequence derived from
  the completed historical 2 mm same-side experiment;
- `differential_fore_aft`: left and right side fore/aft reactions form a force
  couple with a unit-tested intended right-positive moment;
- `staged_pivot`: transfers support, lifts and repositions one foot at a time,
  replants it, then advances the complementary foot before recovery.

Each family exposes a small hand-designed candidate set. Candidate enumeration
is stable and deliberately limited to 3–8 variants per family.

### Generic execution

`sim.turn_runner` owns model setup, initial spawn, joint discovery, interpolation,
safety sampling, and result construction. The only base reset is the existing
initial spawn. There is no base velocity command and no base pose mutation after
step zero. The old `sim.pivot_runner.run_candidate` remains a compatibility
entry point that delegates to the diagonal control primitive.

### Contact mechanics

`sim.contact_mechanics` decodes each Bullet contact tuple using the documented
PyBullet layout:

- index 3: link index on body A;
- index 5: contact position on body A in world coordinates;
- index 7: contact normal on body B, pointing toward body A;
- index 9: normal-force magnitude;
- indexes 10 and 12: lateral friction-force magnitudes;
- indexes 11 and 13: corresponding world-space friction directions.

For body A (PiP), the ground reaction is reconstructed as
`normalForce * contactNormalOnB + lateralFriction1 * direction1 +
lateralFriction2 * direction2`. Tests use synthetic contacts with unambiguous
orthogonal directions to validate sign, magnitude, aggregation, and missing
friction fields. Missing optional friction data yields an unavailable tangential
force, never a fabricated zero. Normal force and contact state remain usable.

The planar contact moment is computed about the measured base/COM reference.
Multiple points for one foot are summed. Torso link `-1` contacts are reported
separately and never assigned to a foot.

### Diagnostic trace

`sim.diagnostics` defines finite, immutable per-foot and per-sample records plus
a deterministic JSON document. Samples contain phase, time, pose, Euler angles,
linear/angular velocity, foot contacts, normal and optional tangential forces,
foot world positions, displacement since the previous sample, cumulative slip,
per-foot/native/right-positive moments, totals, support count, total normal
force, and torso contact.

Runtime traces are written atomically beneath the already ignored
`runs/sim/diagnostics/` path. JSON uses sorted keys and compact separators;
non-finite values are rejected before the destination is touched. A compact
summary groups samples by phase and reports contacts, dominant torque foot, net
and peak signed moments, phase yaw gain/loss, slip, and roll/pitch excursion.

Slip is measured from contact-foot world displacement while the same foot
remains in contact. It is a geometric diagnostic, not asserted to be Bullet's
internal friction impulse.

### Baseline diagnosis

The diagonal control is run once on low, nominal, and high friction surfaces.
Phase summaries determine where leftward yaw is acquired, which foot supplies
the dominant moment, whether FL/RR actually unload, how much contacted-foot
motion becomes slip, and how much DRIVE_TURN yaw REPLANT/RECOVER cancels. The
engineering findings document cites measured outputs only.

### Family comparison

`sim.family_comparison` executes every small candidate set against the canonical
low/nominal/high surfaces and produces deterministic JSON plus Markdown. Each
candidate includes yaw, translation, attitude excursions, contact instability,
peak positive/negative right-positive moment, cumulative signed moment proxy,
slip, fall/abort state, and phase explanation.

A candidate is promotable only when it is fall-free, never reverses to left yaw
on a required surface, has positive right yaw on every surface for the preferred
case, keeps translation and attitude within existing safety thresholds, and
shows the expected contact-moment phase without relying on uncontrolled slip.
Family ranking prioritizes sign correctness, falls, mechanical plausibility,
cross-surface consistency, translation, attitude, then yaw magnitude. A small
consistent positive result outranks a larger sign-reversing result.

## Error and safety behavior

- Malformed/non-finite contact data invalidates the diagnostic run with a clear
  reason; absent optional friction fields are represented as `null`.
- Fall detection is unchanged and remains active during every phase.
- A lifted foot cannot also be declared expected support; constructors reject
  contradictory actions.
- Diagnostic write failure cannot change a simulation result and cannot leave a
  partial destination file.
- PyBullet remains DIRECT/headless for tests and final comparisons. Optional GUI
  inspection is not required and is outside final verification.
- No module imports hardware adapters, opens network connections, or addresses
  real PiDog devices.

## Testing strategy

All production behavior follows red-green-refactor TDD. Pure tests cover torque
sign and aggregation, friction reconstruction, missing fields, trace validation
and deterministic serialization, primitive contract consistency, deterministic
phase sequences, staged support invariants, family promotion/ranking, and the
known differential force couple. Injected-client tests prove the runner never
resets the base after initial spawn. DIRECT integration tests use tolerances and
establish the Milestone 2 yaw regression range without overfitting exact floats.

Final verification runs the full test suite, baseline diagnostics on all three
surfaces, and the complete small family comparison twice. Outputs must be byte
identical; otherwise measured variance is reported. `git diff --check` and a
clean `git status` complete the local-only milestone.

## Historical evidence policy

Historical files are read-only design evidence. The clean repository will not
copy scripts, dumps, logs, observer-status data, or additional source from
`virtual-pip`. The findings document lists exact paths inspected and separates
human observation from simulation evidence.

## Completion boundary

Milestone 3 completes only if diagnostics explain the control primitive and at
least one redesigned family produces mechanically plausible, fall-free,
right-positive yaw consistently across the surface set. If none does, the final
status is honestly blocked on deeper model/control geometry redesign. Nothing
is pushed and no physical test is performed.
