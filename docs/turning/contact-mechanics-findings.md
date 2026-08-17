# PiP stationary-turn contact mechanics findings

## Result

Milestone 3 identifies a sign-convention defect in the Milestone 2 evaluation
and explains the phase/contact mechanism, but no redesigned family meets every
promotion gate. Several differential candidates are deterministic, fall-free,
and physically right-positive on all surfaces, yet their intended drive phase
has leftward reconstructed torque. They are therefore not mechanically
promotable. The software milestone ends honestly blocked on deeper calibrated
leg/contact geometry redesign; no hardware candidate is selected.

## Coordinate and torque convention

PyBullet uses a right-handed world frame. At PiP's spawn pose +X is forward, +Y
is left, and +Z is up. Positive native Bullet yaw is counter-clockwise/left when
viewed from above; a physical clockwise/right turn is negative native yaw.

For a contact point relative to the measured base reference:

```text
bullet_tau_z_nm = r_x * F_y - r_y * F_x
right_yaw_torque_nm = -bullet_tau_z_nm
right_yaw_delta_deg = -wrapped_bullet_yaw_delta_deg
```

Traces retain both torque signs. Public Milestone 3 comparisons use the physical
right-positive values.

## Why Milestone 2 reported the wrong sign

Milestone 2 passed the native Bullet quaternion yaw directly into a scorer that
required positive yaw for a right turn. Its final verified nominal result was
`-0.34605666639762944°`, so it was disqualified as leftward. In Bullet's actual
world convention that negative angle is clockwise/right. The instrumented
control reproduces the legacy motion and translation exactly while explicitly
converting the sign: `+0.34605666639762944°` physical right yaw.

This is the primary explanation for the prior all-candidate “wrong yaw” result:
the measurement was physical, but its coordinate sign was mislabeled at the
scoring boundary. The movement remains inefficient and highly surface/contact
dependent; historical human observation still found no visible turn. Correcting
the sign does not make the baseline a proven real-robot turn.

## PyBullet contact interpretation

The diagnostic decoder uses contact tuple fields as follows:

| Index | Meaning used |
|---:|---|
| 3 | link index on PiP/body A |
| 5 | world contact position on body A |
| 7 | normal on body B, pointing toward body A |
| 9 | normal-force magnitude |
| 10 / 12 | lateral friction-force magnitudes |
| 11 / 13 | corresponding world-space lateral directions |

The normal ground reaction on PiP is `normal * normalForce`. Bullet's lateral
pair is reported in the reaction direction on body B, so the force on PiP/body A
is `-(direction1 * lateral1 + direction2 * lateral2)`. A deterministic real
DIRECT test slides a box in +X and proves the reconstructed force on the box is
negative X. Synthetic tests separately validate both lateral components,
magnitude, native/right torque signs, and multi-contact aggregation. When
lateral fields are absent the tangential force and torque are `null`, not
invented as zero.

## Baseline phase trace

The corrected physical-right baseline result is positive on all surfaces:

| Surface | Right yaw | Translation | Max roll | Max pitch | Fall |
|---|---:|---:|---:|---:|:---:|
| low | +0.738336° | 0.004577 m | 1.502° | 2.726° | no |
| nominal | +0.346057° | 0.004996 m | 1.588° | 2.746° | no |
| high | +0.440360° | 0.004684 m | 1.624° | 2.657° | no |

On nominal friction, the phase evidence is:

| Phase | Right yaw gained/lost | Dominant foot | Contact/load finding |
|---|---:|---|---|
| SHIFT_UNLOAD | +0.00777° | FL | RR drops to 0.056 N average and contacts only 9.5%; FL drops to 0.394 N but remains intermittently contacting. |
| DRIVE_TURN | +0.36656° | FR | RR is fully clear; FR/RL average 3.365/2.252 N and supply the loaded reaction. |
| REPLANT | +0.03084° | FL | RR still does not establish contact during this phase, so “replant” is delayed. |
| RECOVER | −0.05880° | RR | all feet return; recovery cancels about 16% of DRIVE_TURN yaw. |

The intended unload therefore does unload RR strongly and FL partially. The
dominant productive phase is DRIVE_TURN, not recovery. Final cumulative
contacted-foot planar displacement is 0.04712 m across all four feet, including
settling motion; this indicates appreciable compliance/slip and is why the
baseline is retained as a diagnostic control rather than treated as robust
hardware proof.

## Family comparison

Each family used three hand-designed variants (`small`, `baseline`, `strong`)
on the same low/nominal/high surfaces: 12 candidates and 36 runs. There were no
falls. The complete comparison was run twice; JSON and Markdown outputs were
byte-identical.

Family-level conclusions:

1. **Differential fore/aft — best near miss, not promoted.** All three candidates
   remain final-right-positive and fall-free on every surface. However, the
   corrected real-Bullet contact sign shows DRIVE_FORCE_COUPLE has leftward
   torque and slightly leftward yaw. Earlier load transfer supplies the residual
   right angular momentum, so the intended force-couple explanation fails.
2. **Same-side shear — rejected.** Baseline and strong reverse sign across
   surfaces; the small candidate's nominal drive-phase torque/yaw mechanism is
   inconsistent even though final yaw is positive.
3. **Staged pivot — rejected for now.** Low friction can produce right yaw, but
   nominal/high reverse sign. Replant/recovery behavior remains surface-sensitive.
4. **Diagonal unload control.** The corrected final heading is right-positive
   across surfaces, proving the Milestone 2 scoring-sign defect. DRIVE_TURN yaw
   is carried by existing angular momentum while corrected contact torque is
   leftward, so it also fails the mechanics gate.

The machine-readable report ranks families separately and records no promotable
family or candidate. A larger corrected final yaw does not override an
unexplained/opposite contact-torque mechanism.

## No selected candidate

The closest diagnostic candidate is `differential_fore_aft/small`, using:

- unload: 3.0 mm;
- side-opposed fore/aft travel: 2.0 mm;
- endpoint hold: 0.25 s;
- settle/replant: 0.35 s;
- one cycle at speed 20.

| Surface | Right yaw | Translation | Max roll | Max pitch | Final slip proxy | Fall |
|---|---:|---:|---:|---:|---:|:---:|
| low | +0.030184° | 0.002029 m | 0.026° | 2.176° | 0.1599 m | no |
| nominal | +0.029032° | 0.001981 m | 0.044° | 2.197° | 0.1343 m | no |
| high | +0.012759° | 0.001823 m | 0.083° | 2.181° | 0.3400 m | no |

The first side-opposed version gained right yaw during DRIVE but the
real-Bullet-validated force reconstruction showed net left torque. Per-foot
tracing showed FR/RL dominated the wrong sign. A second fore/rear-opposed version
was tested without changing amplitude. On nominal friction it ends at
`+0.02903°`, but DRIVE_FORCE_COUPLE itself loses about `0.0010°` and records a
leftward right-positive torque proxy near `-0.0756`; the preceding LOAD_ALL
phase supplies the positive angular momentum. This sign disagreement violates
the physics sanity check, so the candidate is explicitly rejected.

The required next action is not a larger sweep. It is to map calibrated joint
target direction to per-foot world displacement/force under load, then design a
phase whose measured contact torque and yaw acceleration agree before tuning.

## Historical references inspected (read-only)

- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-efficient-right-pivot-yaw-design-handover-2026-08-11.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-efficient-right-pivot-yaw-human-observation-2026-08-11.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-efficient-right-pivot-yaw-success-2026-08-11.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-diagonal-unload-stationary-turn-success-2026-08-11.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-diagonal-pair-pivot-turn-right-v1-proof-2026-08-12.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-diagonal-pair-pivot-turn-right-10x-v1-proof-2026-08-12.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-balance-assisted-diagonal-yaw-v1-proof-2026-08-11.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-sameside-shear-turn-right-experiment-v1-proof-2026-08-12.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-sameside-shear-prewind-turn-right-experiment-v1-proof-2026-08-12.md`
- `C:\Users\paul_\Documents\virtual-pip\pip-source\pumpkin-pidog-agent\pip-sameside-shear-direct-prewind-turn-right-10x-attempt-2026-08-12.md`

The human-readable proofs were design evidence only. No historical source,
status dump, observer-status data, log, or executable file was copied.
