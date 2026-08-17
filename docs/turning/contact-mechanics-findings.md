# PiP stationary-turn contact mechanics findings

## Result

Milestone 3 identifies a sign-convention defect in the Milestone 2 evaluation
and a mechanically distinct family worth tuning later. The conservative
`differential_fore_aft/small` candidate is deterministic, fall-free, and
physically right-positive on low, nominal, and high friction surfaces. It uses
measured foot-ground forces and joint targets only; no base rotation or velocity
is commanded.

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

The ground reaction on PiP is reconstructed as `normal * normalForce +
direction1 * lateral1 + direction2 * lateral2`. Synthetic deterministic tests
validate the two lateral components, magnitude, native/right torque signs, and
multi-contact aggregation. When lateral fields are absent the tangential force
and torque are `null`, not invented as zero.

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

1. **Differential fore/aft — selected redesigned family.** Small and baseline
   candidates remain right-positive on every surface. The small candidate has
   lower translation and attitude excursion and the clearest consistent result.
2. **Same-side shear — rejected.** Baseline and strong reverse sign across
   surfaces; the small candidate's nominal drive-phase torque/yaw mechanism is
   inconsistent even though final yaw is positive.
3. **Staged pivot — rejected for now.** Low friction can produce right yaw, but
   nominal/high reverse sign. Replant/recovery behavior remains surface-sensitive.
4. **Diagonal unload control.** The corrected baseline is right-positive across
   surfaces, proving the Milestone 2 scoring-sign defect. Nearby variants fail
   the mechanism-consistency gate on at least one surface, showing it is not a
   uniformly robust family.

The family promotion policy intentionally separates the historical control from
redesigned-family selection. A larger corrected yaw from the control does not
erase its historical inefficiency or make it the best next tuning target.

## Selected candidate

`differential_fore_aft/small` uses:

- unload: 3.0 mm;
- side-opposed fore/aft travel: 2.0 mm;
- endpoint hold: 0.25 s;
- settle/replant: 0.35 s;
- one cycle at speed 20.

| Surface | Right yaw | Translation | Max roll | Max pitch | Final slip proxy | Fall |
|---|---:|---:|---:|---:|---:|:---:|
| low | +0.039264° | 0.002040 m | 0.029° | 2.176° | 0.0494 m | no |
| nominal | +0.047181° | 0.003180 m | 0.040° | 2.197° | 0.0454 m | no |
| high | +0.068252° | 0.002377 m | 0.057° | 2.181° | 0.0269 m | no |

On nominal friction all four feet support through the force-couple drive (FR
contacts for 89.6% of samples; all others for 100%). Left feet are driven toward
negative X and right feet toward positive X. Their ground reactions form a
clockwise/right couple. DRIVE_FORCE_COUPLE records a positive right-torque proxy
of `+0.07422` across its samples and gains `+0.02623°`. Recovery gives back
`−0.02593°`, but transfer/load phases preserve a final `+0.04718°` net result.
The agreement between intended force couple, reconstructed torque sign, and
measured yaw sign is the mechanical basis for promotion.

The yaw is small. That is acceptable for family selection and preferable to a
large sign-reversing result. Later work should reduce recovery cancellation and
separate settling displacement from true contacted-foot slip before increasing
amplitude.

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
