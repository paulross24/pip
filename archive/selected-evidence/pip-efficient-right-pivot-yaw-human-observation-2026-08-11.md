# PiP efficient right pivot-yaw human observation — 2026-08-11

After the clean daemon-gated efficient right pivot-yaw execution, Paul reported:

> didnt see a sideway turn

## Related proof

- wrapper rung: `pivot_yaw_efficient_right`
- daemon request: `REQUEST_PIVOT_YAW_EFFICIENT_RIGHT`
- daemon status: `/home/rossp/pumpkin-pidog-agent/pivot_yaw_efficient_right_status.json`
- prior success handover: `/share/pip-efficient-right-pivot-yaw-success-2026-08-11.md`

## What executed

The primitive executed safely:

- result: `done`
- action duration: `18.81 s`
- candidate yaw shift: `10 mm`
- paw lift: `6 mm`
- ratchet cycles: `2`
- speed: `15`
- no vendor turn/gait
- no Walk/Trot
- no forward translation target
- `step_count: 0`
- IMU delta: roll `0.0`, pitch `0.0`
- post-scan blockers: `[]`

## Human-visible interpretation

Despite the clean execution, Paul did not see a sideways turn.

Therefore this rung should be labelled:

- safe actuator/pose-ratchet proof;
- not a visible right turn;
- not yet useful as a practical stationary heading-change method.

## Next design implication

Do not simply repeat `pivot_yaw_efficient_right` expecting visible rightward yaw. The stationary no-forward leg-coordinate ratchet is still too inefficient on the current surface/setup.

Next work should move to a measured design change, for example:

1. capture before/after camera frames around a stronger candidate so we can measure scene/heading delta;
2. audit the leg-coordinate geometry and whether the final neutral return cancels most accumulated yaw;
3. test a non-neutral hold/settle variant only if it remains stable and safe;
4. or design a true pivot-crawl/yaw-rate primitive with tiny body-frame rotation and explicit camera confirmation.

Still do not use stock PiDog `turn_left/right` while charging/tethered, because those are forward curved crawl, not stationary yaw.