# PiP diagonal-unload stationary turn success — 2026-08-11

Paul observed the first attempt did not even stand. That was correct: the first design assumed PiP was already standing, but he was sat from the full-charge sit cue. The pre-scan then blocked before any leg/body primitive.

## Correction made

Updated `/home/rossp/pumpkin-pidog-agent/pumpkin_pidog_living_daemon.py` so the diagonal-unload yaw rung first performs a slow daemon-owned stand prep before scan/turn:

- stand prep: `stand`, `step_count=1`, speed `20`
- verifies IMU/topple before and after stand
- then runs the head scan and vertical floor profile
- then runs the tiny stationary diagonal-unload primitive only if gates pass

Backup:

- `/home/rossp/pumpkin-pidog-agent/pumpkin_pidog_living_daemon.py.bak-20260811-diagonal-stand-prep`

## Executed supervised attempt

Wrapper:

- `python3 pip_mobile_learning_proof.py --rung diagonal_unload_yaw_right --execute --i-confirm-paul-is-supervising`

Result: `done`

What executed:

1. Slow stand prep:
   - result: `standing_done`
   - speed: `20`
   - action duration: `0.92 s`
   - IMU delta: roll `0.0`, pitch `0.0`
2. Slow stationary diagonal-unload yaw primitive:
   - speed: `8`
   - unload lift: `2 mm`
   - twist: `3 mm`
   - unload pair: front-left index `0` + back-right index `3`
   - twist pair: front-right index `1` forward + back-left index `2` back
   - action duration: `4.44 s`
   - no vendor `turn_left/right`
   - no Walk/Trot
   - no forward body-translation target
   - no head motion during primitive

## Safety evidence

- precheck blockers: `[]`
- pre-scan blockers: `[]`
- pre vertical profile blockers: `[]`
- post-scan blockers: `[]`
- trusted floor posture band before/post: `true_standing_on_floor`
- IMU/topple after primitive: stable/upright
- IMU delta: roll `0.0`, pitch `0.0`
- E-stop: absent
- request/gate files after cleanup: `[]`
- daemon owner count: `1`
- final daemon PID: `15271`

Status files:

- `/home/rossp/pumpkin-pidog-agent/diagonal_unload_yaw_right_status.json`
- `/home/rossp/pumpkin-pidog-agent/pip_mobile_learning_proof_status.json`

## Interpretation

This proves the corrected stationary diagonal-unload rung can safely stand PiP first and execute the tiny friction-reduction/twist primitive.

Do not claim a visible turn unless Paul/camera confirms a heading change. If Paul reports no visible yaw, next tuning should be cautious: either preserve a non-neutral hold slightly longer, increase twist a little, or capture before/after camera frames before changing amplitude.
