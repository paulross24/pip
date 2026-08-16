# PiP diagonal-pair pivot 10x proof — 2026-08-12

Request/rung:
- REQUEST_DIAGONAL_PAIR_PIVOT_TURN_RIGHT_10X_V1
- diagonal_pair_pivot_turn_right_10x_v1

Movement:
- leg/body movement executed: yes
- stationary-only while charging
- no vendor gait / Walk / Trot / forward translation
- 10 repeated cycles of the completed safe pattern
- drive: 1 mm
- unload: 1 mm
- speed: 6
- action duration: 81.88 s

Result:
- done
- pivot summary: {"abort_reason": null, "aborted": false, "completed_all_frames": true, "cycles_requested": 10, "drive_mm": 1, "frame_count": 51, "speed": 6, "unload_mm": 1}
- frames: 51
- live IMU sample records: 52

Safety evidence:
- max abs median roll delta: 1.823 deg
- max abs median pitch delta: 1.02 deg
- worst roll sample: ('cycle_04_phase_1c_transfer_pair2_loaded_pair1_unloaded', 1.823, -1.017, True)
- worst pitch sample: ('cycle_01_phase_1c_transfer_pair2_loaded_pair1_unloaded', 1.7, -1.02, True)
- post scan allowed: True
- post scan blockers: []
- topple stayed: UPRIGHT_OR_STABLE

Final cleanup:
- daemon owner count: 1
- daemon owners: [9467]
- E-stop: False
- active body requests: []
- active gates: []
- final state/mode: idle_loop / idle
- final sensors: {"battery_voltage": 7.91, "distance_cm": 153.45, "imu_source": "observer_status", "pitch_deg": -4.02, "roll_deg": -7.6, "topple_state": "UPRIGHT_OR_STABLE", "touch": "N", "ts": 1786527586.7264676}

Notes:
- The wrapper timed out before the long 10-cycle proof finished, but the daemon continued and wrote a fresh completed status afterward.
- Do not claim visible yaw without Paul/camera confirmation.
