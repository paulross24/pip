from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg
from sim.pivot_runner import (
    SimulationSettings,
    contact_snapshot,
    load_simulation_settings,
    load_turn_parameters,
    phase_endpoint_targets,
    run_candidate,
)
from sim.surfaces import Surface


PARAMETERS = TurnParameters(
    direction="right",
    unload_pair="FL_RR",
    unload_mm=4.0,
    tangential_mm=3.0,
    hold_s=0.20,
    settle_s=0.10,
    replant_s=0.15,
    cycles=1,
    speed=20,
)


def settings_mapping(model_path: str = "models/pip.urdf") -> dict[str, object]:
    return {
        "time_step_s": 0.05,
        "solver_iterations": 80,
        "gravity_xyz": [0.0, 0.0, -9.81],
        "spawn_height_m": 0.14,
        "initial_settle_s": 0.10,
        "fall_roll_deg": 10.0,
        "fall_pitch_deg": 12.0,
        "min_height_m": 0.075,
        "support_loss_duration_s": 0.10,
        "model_path": model_path,
    }


def quaternion(roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> tuple[float, float, float, float]:
    roll = math.radians(roll_deg) / 2.0
    pitch = math.radians(pitch_deg) / 2.0
    yaw = math.radians(yaw_deg) / 2.0
    return (
        math.sin(roll) * math.cos(pitch) * math.cos(yaw)
        - math.cos(roll) * math.sin(pitch) * math.sin(yaw),
        math.cos(roll) * math.sin(pitch) * math.cos(yaw)
        + math.sin(roll) * math.cos(pitch) * math.sin(yaw),
        math.cos(roll) * math.cos(pitch) * math.sin(yaw)
        - math.sin(roll) * math.sin(pitch) * math.cos(yaw),
        math.cos(roll) * math.cos(pitch) * math.cos(yaw)
        + math.sin(roll) * math.sin(pitch) * math.sin(yaw),
    )


def contact(link_index: int, force: float) -> tuple[object, ...]:
    return (0, 1, 0, link_index, -1, None, None, None, 0.0, force)


class FakeBulletClient:
    POSITION_CONTROL = 2

    JOINTS = (
        ("P2_FL_HIP", "fl_upper"),
        ("P3_FL_KNEE", "fl_lower"),
        ("P7_FR_HIP", "fr_upper"),
        ("P8_FR_KNEE", "fr_lower"),
        ("P0_RL_HIP", "rl_upper"),
        ("P1_RL_KNEE", "rl_lower"),
        ("P10_RR_HIP", "rr_upper"),
        ("P11_RR_KNEE", "rr_lower"),
    )

    def __init__(self, *, fall_after_steps: int | None = None) -> None:
        self.steps = 0
        self.fall_after_steps = fall_after_steps
        self.calls: list[tuple[str, int, tuple[object, ...], dict[str, object]]] = []

    def _record(self, name: str, *args: object, **kwargs: object) -> None:
        self.calls.append((name, self.steps, args, kwargs))

    def resetSimulation(self) -> None:
        self._record("resetSimulation")

    def setTimeStep(self, value: float) -> None:
        self._record("setTimeStep", value)

    def setPhysicsEngineParameter(self, **kwargs: object) -> None:
        self._record("setPhysicsEngineParameter", **kwargs)

    def setGravity(self, *gravity: float) -> None:
        self._record("setGravity", *gravity)

    def loadURDF(self, path: str, **kwargs: object) -> int:
        self._record("loadURDF", path, **kwargs)
        return 0 if "plane" in path else 1

    def changeDynamics(self, *args: object, **kwargs: object) -> None:
        self._record("changeDynamics", *args, **kwargs)

    def getNumJoints(self, body_id: int) -> int:
        return len(self.JOINTS)

    def getJointInfo(self, body_id: int, index: int) -> tuple[object, ...]:
        name, link_name = self.JOINTS[index]
        values: list[object] = [None] * 13
        values[0] = index
        values[1] = name.encode("ascii")
        values[12] = link_name.encode("ascii")
        return tuple(values)

    def resetBasePositionAndOrientation(self, *args: object, **kwargs: object) -> None:
        self._record("resetBasePositionAndOrientation", *args, **kwargs)

    def resetJointState(self, *args: object, **kwargs: object) -> None:
        self._record("resetJointState", *args, **kwargs)

    def setJointMotorControl2(self, *args: object, **kwargs: object) -> None:
        self._record("setJointMotorControl2", *args, **kwargs)

    def stepSimulation(self) -> None:
        self._record("stepSimulation")
        self.steps += 1

    def getBasePositionAndOrientation(self, body_id: int) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        if self.fall_after_steps is not None and self.steps >= self.fall_after_steps:
            return (0.01, 0.02, 0.06), quaternion(-2.397, 0.0, -179.0)
        if self.steps <= 4:
            return (0.0, 0.0, 0.14), quaternion(-2.397, 0.0, 179.0)
        if self.steps == 5:
            return (0.005, 0.01, 0.14), quaternion(1.603, -3.0, -180.0)
        return (0.03, 0.04, 0.14), quaternion(-0.397, 1.0, -179.0)

    def getContactPoints(self, **kwargs: object) -> tuple[tuple[object, ...], ...]:
        if self.fall_after_steps is not None and self.steps >= self.fall_after_steps:
            return (contact(-1, 5.0),)
        return tuple(contact(index, 2.0) for index in (1, 3, 5, 7))

    def disconnect(self) -> None:
        self._record("disconnect")


def test_settings_and_json_loaders_validate_and_preserve_the_shared_parameter_type(tmp_path: Path) -> None:
    turn_path = tmp_path / "turn.json"
    turn_path.write_text(json.dumps({**PARAMETERS.__dict__, "hold_s": 0.2}), encoding="utf-8")
    settings_path = tmp_path / "simulation.json"
    settings_path.write_text(json.dumps(settings_mapping("robot.urdf")), encoding="utf-8")

    loaded_parameters = load_turn_parameters(turn_path)
    loaded_settings = load_simulation_settings(settings_path)

    assert type(loaded_parameters) is TurnParameters
    assert loaded_parameters == PARAMETERS
    assert loaded_settings == SimulationSettings.from_mapping(settings_mapping("robot.urdf"))
    with pytest.raises(ValueError, match="time_step_s"):
        SimulationSettings.from_mapping({**settings_mapping(), "time_step_s": 0.0})
    with pytest.raises(ValueError, match="exact"):
        SimulationSettings.from_mapping({**settings_mapping(), "hardware_port": "/dev/ttyUSB0"})


def test_phase_targets_follow_the_canonical_order_and_literal_fl_rr_geometry() -> None:
    phases = phase_endpoint_targets(PARAMETERS)

    assert tuple(phases) == (
        "STAND",
        "SETTLE",
        "SHIFT_UNLOAD",
        "DRIVE_TURN",
        "REPLANT",
        "RECOVER",
    )
    assert phases["STAND"] == {
        Leg.FL: (-15.0, 95.0),
        Leg.FR: (-15.0, 95.0),
        Leg.RL: (5.0, 90.0),
        Leg.RR: (5.0, 90.0),
    }
    assert phases["SETTLE"] == phases["STAND"]
    assert phases["SHIFT_UNLOAD"] == {
        Leg.FL: (-15.0, 91.0),
        Leg.FR: (-15.0, 95.0),
        Leg.RL: (5.0, 90.0),
        Leg.RR: (5.0, 86.0),
    }
    assert phases["DRIVE_TURN"] == {
        Leg.FL: (-18.0, 91.0),
        Leg.FR: (-12.0, 95.0),
        Leg.RL: (2.0, 90.0),
        Leg.RR: (8.0, 86.0),
    }
    assert phases["REPLANT"] == {
        Leg.FL: (-18.0, 95.0),
        Leg.FR: (-12.0, 95.0),
        Leg.RL: (2.0, 90.0),
        Leg.RR: (8.0, 90.0),
    }
    assert phases["RECOVER"] == phases["STAND"]


def test_contact_snapshot_sums_normal_force_from_bullet_tuple_index_nine() -> None:
    class Contacts:
        def getContactPoints(self, **kwargs: object) -> tuple[tuple[object, ...], ...]:
            assert kwargs == {"bodyA": 7, "bodyB": 8}
            return (contact(11, 1.25), contact(11, 0.75), contact(13, 3.0), contact(-1, 99.0))

    measured = contact_snapshot(
        Contacts(),
        body_id=7,
        plane_id=8,
        foot_links={Leg.FL: 11, Leg.FR: 12, Leg.RL: 13, Leg.RR: 14},
    )

    assert [(item.foot, item.in_contact, item.normal_force_n) for item in measured] == [
        ("FL", True, 2.0),
        ("FR", False, 0.0),
        ("RL", True, 3.0),
        ("RR", False, 0.0),
    ]


def test_runner_uses_only_motor_targets_after_spawn_and_measures_bullet_motion() -> None:
    client = FakeBulletClient()

    result = run_candidate(
        PARAMETERS,
        Surface("nominal", 0.70),
        SimulationSettings.from_mapping(settings_mapping()),
        lambda: client,
    )

    assert result.parameters is PARAMETERS
    assert result.yaw_delta_deg == pytest.approx(-2.0, abs=1e-6)
    assert (result.translation_x_m, result.translation_y_m, result.translation_m) == pytest.approx((0.03, 0.04, 0.05))
    assert result.max_roll_deviation_deg == pytest.approx(4.0, abs=1e-6)
    assert result.max_pitch_deviation_deg == pytest.approx(3.0, abs=1e-6)
    assert result.elapsed_sim_s == pytest.approx(0.85)
    assert result.fell is False
    assert result.aborted is False
    assert result.invalid_reason is None
    assert tuple(contact.foot for contact in result.foot_contacts) == ("FL", "FR", "RL", "RR")

    base_resets = [call for call in client.calls if call[0] == "resetBasePositionAndOrientation"]
    joint_resets = [call for call in client.calls if call[0] == "resetJointState"]
    motor_calls = [call for call in client.calls if call[0] == "setJointMotorControl2"]
    assert len(base_resets) == 1
    assert base_resets[0][2][2] == (0.0, 0.0, 0.0, 1.0)
    assert all(call[1] == 0 for call in base_resets + joint_resets)
    assert motor_calls
    assert all(call[3]["controlMode"] == client.POSITION_CONTROL for call in motor_calls)
    assert all(call[3]["maxVelocity"] == pytest.approx(math.radians(PARAMETERS.speed)) for call in motor_calls)
    assert not any(call[0] == "resetBaseVelocity" for call in client.calls)


def test_runner_aborts_remaining_motion_when_a_measured_fall_occurs() -> None:
    safe_client = FakeBulletClient()
    fallen_client = FakeBulletClient(fall_after_steps=5)
    settings = SimulationSettings.from_mapping(settings_mapping())

    safe = run_candidate(PARAMETERS, Surface("nominal", 0.70), settings, lambda: safe_client)
    fallen = run_candidate(PARAMETERS, Surface("nominal", 0.70), settings, lambda: fallen_client)

    assert fallen.fell is True
    assert fallen.aborted is True
    assert fallen.invalid_reason == "fall-detected"
    assert fallen.elapsed_sim_s < safe.elapsed_sim_s
    assert sum(call[0] == "setJointMotorControl2" for call in fallen_client.calls) < sum(
        call[0] == "setJointMotorControl2" for call in safe_client.calls
    )


def test_runner_aborts_a_fall_during_initial_stand_before_settle_or_turn_motion() -> None:
    client = FakeBulletClient(fall_after_steps=1)

    result = run_candidate(
        PARAMETERS,
        Surface("nominal", 0.70),
        SimulationSettings.from_mapping(settings_mapping()),
        lambda: client,
    )

    assert result.fell is True
    assert result.aborted is True
    assert result.invalid_reason == "fall-detected"
    assert result.elapsed_sim_s == pytest.approx(0.05)
    assert sum(call[0] == "setJointMotorControl2" for call in client.calls) == 8


@pytest.mark.parametrize(
    "malformed_pose",
    [
        ((0.0, 0.0, 0.14), (0.0, 0.0, 0.0, 0.0)),
        ((0.0, 0.0), quaternion(-2.397)),
    ],
)
def test_malformed_base_pose_returns_a_structured_invalid_result_with_safe_fallback(
    malformed_pose: tuple[tuple[float, ...], tuple[float, ...]],
) -> None:
    class MalformedPoseClient(FakeBulletClient):
        def getBasePositionAndOrientation(self, body_id: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
            return malformed_pose

    result = run_candidate(
        PARAMETERS,
        Surface("nominal", 0.70),
        SimulationSettings.from_mapping(settings_mapping()),
        MalformedPoseClient,
    )

    assert result.aborted is True
    assert result.invalid_reason is not None
    assert result.final_pose.position_xyz == pytest.approx((0.0, 0.0, 0.14))
    assert sum(value * value for value in result.final_pose.quaternion_xyzw) == pytest.approx(1.0)


def test_drive_reaches_its_endpoint_then_holds_it_for_the_full_hold_duration() -> None:
    client = FakeBulletClient()

    run_candidate(
        PARAMETERS,
        Surface("nominal", 0.70),
        SimulationSettings.from_mapping(settings_mapping()),
        lambda: client,
    )

    fl_hip_drive_target = 0.8371526679379733
    fl_hip_targets = [
        call[3]["targetPosition"]
        for call in client.calls
        if call[0] == "setJointMotorControl2" and call[3]["jointIndex"] == 0
    ]
    drive_target_runs: list[int] = []
    for target in fl_hip_targets:
        if target == pytest.approx(fl_hip_drive_target, abs=1e-12):
            if not drive_target_runs or drive_target_runs[-1] == 0:
                drive_target_runs.append(1)
            else:
                drive_target_runs[-1] += 1
        elif drive_target_runs:
            drive_target_runs.append(0)

    assert max(drive_target_runs) == 1 + math.ceil(PARAMETERS.hold_s / 0.05)


def test_importing_runner_does_not_load_hardware_modules() -> None:
    code = (
        "import sys; import sim.pivot_runner; "
        "print(','.join(sorted(name for name in sys.modules "
        "if name.split('.')[0] in {'pidog','robot_hat','sh3001'})))"
    )

    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=Path(__file__).parents[2],
        check=True,
        capture_output=True,
        text=True,
    )

    assert completed.stdout.strip() == ""
