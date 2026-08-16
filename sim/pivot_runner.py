"""Deterministic Bullet runner for stationary-turn candidates.

The simulation boundary remains injectable for unit tests.  The production
adapter imports PyBullet lazily and always connects in headless DIRECT mode;
physical-hardware packages are never imported.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import functools
import inspect
import json
import math
from numbers import Real
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

from pip_robot.turn.models import TurnParameters

from .kinematics import Leg, factory_stance_mm, inverse_kinematics_deg
from .model import FinalPose, FootContact, SimulationResult, detect_fall, yaw_delta_from_quaternions, yaw_from_quaternion
from .surfaces import Surface, required_surfaces


PHASES = ("STAND", "SETTLE", "SHIFT_UNLOAD", "DRIVE_TURN", "REPLANT", "RECOVER")

_SETTINGS_FIELDS = frozenset(
    {
        "time_step_s",
        "solver_iterations",
        "gravity_xyz",
        "spawn_height_m",
        "initial_settle_s",
        "fall_roll_deg",
        "fall_pitch_deg",
        "min_height_m",
        "support_loss_duration_s",
        "model_path",
    }
)
_JOINT_NAMES = {
    Leg.FL: ("P2_FL_HIP", "P3_FL_KNEE"),
    Leg.FR: ("P7_FR_HIP", "P8_FR_KNEE"),
    Leg.RL: ("P0_RL_HIP", "P1_RL_KNEE"),
    Leg.RR: ("P10_RR_HIP", "P11_RR_KNEE"),
}
_FOOT_LINK_NAMES = {
    Leg.FL: "fl_lower",
    Leg.FR: "fr_lower",
    Leg.RL: "rl_lower",
    Leg.RR: "rr_lower",
}


def _finite(value: object, field: str, *, positive: bool = False, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number")
    number = float(value)
    if positive and number <= 0.0:
        raise ValueError(f"{field} must be positive")
    if nonnegative and number < 0.0:
        raise ValueError(f"{field} must be nonnegative")
    return number


def _vector3(value: object, field: str) -> tuple[float, float, float]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must contain exactly three finite numbers")
    try:
        values = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise ValueError(f"{field} must contain exactly three finite numbers") from error
    if len(values) != 3:
        raise ValueError(f"{field} must contain exactly three finite numbers")
    return tuple(_finite(item, field) for item in values)  # type: ignore[return-value]


@dataclass(frozen=True)
class SimulationSettings:
    """Validated deterministic environment and safety settings."""

    time_step_s: float
    solver_iterations: int
    gravity_xyz: tuple[float, float, float]
    spawn_height_m: float
    initial_settle_s: float
    fall_roll_deg: float
    fall_pitch_deg: float
    min_height_m: float
    support_loss_duration_s: float
    model_path: str

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SimulationSettings":
        if not isinstance(data, Mapping) or set(data) != _SETTINGS_FIELDS:
            raise ValueError("simulation setting fields must exactly match the approved schema")
        solver_iterations = data["solver_iterations"]
        if (
            isinstance(solver_iterations, bool)
            or not isinstance(solver_iterations, int)
            or solver_iterations <= 0
        ):
            raise ValueError("solver_iterations must be a positive integer")
        model_path = data["model_path"]
        if not isinstance(model_path, str) or not model_path.strip():
            raise ValueError("model_path must be a nonblank string")
        return cls(
            time_step_s=_finite(data["time_step_s"], "time_step_s", positive=True),
            solver_iterations=solver_iterations,
            gravity_xyz=_vector3(data["gravity_xyz"], "gravity_xyz"),
            spawn_height_m=_finite(data["spawn_height_m"], "spawn_height_m", positive=True),
            initial_settle_s=_finite(data["initial_settle_s"], "initial_settle_s", nonnegative=True),
            fall_roll_deg=_finite(data["fall_roll_deg"], "fall_roll_deg", positive=True),
            fall_pitch_deg=_finite(data["fall_pitch_deg"], "fall_pitch_deg", positive=True),
            min_height_m=_finite(data["min_height_m"], "min_height_m", positive=True),
            support_loss_duration_s=_finite(
                data["support_loss_duration_s"], "support_loss_duration_s", positive=True
            ),
            model_path=model_path.strip(),
        )


def _load_json_mapping(path: str | Path) -> Mapping[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, Mapping):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def load_turn_parameters(path: str | Path) -> TurnParameters:
    """Load the sole shared turn-parameter contract from JSON."""
    return TurnParameters.from_mapping(_load_json_mapping(path))


def load_simulation_settings(path: str | Path) -> SimulationSettings:
    """Load deterministic simulation-only settings from JSON."""
    return SimulationSettings.from_mapping(_load_json_mapping(path))


def _endpoints(
    *,
    unloaded_diagonal_offset_mm: float = 0.0,
    loaded_diagonal_offset_mm: float = 0.0,
    unload_mm: float = 0.0,
    replant: bool = False,
) -> dict[Leg, tuple[float, float]]:
    result: dict[Leg, tuple[float, float]] = {}
    for leg in Leg:
        x_mm, down_mm = factory_stance_mm[leg]
        offset = unloaded_diagonal_offset_mm if leg in (Leg.FL, Leg.RL) else loaded_diagonal_offset_mm
        lifted = unload_mm if leg in (Leg.FL, Leg.RR) and not replant else 0.0
        result[leg] = (x_mm + offset, down_mm - lifted)
    return result


def phase_endpoint_targets(parameters: TurnParameters) -> dict[str, dict[Leg, tuple[float, float]]]:
    """Return the canonical right-turn endpoint targets in execution order."""
    if not isinstance(parameters, TurnParameters):
        raise ValueError("parameters must be TurnParameters")
    stand = _endpoints()
    shifted = _endpoints(unload_mm=parameters.unload_mm)
    drive = _endpoints(
        unloaded_diagonal_offset_mm=-parameters.tangential_mm,
        loaded_diagonal_offset_mm=parameters.tangential_mm,
        unload_mm=parameters.unload_mm,
    )
    replant = _endpoints(
        unloaded_diagonal_offset_mm=-parameters.tangential_mm,
        loaded_diagonal_offset_mm=parameters.tangential_mm,
        replant=True,
    )
    return {
        "STAND": stand,
        "SETTLE": dict(stand),
        "SHIFT_UNLOAD": shifted,
        "DRIVE_TURN": drive,
        "REPLANT": replant,
        "RECOVER": dict(stand),
    }


def _contacts_from_points(
    points: Iterable[Sequence[object]], foot_links: Mapping[Leg | str, int]
) -> tuple[FootContact, ...]:
    normalized_links = {
        leg: foot_links.get(leg, foot_links.get(leg.value))
        for leg in Leg
    }
    if any(index is None for index in normalized_links.values()):
        raise ValueError("foot_links must map every canonical leg")
    forces = {leg: 0.0 for leg in Leg}
    link_to_leg = {int(index): leg for leg, index in normalized_links.items() if index is not None}
    for point in points:
        if len(point) <= 9:
            raise ValueError("Bullet contact tuples must include normal force at index 9")
        link_index = point[3]
        if isinstance(link_index, bool) or not isinstance(link_index, int):
            continue
        leg = link_to_leg.get(link_index)
        if leg is not None:
            forces[leg] += _finite(point[9], "normal_force_n", nonnegative=True)
    return tuple(FootContact(leg.value, forces[leg] > 0.0, forces[leg]) for leg in Leg)


def contact_snapshot(
    client: object,
    body_id: int,
    plane_id: int,
    foot_links: Mapping[Leg | str, int],
) -> tuple[FootContact, ...]:
    """Aggregate one Bullet contact query into canonical per-foot forces."""
    points = client.getContactPoints(bodyA=body_id, bodyB=plane_id)  # type: ignore[attr-defined]
    return _contacts_from_points(points, foot_links)


def _joint_targets(endpoints: Mapping[Leg, tuple[float, float]]) -> dict[str, float]:
    targets: dict[str, float] = {}
    for leg in Leg:
        hip_deg, knee_deg = inverse_kinematics_deg(*endpoints[leg])
        hip_name, knee_name = _JOINT_NAMES[leg]
        targets[hip_name] = math.radians(hip_deg)
        targets[knee_name] = math.radians(knee_deg)
    return targets


def _decoded(value: object) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _discover_indices(client: object, body_id: int) -> tuple[dict[str, int], dict[Leg, int]]:
    joint_indices: dict[str, int] = {}
    links_by_name: dict[str, int] = {}
    for index in range(client.getNumJoints(body_id)):  # type: ignore[attr-defined]
        info = client.getJointInfo(body_id, index)  # type: ignore[attr-defined]
        if len(info) <= 12:
            raise ValueError("Bullet joint info must include link name at index 12")
        joint_indices[_decoded(info[1])] = index
        links_by_name[_decoded(info[12])] = index
    required_joints = {name for pair in _JOINT_NAMES.values() for name in pair}
    missing_joints = required_joints - joint_indices.keys()
    if missing_joints:
        raise ValueError(f"model is missing joints: {', '.join(sorted(missing_joints))}")
    missing_links = set(_FOOT_LINK_NAMES.values()) - links_by_name.keys()
    if missing_links:
        raise ValueError(f"model is missing foot links: {', '.join(sorted(missing_links))}")
    return joint_indices, {leg: links_by_name[name] for leg, name in _FOOT_LINK_NAMES.items()}


def _quaternion_from_roll_deg(roll_deg: float) -> tuple[float, float, float, float]:
    half_roll = math.radians(roll_deg) / 2.0
    return (math.sin(half_roll), 0.0, 0.0, math.cos(half_roll))


def _euler_deg(quaternion_xyzw: Sequence[object]) -> tuple[float, float, float]:
    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion must contain four values")
    x, y, z, w = (_finite(value, "quaternion") for value in quaternion_xyzw)
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude == 0.0:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = (value / magnitude for value in (x, y, z, w))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    return math.degrees(roll), math.degrees(pitch), yaw_from_quaternion((x, y, z, w))


def _safe_contacts() -> tuple[FootContact, ...]:
    return tuple(FootContact(leg.value, False, 0.0) for leg in Leg)


def _invalid_result(
    parameters: TurnParameters,
    surface: Surface,
    settings: SimulationSettings,
    reason: str,
    elapsed_s: float,
    pose: tuple[Sequence[object], Sequence[object]] | None = None,
    contacts: tuple[FootContact, ...] | None = None,
    *,
    fell: bool = False,
) -> SimulationResult:
    fallback_position = (0.0, 0.0, settings.spawn_height_m)
    fallback_quaternion = (0.0, 0.0, 0.0, 1.0)
    try:
        if pose is None:
            raise ValueError("no measured pose")
        position, quaternion = pose
        position_xyz = tuple(_finite(value, "position_xyz") for value in position)
        quaternion_xyzw = tuple(_finite(value, "quaternion_xyzw") for value in quaternion)
        if len(position_xyz) != 3:
            raise ValueError("position_xyz must contain three values")
        roll, pitch, yaw = _euler_deg(quaternion_xyzw)
        final_pose = FinalPose(position_xyz, quaternion_xyzw, roll, pitch, yaw)  # type: ignore[arg-type]
    except (IndexError, TypeError, ValueError):
        roll, pitch, yaw = _euler_deg(fallback_quaternion)
        final_pose = FinalPose(
            fallback_position,
            fallback_quaternion,
            roll,
            pitch,
            yaw,
        )
    return SimulationResult(
        parameters=parameters,
        surface_name=surface.name,
        friction=surface.friction,
        yaw_delta_deg=0.0,
        translation_x_m=0.0,
        translation_y_m=0.0,
        translation_m=0.0,
        max_roll_deviation_deg=0.0,
        max_pitch_deviation_deg=0.0,
        fell=fell,
        contact_instability=1.0,
        elapsed_sim_s=elapsed_s,
        final_pose=final_pose,
        foot_contacts=contacts or _safe_contacts(),
        aborted=True,
        invalid_reason=reason,
    )


def run_candidate(
    parameters: TurnParameters,
    surface: Surface,
    settings: SimulationSettings,
    client_factory: Callable[[], object],
) -> SimulationResult:
    """Run one candidate using only an injected Bullet-compatible client."""
    if not isinstance(parameters, TurnParameters):
        raise ValueError("parameters must be TurnParameters")
    if not isinstance(surface, Surface):
        raise ValueError("surface must be Surface")
    if not isinstance(settings, SimulationSettings):
        raise ValueError("settings must be SimulationSettings")

    client: object | None = None
    elapsed_s = 0.0
    last_pose: tuple[Sequence[object], Sequence[object]] | None = None
    last_contacts = _safe_contacts()
    try:
        client = client_factory()
        client.resetSimulation()  # type: ignore[attr-defined]
        client.setTimeStep(settings.time_step_s)  # type: ignore[attr-defined]
        client.setPhysicsEngineParameter(numSolverIterations=settings.solver_iterations)  # type: ignore[attr-defined]
        client.setGravity(*settings.gravity_xyz)  # type: ignore[attr-defined]
        plane_id = client.loadURDF("plane.urdf", useFixedBase=True)  # type: ignore[attr-defined]
        body_id = client.loadURDF(settings.model_path)  # type: ignore[attr-defined]
        joint_indices, foot_links = _discover_indices(client, body_id)
        client.changeDynamics(plane_id, -1, lateralFriction=surface.friction)  # type: ignore[attr-defined]
        for link_index in foot_links.values():
            client.changeDynamics(body_id, link_index, lateralFriction=surface.friction)  # type: ignore[attr-defined]

        initial_quaternion = (0.0, 0.0, 0.0, 1.0)
        client.resetBasePositionAndOrientation(  # type: ignore[attr-defined]
            body_id,
            (0.0, 0.0, settings.spawn_height_m),
            initial_quaternion,
        )

        phases = phase_endpoint_targets(parameters)
        phase_joints = {phase: _joint_targets(endpoints) for phase, endpoints in phases.items()}
        current_targets = dict(phase_joints["STAND"])
        for joint_name, target in current_targets.items():
            client.resetJointState(body_id, joint_indices[joint_name], targetValue=target)  # type: ignore[attr-defined]

        support_loss_s = 0.0
        unstable_samples = 0
        measured_samples = 0
        max_roll_deviation = 0.0
        max_pitch_deviation = 0.0
        baseline_roll: float | None = None
        baseline_pitch: float | None = None
        safety_roll_reference: float | None = None
        safety_pitch_reference: float | None = None
        baseline_pose: tuple[Sequence[object], Sequence[object]] | None = None
        fell = False

        def sample() -> None:
            nonlocal last_pose, last_contacts, support_loss_s, unstable_samples
            nonlocal measured_samples, max_roll_deviation, max_pitch_deviation, fell
            nonlocal safety_roll_reference, safety_pitch_reference
            position, orientation = client.getBasePositionAndOrientation(body_id)  # type: ignore[attr-defined]
            position = tuple(_finite(value, "base position") for value in position)
            orientation = tuple(_finite(value, "base quaternion") for value in orientation)
            last_pose = (position, orientation)
            points = tuple(client.getContactPoints(bodyA=body_id, bodyB=plane_id))  # type: ignore[attr-defined]
            last_contacts = _contacts_from_points(points, foot_links)
            supported_feet = sum(contact.in_contact for contact in last_contacts)
            support_loss_s = support_loss_s + settings.time_step_s if supported_feet < 2 else 0.0
            roll, pitch, _ = _euler_deg(orientation)
            if safety_roll_reference is None or safety_pitch_reference is None:
                safety_roll_reference, safety_pitch_reference = roll, pitch
            torso_contact = any(
                len(point) > 9
                and point[3] == -1
                and _finite(point[9], "normal_force_n", nonnegative=True) > 0.0
                for point in points
            )
            if baseline_roll is not None and baseline_pitch is not None:
                measured_samples += 1
                if supported_feet < 3:
                    unstable_samples += 1
                max_roll_deviation = max(max_roll_deviation, abs(roll - baseline_roll))
                max_pitch_deviation = max(max_pitch_deviation, abs(pitch - baseline_pitch))
            roll_reference = baseline_roll if baseline_roll is not None else safety_roll_reference
            pitch_reference = baseline_pitch if baseline_pitch is not None else safety_pitch_reference
            fell = detect_fall(
                corrected_roll_deg=roll - roll_reference,
                pitch_deviation_deg=pitch - pitch_reference,
                height_m=position[2],
                torso_contact=torso_contact,
                supported_feet=supported_feet,
                support_loss_duration_s=support_loss_s,
                fall_roll_deg=settings.fall_roll_deg,
                fall_pitch_deg=settings.fall_pitch_deg,
                min_height_m=settings.min_height_m,
                max_support_loss_duration_s=settings.support_loss_duration_s,
            )

        def drive(targets: Mapping[str, float], duration_s: float) -> None:
            nonlocal current_targets, elapsed_s
            steps = max(1, math.ceil(duration_s / settings.time_step_s - 1e-12))
            start = dict(current_targets)
            for step in range(1, steps + 1):
                fraction = step / steps
                for joint_name in start:
                    target = start[joint_name] + (targets[joint_name] - start[joint_name]) * fraction
                    client.setJointMotorControl2(  # type: ignore[attr-defined]
                        bodyUniqueId=body_id,
                        jointIndex=joint_indices[joint_name],
                        controlMode=client.POSITION_CONTROL,  # type: ignore[attr-defined]
                        targetPosition=target,
                        maxVelocity=math.radians(parameters.speed),
                    )
                client.stepSimulation()  # type: ignore[attr-defined]
                elapsed_s += settings.time_step_s
                sample()
                if fell:
                    break
            current_targets = dict(targets)

        drive(phase_joints["STAND"], settings.initial_settle_s)
        if not fell:
            drive(phase_joints["SETTLE"], parameters.settle_s)
        if fell:
            return _invalid_result(
                parameters,
                surface,
                settings,
                "fall-detected",
                elapsed_s,
                last_pose,
                last_contacts,
                fell=True,
            )

        baseline_pose = client.getBasePositionAndOrientation(body_id)  # type: ignore[attr-defined]
        baseline_position, baseline_orientation = baseline_pose
        baseline_roll, baseline_pitch, _ = _euler_deg(baseline_orientation)
        last_pose = (baseline_position, baseline_orientation)

        durations = {
            "SHIFT_UNLOAD": parameters.settle_s,
            "DRIVE_TURN": parameters.settle_s,
            "REPLANT": parameters.replant_s,
            "RECOVER": parameters.settle_s,
        }
        for _cycle in range(parameters.cycles):
            for phase in PHASES[2:]:
                drive(phase_joints[phase], durations[phase])
                if phase == "DRIVE_TURN" and not fell:
                    drive(phase_joints[phase], parameters.hold_s)
                if fell:
                    break
            if fell:
                break

        if last_pose is None or baseline_pose is None:
            raise RuntimeError("simulation produced no base pose")
        final_position, final_orientation = last_pose
        initial_position, initial_orientation = baseline_pose
        translation_x = float(final_position[0]) - float(initial_position[0])
        translation_y = float(final_position[1]) - float(initial_position[1])
        roll, pitch, yaw = _euler_deg(final_orientation)
        return SimulationResult(
            parameters=parameters,
            surface_name=surface.name,
            friction=surface.friction,
            yaw_delta_deg=yaw_delta_from_quaternions(initial_orientation, final_orientation),
            translation_x_m=translation_x,
            translation_y_m=translation_y,
            translation_m=math.hypot(translation_x, translation_y),
            max_roll_deviation_deg=max_roll_deviation,
            max_pitch_deviation_deg=max_pitch_deviation,
            fell=fell,
            contact_instability=(unstable_samples / measured_samples if measured_samples else 1.0),
            elapsed_sim_s=elapsed_s,
            final_pose=FinalPose(tuple(final_position), tuple(final_orientation), roll, pitch, yaw),  # type: ignore[arg-type]
            foot_contacts=last_contacts,
            aborted=fell,
            invalid_reason="fall-detected" if fell else None,
        )
    except Exception as error:
        return _invalid_result(
            parameters,
            surface,
            settings,
            f"{type(error).__name__}: {error}",
            elapsed_s,
            last_pose,
            last_contacts,
        )
    finally:
        if client is not None and hasattr(client, "disconnect"):
            try:
                client.disconnect()  # type: ignore[attr-defined]
            except Exception:
                pass


def _result_mapping(result: SimulationResult) -> dict[str, object]:
    return asdict(result)


class _DirectBulletClient:
    """Bind PyBullet calls to one quiet DIRECT connection."""

    def __init__(self, module: object) -> None:
        self._module = module
        self._client_id = module.connect(module.DIRECT)  # type: ignore[attr-defined]
        if self._client_id < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")

    def __getattr__(self, name: str) -> object:
        attribute = getattr(self._module, name)
        if inspect.isbuiltin(attribute):
            return functools.partial(attribute, physicsClientId=self._client_id)
        return attribute

    def disconnect(self) -> None:
        if self._client_id >= 0:
            self._module.disconnect(physicsClientId=self._client_id)  # type: ignore[attr-defined]
            self._client_id = -1


def direct_client_factory() -> object:
    """Create an isolated, deterministic PyBullet client in DIRECT mode."""
    try:
        import pybullet
        import pybullet_data
    except ImportError as error:
        raise RuntimeError("PyBullet is required for DIRECT simulation") from error

    client = _DirectBulletClient(pybullet)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    client.setRealTimeSimulation(0)
    return client


def main(argv: Sequence[str] | None = None) -> int:
    """Load one candidate and emit one structured JSON result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/turn_right_baseline.json")
    parser.add_argument("--simulation-config", default="config/simulation.json")
    parser.add_argument("--surface", choices=[surface.name for surface in required_surfaces()], default="nominal")
    parser.add_argument("--smoke", action="store_true", help="run one headless DIRECT smoke simulation")
    args = parser.parse_args(argv)
    parameters = load_turn_parameters(args.config)
    settings = load_simulation_settings(args.simulation_config)
    surface = next(item for item in required_surfaces() if item.name == args.surface)
    result = run_candidate(parameters, surface, settings, direct_client_factory)
    print(json.dumps(_result_mapping(result), sort_keys=True, separators=(",", ":")))
    return 0 if result.invalid_reason is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
