"""Generic deterministic executor for pure simulated turn primitives."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Mapping

from pip_robot.turn.models import TurnParameters

from .contact_mechanics import aggregate_foot_mechanics
from .diagnostics import DiagnosticTrace, FootTraceSample, PhaseTraceSample
from .kinematics import Leg, inverse_kinematics_deg
from .model import FootContact, FinalPose, SimulationResult, detect_fall, yaw_delta_from_quaternions
from .pivot_runner import (
    SimulationSettings,
    _discover_indices,
    _euler_deg,
    _invalid_result,
    _JOINT_NAMES,
)
from .surfaces import Surface
from .turn_primitives.base import FootTarget, TurnPrimitive, validate_actions


@dataclass(frozen=True)
class InstrumentedRun:
    result: SimulationResult
    trace: DiagnosticTrace | None


def _joint_targets(targets: Mapping[Leg, FootTarget]) -> dict[str, float]:
    result: dict[str, float] = {}
    for leg in Leg:
        hip_deg, knee_deg = inverse_kinematics_deg(targets[leg].x_mm, targets[leg].down_mm)
        hip_name, knee_name = _JOINT_NAMES[leg]
        result[hip_name] = math.radians(hip_deg)
        result[knee_name] = math.radians(knee_deg)
    return result


def run_primitive(
    primitive: TurnPrimitive[TurnParameters],
    parameters: TurnParameters,
    surface: Surface,
    settings: SimulationSettings,
    client_factory: Callable[[], object],
    *,
    run_id: str,
    candidate_id: str = "baseline",
) -> InstrumentedRun:
    """Execute a pure primitive and return its physical result plus trace."""
    actions = validate_actions(primitive.build_actions(parameters))
    client: object | None = None
    elapsed_s = 0.0
    samples: list[PhaseTraceSample] = []
    last_pose = None
    last_contacts = None
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
        client.resetBasePositionAndOrientation(  # type: ignore[attr-defined]
            body_id, (0.0, 0.0, settings.spawn_height_m), (0.0, 0.0, 0.0, 1.0)
        )

        action_joints = [(action, _joint_targets(action.targets)) for action in actions]
        current_targets = dict(action_joints[0][1])
        for joint_name, target in current_targets.items():
            client.resetJointState(body_id, joint_indices[joint_name], targetValue=target)  # type: ignore[attr-defined]

        baseline_pose = None
        baseline_roll = baseline_pitch = None
        safety_roll_reference = safety_pitch_reference = None
        support_loss_s = 0.0
        unstable_samples = measured_samples = 0
        max_roll = max_pitch = 0.0
        fell = False
        previous_foot_positions: dict[Leg, tuple[float, float, float]] = {}
        previous_contacts: dict[Leg, bool] = {leg: False for leg in Leg}
        cumulative_slip = {leg: 0.0 for leg in Leg}

        def sample(phase: str) -> None:
            nonlocal last_pose, last_contacts, support_loss_s, unstable_samples, measured_samples
            nonlocal max_roll, max_pitch, fell, safety_roll_reference, safety_pitch_reference
            position, orientation = client.getBasePositionAndOrientation(body_id)  # type: ignore[attr-defined]
            position = tuple(float(value) for value in position)
            orientation = tuple(float(value) for value in orientation)
            last_pose = (position, orientation)
            if hasattr(client, "getBaseVelocity"):
                linear_velocity, angular_velocity = client.getBaseVelocity(body_id)  # type: ignore[attr-defined]
            else:
                linear_velocity = angular_velocity = (0.0, 0.0, 0.0)
            points = tuple(client.getContactPoints(bodyA=body_id, bodyB=plane_id))  # type: ignore[attr-defined]
            if points and all(len(point) <= 10 for point in points):
                from .contact_mechanics import FootMechanics
                normal_by_leg = {leg: 0.0 for leg in Leg}
                link_to_leg = {index: leg for leg, index in foot_links.items()}
                torso_contact = False
                for point in points:
                    if point[3] == -1 and float(point[9]) > 0.0:
                        torso_contact = True
                    elif point[3] in link_to_leg:
                        normal_by_leg[link_to_leg[point[3]]] += float(point[9])
                mechanics = tuple(
                    FootMechanics(leg.value, normal_by_leg[leg] > 0.0, normal_by_leg[leg], None, None, None, None)
                    for leg in Leg
                )
            else:
                mechanics, torso_contact = aggregate_foot_mechanics(points, foot_links, position)
            last_contacts = mechanics
            support_count = sum(item.in_contact for item in mechanics)
            support_loss_s = support_loss_s + settings.time_step_s if support_count < 2 else 0.0
            roll, pitch, bullet_yaw = _euler_deg(orientation)
            if safety_roll_reference is None or safety_pitch_reference is None:
                safety_roll_reference, safety_pitch_reference = roll, pitch
            if baseline_roll is not None and baseline_pitch is not None:
                measured_samples += 1
                if support_count < 3:
                    unstable_samples += 1
                max_roll = max(max_roll, abs(roll - baseline_roll))
                max_pitch = max(max_pitch, abs(pitch - baseline_pitch))
            roll_reference = baseline_roll if baseline_roll is not None else safety_roll_reference
            pitch_reference = baseline_pitch if baseline_pitch is not None else safety_pitch_reference
            fell = detect_fall(
                corrected_roll_deg=roll - roll_reference,
                pitch_deviation_deg=pitch - pitch_reference,
                height_m=position[2],
                torso_contact=torso_contact,
                supported_feet=support_count,
                support_loss_duration_s=support_loss_s,
                fall_roll_deg=settings.fall_roll_deg,
                fall_pitch_deg=settings.fall_pitch_deg,
                min_height_m=settings.min_height_m,
                max_support_loss_duration_s=settings.support_loss_duration_s,
            )
            foot_samples = []
            for leg, item in zip(Leg, mechanics):
                if item.in_contact and item.contact_position_xyz_m is not None:
                    foot_position = item.contact_position_xyz_m
                elif hasattr(client, "getLinkState"):
                    link_state = client.getLinkState(body_id, foot_links[leg], computeLinkVelocity=1)  # type: ignore[attr-defined]
                    foot_position = tuple(float(value) for value in link_state[0])
                else:
                    foot_position = previous_foot_positions.get(leg, (0.0, 0.0, 0.0))
                prior = previous_foot_positions.get(leg)
                displacement = None if prior is None else tuple(now - old for now, old in zip(foot_position, prior))
                if displacement is not None and item.in_contact and previous_contacts[leg]:
                    cumulative_slip[leg] += math.sqrt(sum(value * value for value in displacement[:2]))
                previous_foot_positions[leg] = foot_position
                previous_contacts[leg] = item.in_contact
                foot_samples.append(
                    FootTraceSample(
                        leg=leg.value,
                        in_contact=item.in_contact,
                        normal_force_n=item.normal_force_n,
                        tangential_force_xyz_n=item.tangential_force_xyz_n,
                        world_position_xyz_m=foot_position,
                        displacement_xyz_m=displacement,
                        cumulative_slip_m=cumulative_slip[leg],
                        bullet_tau_z_nm=item.bullet_tau_z_nm,
                        right_yaw_torque_nm=item.right_yaw_torque_nm,
                    )
                )
            contacting = [item for item in mechanics if item.in_contact]
            native_torques = [item.bullet_tau_z_nm for item in contacting]
            right_torques = [item.right_yaw_torque_nm for item in contacting]
            samples.append(
                PhaseTraceSample(
                    phase=phase,
                    elapsed_s=elapsed_s,
                    body_yaw_deg=-bullet_yaw,
                    body_roll_deg=roll,
                    body_pitch_deg=pitch,
                    body_position_xyz_m=position,
                    angular_velocity_xyz_rad_s=tuple(float(v) for v in angular_velocity),
                    linear_velocity_xyz_m_s=tuple(float(v) for v in linear_velocity),
                    feet=tuple(foot_samples),
                    support_foot_count=support_count,
                    total_contact_normal_force_n=sum(item.normal_force_n for item in mechanics),
                    total_bullet_tau_z_nm=(sum(value for value in native_torques if value is not None) if all(value is not None for value in native_torques) else None),
                    total_right_yaw_torque_nm=(sum(value for value in right_torques if value is not None) if all(value is not None for value in right_torques) else None),
                    torso_contact=torso_contact,
                )
            )

        for index, (action, targets) in enumerate(action_joints):
            transition_s = settings.initial_settle_s if index == 0 and action.name == "STAND" else action.duration_s
            steps = max(1, math.ceil(transition_s / settings.time_step_s - 1e-12))
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
                sample(action.name)
                if fell:
                    break
            current_targets = dict(targets)
            if not fell and action.hold_s > 0.0:
                hold_steps = max(1, math.ceil(action.hold_s / settings.time_step_s - 1e-12))
                for _ in range(hold_steps):
                    for joint_name, target in targets.items():
                        client.setJointMotorControl2(  # type: ignore[attr-defined]
                            bodyUniqueId=body_id,
                            jointIndex=joint_indices[joint_name],
                            controlMode=client.POSITION_CONTROL,  # type: ignore[attr-defined]
                            targetPosition=target,
                            maxVelocity=math.radians(parameters.speed),
                        )
                    client.stepSimulation()  # type: ignore[attr-defined]
                    elapsed_s += settings.time_step_s
                    sample(action.name)
                    if fell:
                        break
            if index == 1 and not fell:
                baseline_pose = last_pose
                baseline_roll, baseline_pitch, _ = _euler_deg(baseline_pose[1])
            if fell:
                break

        if fell and baseline_pose is None:
            result = _invalid_result(parameters, surface, settings, "fall-detected", elapsed_s, last_pose, fell=True)
            trace = DiagnosticTrace(1, run_id, primitive.family, candidate_id, surface.name, tuple(samples))
            return InstrumentedRun(result, trace)
        if last_pose is None or baseline_pose is None:
            raise RuntimeError("simulation produced no baseline/final pose")
        final_position, final_orientation = last_pose
        initial_position, initial_orientation = baseline_pose
        translation_x = final_position[0] - initial_position[0]
        translation_y = final_position[1] - initial_position[1]
        roll, pitch, yaw = _euler_deg(final_orientation)
        result = SimulationResult(
            parameters=parameters,
            surface_name=surface.name,
            friction=surface.friction,
            yaw_delta_deg=-yaw_delta_from_quaternions(initial_orientation, final_orientation),
            translation_x_m=translation_x,
            translation_y_m=translation_y,
            translation_m=math.hypot(translation_x, translation_y),
            max_roll_deviation_deg=max_roll,
            max_pitch_deviation_deg=max_pitch,
            fell=fell,
            contact_instability=(unstable_samples / measured_samples if measured_samples else 1.0),
            elapsed_sim_s=elapsed_s,
            final_pose=FinalPose(tuple(final_position), tuple(final_orientation), roll, pitch, yaw),
            foot_contacts=tuple(
                FootContact(item.leg, item.in_contact, item.normal_force_n)
                for item in last_contacts
            ),
            aborted=fell,
            invalid_reason="fall-detected" if fell else None,
        )
        trace = DiagnosticTrace(1, run_id, primitive.family, candidate_id, surface.name, tuple(samples))
        return InstrumentedRun(result, trace)
    except Exception as error:
        result = _invalid_result(parameters, surface, settings, f"{type(error).__name__}: {error}", elapsed_s)
        return InstrumentedRun(result, None)
    finally:
        if client is not None and hasattr(client, "disconnect"):
            try:
                client.disconnect()  # type: ignore[attr-defined]
            except Exception:
                pass
