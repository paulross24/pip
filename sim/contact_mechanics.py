"""Decode PyBullet contacts and calculate contact-induced yaw moments.

PyBullet contact tuples use link A at index 3, world contact position A at 5,
normal on B at 7, normal magnitude at 9, and two lateral magnitude/direction
pairs at 10/11 and 12/13.  The normal points from body B (ground) toward body A
(PiP), so the reconstructed values are ground-reaction forces on PiP.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Iterable, Mapping, Sequence

from .kinematics import Leg


Vector3 = tuple[float, float, float]


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number")
    return float(value)


def _vector3(value: object, field: str) -> Vector3:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must contain three finite numbers")
    try:
        values = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise ValueError(f"{field} must contain three finite numbers") from error
    if len(values) != 3:
        raise ValueError(f"{field} must contain three finite numbers")
    return tuple(_finite(item, field) for item in values)  # type: ignore[return-value]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _scale(vector: Vector3, magnitude: float) -> Vector3:
    return tuple(item * magnitude for item in vector)  # type: ignore[return-value]


@dataclass(frozen=True)
class ContactForce:
    normal_xyz_n: Vector3
    tangential_xyz_n: Vector3 | None
    total_xyz_n: Vector3 | None


@dataclass(frozen=True)
class FootMechanics:
    leg: str
    in_contact: bool
    normal_force_n: float
    tangential_force_xyz_n: Vector3 | None
    contact_position_xyz_m: Vector3 | None
    bullet_tau_z_nm: float | None
    right_yaw_torque_nm: float | None


def reconstruct_contact_force(point: Sequence[object]) -> ContactForce:
    """Reconstruct the ground-reaction force on body A from one Bullet point."""
    if len(point) <= 9:
        raise ValueError("Bullet contact must include normal fields through index 9")
    normal = _vector3(point[7], "contact_normal_on_b")
    normal_force = _finite(point[9], "normal_force_n")
    if normal_force < 0.0:
        raise ValueError("normal_force_n must be nonnegative")
    normal_xyz = _scale(normal, normal_force)
    if len(point) <= 13:
        return ContactForce(normal_xyz, None, None)
    magnitude_1 = _finite(point[10], "lateral_friction_1_n")
    direction_1 = _vector3(point[11], "lateral_direction_1")
    magnitude_2 = _finite(point[12], "lateral_friction_2_n")
    direction_2 = _vector3(point[13], "lateral_direction_2")
    tangential = _add(_scale(direction_1, magnitude_1), _scale(direction_2, magnitude_2))
    return ContactForce(normal_xyz, tangential, _add(normal_xyz, tangential))


def yaw_moment_z_nm(
    base_xy: Sequence[object], contact_xy: Sequence[object], force_xy: Sequence[object]
) -> tuple[float, float]:
    """Return native Bullet (left-positive) and PiP right-positive yaw moments."""
    if len(base_xy) != 2 or len(contact_xy) != 2 or len(force_xy) != 2:
        raise ValueError("yaw moment inputs must be planar pairs")
    bx, by = (_finite(item, "base_xy") for item in base_xy)
    px, py = (_finite(item, "contact_xy") for item in contact_xy)
    fx, fy = (_finite(item, "force_xy") for item in force_xy)
    native = (px - bx) * fy - (py - by) * fx
    return native, -native


def aggregate_foot_mechanics(
    points: Iterable[Sequence[object]],
    foot_links: Mapping[Leg | str, int],
    base_position: Sequence[object],
) -> tuple[tuple[FootMechanics, ...], bool]:
    """Aggregate all contact points into canonical per-foot mechanics."""
    base = _vector3(base_position, "base_position")
    canonical = {leg: foot_links.get(leg, foot_links.get(leg.value)) for leg in Leg}
    if any(index is None for index in canonical.values()):
        raise ValueError("foot_links must map every canonical leg")
    link_to_leg = {int(index): leg for leg, index in canonical.items() if index is not None}
    collected: dict[Leg, list[tuple[Sequence[object], ContactForce]]] = {leg: [] for leg in Leg}
    torso_contact = False
    for point in points:
        if len(point) <= 9:
            raise ValueError("Bullet contact must include normal fields through index 9")
        link = point[3]
        if link == -1 and _finite(point[9], "normal_force_n") > 0.0:
            torso_contact = True
            continue
        if isinstance(link, bool) or not isinstance(link, int) or link not in link_to_leg:
            continue
        force = reconstruct_contact_force(point)
        if _finite(point[9], "normal_force_n") > 0.0:
            collected[link_to_leg[link]].append((point, force))

    result: list[FootMechanics] = []
    for leg in Leg:
        contacts = collected[leg]
        if not contacts:
            result.append(FootMechanics(leg.value, False, 0.0, None, None, None, None))
            continue
        normal_force = sum(_finite(point[9], "normal_force_n") for point, _ in contacts)
        positions = [_vector3(point[5], "contact_position_on_a") for point, _ in contacts]
        if normal_force > 0.0:
            position = tuple(
                sum(pos[axis] * _finite(point[9], "normal_force_n") for (point, _), pos in zip(contacts, positions))
                / normal_force
                for axis in range(3)
            )
        else:
            position = positions[0]
        tangents = [force.tangential_xyz_n for _, force in contacts]
        if any(force is None for force in tangents):
            tangent = None
            native = right = None
        else:
            tangent = (0.0, 0.0, 0.0)
            native = 0.0
            for (point, _force), component in zip(contacts, tangents):
                assert component is not None
                tangent = _add(tangent, component)
                point_position = _vector3(point[5], "contact_position_on_a")
                point_native, _ = yaw_moment_z_nm(base[:2], point_position[:2], component[:2])
                native += point_native
            right = -native
        result.append(FootMechanics(leg.value, True, normal_force, tangent, position, native, right))
    return tuple(result), torso_contact
