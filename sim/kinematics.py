"""Calibrated planar leg kinematics for the copied PiP URDF."""

from __future__ import annotations

from enum import Enum
import math
from numbers import Real


class Leg(str, Enum):
    """The PiP legs in the simulator's stable order."""

    FL = "FL"
    FR = "FR"
    RL = "RL"
    RR = "RR"


UPPER_LINK_MM = 40.0
LOWER_LINK_MM = 80.0

factory_stance_mm = {
    Leg.FL: (-15.0, 95.0),
    Leg.FR: (-15.0, 95.0),
    Leg.RL: (5.0, 90.0),
    Leg.RR: (5.0, 90.0),
}

_JOINT_NAMES = {
    Leg.FL: ("P2_FL_HIP", "P3_FL_KNEE"),
    Leg.FR: ("P7_FR_HIP", "P8_FR_KNEE"),
    Leg.RL: ("P0_RL_HIP", "P1_RL_KNEE"),
    Leg.RR: ("P10_RR_HIP", "P11_RR_KNEE"),
}


def _finite_mm(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def inverse_kinematics_deg(x_mm: object, down_mm: object) -> tuple[float, float]:
    """Return hip and inward-bending knee angles for a planar foot target.

    ``x_mm`` is lateral displacement in the hip plane and ``down_mm`` is the
    positive-down vertical displacement.  The selected elbow branch keeps the
    PiP knees negative, matching the copied URDF's revolute axes.
    """

    x = _finite_mm(x_mm, "x_mm")
    down = _finite_mm(down_mm, "down_mm")
    distance = math.hypot(x, down)
    minimum = abs(LOWER_LINK_MM - UPPER_LINK_MM)
    maximum = LOWER_LINK_MM + UPPER_LINK_MM
    if distance < minimum or distance > maximum:
        raise ValueError("unreachable endpoint")

    cosine_knee = (
        distance * distance - UPPER_LINK_MM * UPPER_LINK_MM - LOWER_LINK_MM * LOWER_LINK_MM
    ) / (2.0 * UPPER_LINK_MM * LOWER_LINK_MM)
    cosine_knee = max(-1.0, min(1.0, cosine_knee))
    knee_rad = -math.acos(cosine_knee)
    hip_rad = math.atan2(x, down) - math.atan2(
        LOWER_LINK_MM * math.sin(knee_rad),
        UPPER_LINK_MM + LOWER_LINK_MM * math.cos(knee_rad),
    )
    return math.degrees(hip_rad), math.degrees(knee_rad)


def forward_kinematics_mm(hip_deg: object, knee_deg: object) -> tuple[float, float]:
    """Return the planar ``(x, down)`` foot position for calibrated angles."""

    hip = math.radians(_finite_mm(hip_deg, "hip_deg"))
    knee = math.radians(_finite_mm(knee_deg, "knee_deg"))
    return (
        UPPER_LINK_MM * math.sin(hip) + LOWER_LINK_MM * math.sin(hip + knee),
        UPPER_LINK_MM * math.cos(hip) + LOWER_LINK_MM * math.cos(hip + knee),
    )


leg_angles_deg = {leg: inverse_kinematics_deg(*position) for leg, position in factory_stance_mm.items()}

stance_joint_targets_rad: dict[str, float] = {}
for _leg in Leg:
    _hip_name, _knee_name = _JOINT_NAMES[_leg]
    _hip_deg, _knee_deg = leg_angles_deg[_leg]
    stance_joint_targets_rad[_hip_name] = math.radians(_hip_deg)
    stance_joint_targets_rad[_knee_name] = math.radians(_knee_deg)
