"""Immutable, validated measurements produced by a turn simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Iterable

from pip_robot.turn.heading import signed_heading_delta_deg
from pip_robot.turn.models import HeadingObservation, TurnParameters

from .surfaces import required_surfaces


_FEET = frozenset(("FL", "FR", "RL", "RR"))
_CANONICAL_SURFACE_FRICTION = {
    surface.name: surface.friction for surface in required_surfaces()
}


def _finite_float(value: object, field: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number")
    number = float(value)
    if minimum is not None and number < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    return number


def _finite_vector(value: object, field: str, size: int) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must contain {size} finite numbers")
    try:
        values = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise ValueError(f"{field} must contain {size} finite numbers") from error
    if len(values) != size:
        raise ValueError(f"{field} must contain {size} finite numbers")
    return tuple(_finite_float(item, field) for item in values)


@dataclass(frozen=True)
class FootContact:
    """One lower-foot contact snapshot measured against the plane."""

    foot: str
    in_contact: bool
    normal_force_n: float

    def __post_init__(self) -> None:
        if self.foot not in _FEET:
            raise ValueError("foot must be one of FL, FR, RL, or RR")
        if not isinstance(self.in_contact, bool):
            raise ValueError("in_contact must be a boolean")
        object.__setattr__(self, "normal_force_n", _finite_float(self.normal_force_n, "normal_force_n", minimum=0.0))


@dataclass(frozen=True)
class FinalPose:
    """Measured base pose at the end of a simulation."""

    position_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    def __post_init__(self) -> None:
        position = _finite_vector(self.position_xyz, "position_xyz", 3)
        quaternion = _finite_vector(self.quaternion_xyzw, "quaternion_xyzw", 4)
        if math.isclose(sum(component * component for component in quaternion), 0.0, abs_tol=1e-15):
            raise ValueError("quaternion_xyzw must not be the zero quaternion")
        object.__setattr__(self, "position_xyz", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        for field in ("roll_deg", "pitch_deg", "yaw_deg"):
            object.__setattr__(self, field, _finite_float(getattr(self, field), field))


@dataclass(frozen=True)
class SimulationResult:
    """All objective measurements collected for one parameter/surface run."""

    parameters: TurnParameters
    surface_name: str
    friction: float
    yaw_delta_deg: float
    translation_x_m: float
    translation_y_m: float
    translation_m: float
    max_roll_deviation_deg: float
    max_pitch_deviation_deg: float
    fell: bool
    contact_instability: float
    elapsed_sim_s: float
    final_pose: FinalPose
    foot_contacts: tuple[FootContact, ...]
    aborted: bool = False
    invalid_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, TurnParameters):
            raise ValueError("parameters must be TurnParameters")
        if not isinstance(self.surface_name, str) or not self.surface_name.strip():
            raise ValueError("surface_name must be a nonblank string")
        object.__setattr__(self, "surface_name", self.surface_name.strip())
        object.__setattr__(self, "friction", _finite_float(self.friction, "friction", minimum=0.0))
        if self.friction == 0.0:
            raise ValueError("friction must be positive")
        expected_friction = _CANONICAL_SURFACE_FRICTION.get(self.surface_name)
        if expected_friction is None or self.friction != expected_friction:
            raise ValueError("surface_name and friction must be a canonical surface pair")
        for field in ("yaw_delta_deg", "translation_x_m", "translation_y_m"):
            object.__setattr__(self, field, _finite_float(getattr(self, field), field))
        for field in ("translation_m", "max_roll_deviation_deg", "max_pitch_deviation_deg", "elapsed_sim_s"):
            object.__setattr__(self, field, _finite_float(getattr(self, field), field, minimum=0.0))
        object.__setattr__(
            self,
            "contact_instability",
            _finite_float(self.contact_instability, "contact_instability", minimum=0.0),
        )
        if self.contact_instability > 1.0:
            raise ValueError("contact_instability must not exceed 1")
        if not isinstance(self.fell, bool) or not isinstance(self.aborted, bool):
            raise ValueError("fell and aborted must be booleans")
        if not isinstance(self.final_pose, FinalPose):
            raise ValueError("final_pose must be FinalPose")
        contacts = _validate_contacts(self.foot_contacts)
        object.__setattr__(self, "foot_contacts", contacts)
        if self.invalid_reason is not None:
            if not isinstance(self.invalid_reason, str) or not self.invalid_reason.strip():
                raise ValueError("invalid_reason must be a nonblank string or None")
            object.__setattr__(self, "invalid_reason", self.invalid_reason.strip())
        measured_distance = math.hypot(self.translation_x_m, self.translation_y_m)
        if not math.isclose(self.translation_m, measured_distance, rel_tol=1e-9, abs_tol=1e-12):
            raise ValueError("translation_m must match translation_x_m and translation_y_m")


def _validate_contacts(value: object) -> tuple[FootContact, ...]:
    if isinstance(value, (str, bytes)):
        raise ValueError("foot_contacts must be FootContact values")
    try:
        contacts = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise ValueError("foot_contacts must be FootContact values") from error
    if not contacts or any(not isinstance(contact, FootContact) for contact in contacts):
        raise ValueError("foot_contacts must be nonempty FootContact values")
    if len({contact.foot for contact in contacts}) != len(contacts):
        raise ValueError("foot_contacts cannot contain duplicate feet")
    return contacts


def yaw_from_quaternion(quaternion_xyzw: Iterable[object]) -> float:
    """Extract the normalized Z-axis heading from a Bullet XYZW quaternion."""
    x, y, z, w = _finite_vector(quaternion_xyzw, "quaternion_xyzw", 4)
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude == 0.0:
        raise ValueError("quaternion_xyzw must not be the zero quaternion")
    x, y, z, w = (component / magnitude for component in (x, y, z, w))
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def heading_from_quaternion(
    quaternion_xyzw: Iterable[object], *, source: str = "pybullet-base"
) -> HeadingObservation:
    """Wrap a measured base quaternion in the existing sourced heading contract."""
    return HeadingObservation(heading_deg=yaw_from_quaternion(quaternion_xyzw), source=source)


def yaw_delta_from_quaternions(
    before_quaternion_xyzw: Iterable[object],
    after_quaternion_xyzw: Iterable[object],
    *,
    source: str = "pybullet-base",
) -> float:
    """Measure wrapped heading change through the shared provenance-aware helper."""
    delta = signed_heading_delta_deg(
        heading_from_quaternion(before_quaternion_xyzw, source=source),
        heading_from_quaternion(after_quaternion_xyzw, source=source),
    )
    assert delta is not None
    return delta


def detect_fall(
    *,
    corrected_roll_deg: object,
    pitch_deviation_deg: object,
    height_m: object,
    torso_contact: object,
    supported_feet: object,
    support_loss_duration_s: object,
    fall_roll_deg: object = 10.0,
    fall_pitch_deg: object = 12.0,
    min_height_m: object = 0.075,
    max_support_loss_duration_s: object = 0.10,
) -> bool:
    """Apply the fixed simulator safety limits to an observed state."""
    roll = _finite_float(corrected_roll_deg, "corrected_roll_deg")
    pitch = _finite_float(pitch_deviation_deg, "pitch_deviation_deg")
    height = _finite_float(height_m, "height_m")
    duration = _finite_float(support_loss_duration_s, "support_loss_duration_s", minimum=0.0)
    roll_limit = _finite_float(fall_roll_deg, "fall_roll_deg", minimum=0.0)
    pitch_limit = _finite_float(fall_pitch_deg, "fall_pitch_deg", minimum=0.0)
    minimum_height = _finite_float(min_height_m, "min_height_m", minimum=0.0)
    support_limit = _finite_float(max_support_loss_duration_s, "max_support_loss_duration_s", minimum=0.0)
    if not isinstance(torso_contact, bool):
        raise ValueError("torso_contact must be a boolean")
    if isinstance(supported_feet, bool) or not isinstance(supported_feet, int) or not 0 <= supported_feet <= 4:
        raise ValueError("supported_feet must be an integer from 0 through 4")
    return (
        abs(roll) > roll_limit
        or abs(pitch) > pitch_limit
        or height < minimum_height
        or torso_contact
        or (supported_feet < 2 and duration >= support_limit)
    )
