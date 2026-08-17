"""Validated deterministic phase traces for simulated turn mechanics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from numbers import Real
from pathlib import Path
from typing import Iterable
from uuid import uuid4


LEGS = ("FL", "FR", "RL", "RR")
Vector3 = tuple[float, float, float]


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return float(value)


def _vector3(value: tuple[float, float, float] | None, field: str) -> None:
    if value is None or len(value) != 3:
        raise ValueError(f"{field} must contain three finite values")
    for item in value:
        _finite(item, field)


def _nonblank(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be nonblank")
    return value.strip()


@dataclass(frozen=True)
class FootTraceSample:
    leg: str
    in_contact: bool
    normal_force_n: float
    tangential_force_xyz_n: Vector3 | None
    world_position_xyz_m: Vector3 | None
    displacement_xyz_m: Vector3 | None
    cumulative_slip_m: float
    bullet_tau_z_nm: float | None
    right_yaw_torque_nm: float | None

    def __post_init__(self) -> None:
        if self.leg not in LEGS:
            raise ValueError("leg must be canonical")
        if not isinstance(self.in_contact, bool):
            raise ValueError("in_contact must be boolean")
        if _finite(self.normal_force_n, "normal_force_n") < 0.0:
            raise ValueError("normal_force_n must be nonnegative")
        if _finite(self.cumulative_slip_m, "cumulative_slip_m") < 0.0:
            raise ValueError("cumulative_slip_m must be nonnegative")
        for field, value in (
            ("tangential_force_xyz_n", self.tangential_force_xyz_n),
            ("world_position_xyz_m", self.world_position_xyz_m),
            ("displacement_xyz_m", self.displacement_xyz_m),
        ):
            if value is not None:
                _vector3(value, field)
        for field, value in (
            ("bullet_tau_z_nm", self.bullet_tau_z_nm),
            ("right_yaw_torque_nm", self.right_yaw_torque_nm),
        ):
            if value is not None:
                _finite(value, field)


@dataclass(frozen=True)
class PhaseTraceSample:
    phase: str
    elapsed_s: float
    body_yaw_deg: float
    body_roll_deg: float
    body_pitch_deg: float
    body_position_xyz_m: Vector3
    angular_velocity_xyz_rad_s: Vector3
    linear_velocity_xyz_m_s: Vector3
    feet: tuple[FootTraceSample, ...]
    support_foot_count: int
    total_contact_normal_force_n: float
    total_bullet_tau_z_nm: float | None
    total_right_yaw_torque_nm: float | None
    torso_contact: bool

    def __post_init__(self) -> None:
        _nonblank(self.phase, "phase")
        for field, value in (
            ("elapsed_s", self.elapsed_s),
            ("body_yaw_deg", self.body_yaw_deg),
            ("body_roll_deg", self.body_roll_deg),
            ("body_pitch_deg", self.body_pitch_deg),
            ("total_contact_normal_force_n", self.total_contact_normal_force_n),
        ):
            _finite(value, field)
        for field, value in (
            ("body_position_xyz_m", self.body_position_xyz_m),
            ("angular_velocity_xyz_rad_s", self.angular_velocity_xyz_rad_s),
            ("linear_velocity_xyz_m_s", self.linear_velocity_xyz_m_s),
        ):
            _vector3(value, field)
        if tuple(foot.leg for foot in self.feet) != LEGS:
            raise ValueError("feet must use canonical FL, FR, RL, RR order")
        if isinstance(self.support_foot_count, bool) or not 0 <= self.support_foot_count <= 4:
            raise ValueError("support_foot_count must be an integer from zero to four")
        if not isinstance(self.torso_contact, bool):
            raise ValueError("torso_contact must be boolean")
        for field, value in (
            ("total_bullet_tau_z_nm", self.total_bullet_tau_z_nm),
            ("total_right_yaw_torque_nm", self.total_right_yaw_torque_nm),
        ):
            if value is not None:
                _finite(value, field)


@dataclass(frozen=True)
class DiagnosticTrace:
    schema_version: int
    run_id: str
    primitive_family: str
    candidate_id: str
    surface_name: str
    samples: tuple[PhaseTraceSample, ...]

    def __post_init__(self) -> None:
        if isinstance(self.schema_version, bool) or self.schema_version != 1:
            raise ValueError("schema_version must be 1")
        for field in ("run_id", "primitive_family", "candidate_id", "surface_name"):
            _nonblank(getattr(self, field), field)
        if not self.samples:
            raise ValueError("trace must contain at least one sample")
        for sample in self.samples:
            sample.__post_init__()


@dataclass(frozen=True)
class PhaseSummary:
    phase: str
    active_contacts: tuple[str, ...]
    dominant_yaw_torque_foot: str | None
    net_right_yaw_torque_nm: float | None
    peak_positive_right_yaw_torque_nm: float | None
    peak_negative_right_yaw_torque_nm: float | None
    yaw_delta_deg: float
    total_slip_m: float
    max_roll_excursion_deg: float
    max_pitch_excursion_deg: float


def trace_json_bytes(trace: DiagnosticTrace) -> bytes:
    trace.__post_init__()
    try:
        text = json.dumps(asdict(trace), sort_keys=True, separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError("trace values must be finite and JSON serializable") from error
    return (text + "\n").encode("utf-8")


def write_trace_atomic(path: str | Path, trace: DiagnosticTrace) -> None:
    destination = Path(path)
    payload = trace_json_bytes(trace)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{uuid4().hex}.tmp")
    try:
        temporary.write_bytes(payload)
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def _wrapped_delta(before: float, after: float) -> float:
    return (after - before + 180.0) % 360.0 - 180.0


def summarize_trace(trace: DiagnosticTrace) -> tuple[PhaseSummary, ...]:
    trace.__post_init__()
    phase_order: list[str] = []
    grouped: dict[str, list[PhaseTraceSample]] = {}
    for sample in trace.samples:
        if sample.phase not in grouped:
            phase_order.append(sample.phase)
            grouped[sample.phase] = []
        grouped[sample.phase].append(sample)
    summaries: list[PhaseSummary] = []
    for phase in phase_order:
        samples = grouped[phase]
        torque_by_foot = {leg: 0.0 for leg in LEGS}
        available = False
        totals: list[float] = []
        for sample in samples:
            if sample.total_right_yaw_torque_nm is not None:
                totals.append(sample.total_right_yaw_torque_nm)
            for foot in sample.feet:
                if foot.right_yaw_torque_nm is not None:
                    available = True
                    torque_by_foot[foot.leg] += foot.right_yaw_torque_nm
        dominant = max(LEGS, key=lambda leg: abs(torque_by_foot[leg])) if available else None
        active = tuple(leg for leg in LEGS if any(next(f for f in s.feet if f.leg == leg).in_contact for s in samples))
        summaries.append(
            PhaseSummary(
                phase=phase,
                active_contacts=active,
                dominant_yaw_torque_foot=dominant,
                net_right_yaw_torque_nm=sum(totals) if totals else None,
                peak_positive_right_yaw_torque_nm=max(totals) if totals else None,
                peak_negative_right_yaw_torque_nm=min(totals) if totals else None,
                yaw_delta_deg=_wrapped_delta(samples[0].body_yaw_deg, samples[-1].body_yaw_deg),
                total_slip_m=sum(foot.cumulative_slip_m for sample in samples for foot in sample.feet),
                max_roll_excursion_deg=max(s.body_roll_deg for s in samples) - min(s.body_roll_deg for s in samples),
                max_pitch_excursion_deg=max(s.body_pitch_deg for s in samples) - min(s.body_pitch_deg for s in samples),
            )
        )
    return tuple(summaries)
