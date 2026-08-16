"""Immutable contracts for the stationary right-turn foundation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Any, Mapping


_PARAMETER_FIELDS = frozenset(
    {
        "direction",
        "unload_pair",
        "unload_mm",
        "tangential_mm",
        "hold_s",
        "settle_s",
        "replant_s",
        "cycles",
        "speed",
    }
)
_POSITIVE_FINITE_FIELDS = (
    "unload_mm",
    "tangential_mm",
    "hold_s",
    "settle_s",
    "replant_s",
)


def _positive_finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{field} must be a positive finite number")
    return float(value)


@dataclass(frozen=True)
class TurnParameters:
    """Approved parameters for a future stationary right turn."""

    direction: str
    unload_pair: str
    unload_mm: float
    tangential_mm: float
    hold_s: float
    settle_s: float
    replant_s: float
    cycles: int
    speed: int

    def __post_init__(self) -> None:
        """Enforce the parameter contract for every construction path."""
        if self.direction != "right":
            raise ValueError("direction must be 'right'")
        if self.unload_pair != "FL_RR":
            raise ValueError("unload_pair must be 'FL_RR'")
        for field in _POSITIVE_FINITE_FIELDS:
            value = _positive_finite_number(getattr(self, field), field)
            object.__setattr__(self, field, value)
        if isinstance(self.cycles, bool) or not isinstance(self.cycles, int) or self.cycles <= 0:
            raise ValueError("cycles must be a positive integer")
        if (
            isinstance(self.speed, bool)
            or not isinstance(self.speed, int)
            or not 1 <= self.speed <= 100
        ):
            raise ValueError("speed must be an integer from 1 through 100")

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TurnParameters":
        """Validate and construct parameters from the exact approved field set."""
        if not isinstance(data, Mapping) or set(data) != _PARAMETER_FIELDS:
            raise ValueError("turn parameter fields must exactly match the approved baseline")
        if data["direction"] != "right":
            raise ValueError("direction must be 'right'")
        if data["unload_pair"] != "FL_RR":
            raise ValueError("unload_pair must be 'FL_RR'")

        values = {
            field: _positive_finite_number(data[field], field)
            for field in _POSITIVE_FINITE_FIELDS
        }
        cycles = data["cycles"]
        if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles <= 0:
            raise ValueError("cycles must be a positive integer")
        speed = data["speed"]
        if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 100:
            raise ValueError("speed must be an integer from 1 through 100")

        return cls(
            direction="right",
            unload_pair="FL_RR",
            cycles=cycles,
            speed=speed,
            **values,
        )


@dataclass(frozen=True)
class ImuSample:
    """One fresh IMU reading, with optional raw axes and derived attitude."""

    roll_deg: float | None = None
    pitch_deg: float | None = None
    accel_xyz: tuple[float, float, float] | None = None
    gyro_xyz: tuple[float, float, float] | None = None
    monotonic_s: float | None = None
    utc_timestamp: str | None = None
    valid: bool = True


@dataclass(frozen=True)
class PoseObservation:
    """Observed robot pose without commanding a pose change."""

    roll_deg: float | None = None
    pitch_deg: float | None = None
    observed_at: str | None = None


@dataclass(frozen=True)
class HeadingObservation:
    """Optional physical heading measurement and its source."""

    heading_deg: float | None = None
    source: str | None = None
    observed_at: str | None = None

    def __post_init__(self) -> None:
        """Require finite measured headings to carry explicit provenance."""
        if self.heading_deg is None:
            return
        if (
            isinstance(self.heading_deg, bool)
            or not isinstance(self.heading_deg, Real)
            or not math.isfinite(self.heading_deg)
        ):
            raise ValueError("heading_deg must be a finite number or None")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source is required for a heading measurement")
        object.__setattr__(self, "heading_deg", float(self.heading_deg))


@dataclass(frozen=True)
class SafetyDecision:
    """Fail-closed safety outcome relative to a settled IMU baseline."""

    allowed: bool
    reason_code: str
    roll_error_deg: float | None = None
    pitch_error_deg: float | None = None
    roll_limit_deg: float | None = None
    pitch_limit_deg: float | None = None


@dataclass(frozen=True)
class TurnResult:
    """Final hardware-independent outcome of a turn run."""

    completed: bool
    aborted: bool
    final_state: str
    reason_code: str | None = None
    measured_heading_delta_deg: float | None = None
