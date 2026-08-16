"""Pure, fail-closed IMU checks relative to a settled stance."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from statistics import median
from typing import Iterable

from pip_robot.turn.models import ImuSample, SafetyDecision


@dataclass(frozen=True)
class ImuBaseline:
    """Median roll and pitch measured while the robot is settled."""

    roll_deg: float
    pitch_deg: float


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _valid_attitude(sample: object) -> tuple[float, float] | None:
    if not isinstance(sample, ImuSample) or not sample.valid:
        return None
    roll = _finite_number(sample.roll_deg)
    pitch = _finite_number(sample.pitch_deg)
    if roll is None or pitch is None:
        return None
    return roll, pitch


def _valid_baseline(baseline: object) -> tuple[float, float]:
    if not isinstance(baseline, ImuBaseline):
        raise ValueError("baseline must be an ImuBaseline with finite attitude")
    roll = _finite_number(baseline.roll_deg)
    pitch = _finite_number(baseline.pitch_deg)
    if roll is None or pitch is None:
        raise ValueError("baseline must be an ImuBaseline with finite attitude")
    return roll, pitch


def _non_negative_finite_limit(value: object, name: str) -> float:
    limit = _finite_number(value)
    if limit is None or limit < 0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return limit


def settled_baseline(samples: Iterable[ImuSample]) -> ImuBaseline:
    """Calculate a median bias from a non-empty window of valid IMU samples."""
    attitudes = [_valid_attitude(sample) for sample in samples]
    if not attitudes:
        raise ValueError("settled baseline requires a non-empty sample window")
    if any(attitude is None for attitude in attitudes):
        raise ValueError("settled baseline requires valid finite attitude samples")

    valid_attitudes = [attitude for attitude in attitudes if attitude is not None]
    return ImuBaseline(
        roll_deg=float(median(roll for roll, _ in valid_attitudes)),
        pitch_deg=float(median(pitch for _, pitch in valid_attitudes)),
    )


def evaluate_imu_safety(
    sample: ImuSample | None,
    baseline: ImuBaseline,
    roll_limit_deg: float,
    pitch_limit_deg: float,
) -> SafetyDecision:
    """Reject absent or invalid readings and deviations outside either limit."""
    roll_limit = _non_negative_finite_limit(roll_limit_deg, "roll_limit_deg")
    pitch_limit = _non_negative_finite_limit(pitch_limit_deg, "pitch_limit_deg")
    baseline_roll, baseline_pitch = _valid_baseline(baseline)

    if sample is None:
        return SafetyDecision(
            allowed=False,
            reason_code="imu_missing",
            roll_limit_deg=roll_limit,
            pitch_limit_deg=pitch_limit,
        )

    attitude = _valid_attitude(sample)
    if attitude is None:
        return SafetyDecision(
            allowed=False,
            reason_code="imu_invalid",
            roll_limit_deg=roll_limit,
            pitch_limit_deg=pitch_limit,
        )

    roll_error, pitch_error = attitude[0] - baseline_roll, attitude[1] - baseline_pitch
    if abs(roll_error) > roll_limit:
        reason_code = "roll_error_limit"
        allowed = False
    elif abs(pitch_error) > pitch_limit:
        reason_code = "pitch_error_limit"
        allowed = False
    else:
        reason_code = "ok"
        allowed = True

    return SafetyDecision(
        allowed=allowed,
        reason_code=reason_code,
        roll_error_deg=roll_error,
        pitch_error_deg=pitch_error,
        roll_limit_deg=roll_limit,
        pitch_limit_deg=pitch_limit,
    )
