"""Heading calculations that accept physical measurements only."""

from __future__ import annotations

import math
from numbers import Real


def _finite_measurement(value: object) -> float | None:
    if value is None or isinstance(value, bool) or not isinstance(value, Real):
        return None
    measurement = float(value)
    return measurement if math.isfinite(measurement) else None


def signed_heading_delta_deg(before: float | None, after: float | None) -> float | None:
    """Return the shortest measured angular change in ``[-180, 180)``."""
    before_measurement = _finite_measurement(before)
    after_measurement = _finite_measurement(after)
    if before_measurement is None or after_measurement is None:
        return None
    return (after_measurement - before_measurement + 180.0) % 360.0 - 180.0


def is_useful_turn(
    before: float | None, after: float | None, minimum_abs_delta_deg: float
) -> bool:
    """Require a physical heading delta whose magnitude meets the threshold."""
    minimum = _finite_measurement(minimum_abs_delta_deg)
    if minimum is None or minimum < 0:
        raise ValueError("minimum_abs_delta_deg must be a non-negative finite number")

    delta = signed_heading_delta_deg(before, after)
    return delta is not None and abs(delta) >= minimum
