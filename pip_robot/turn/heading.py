"""Heading calculations that accept physical measurements only."""

from __future__ import annotations

import math
from numbers import Real

from .models import HeadingObservation


def _observed_heading(value: HeadingObservation | None) -> float | None:
    if value is None:
        return None
    if not isinstance(value, HeadingObservation):
        raise TypeError("heading measurements must be HeadingObservation instances")
    return value.heading_deg


def _finite_threshold(value: object) -> float | None:
    if value is None or isinstance(value, bool) or not isinstance(value, Real):
        return None
    threshold = float(value)
    return threshold if math.isfinite(threshold) else None


def signed_heading_delta_deg(
    before: HeadingObservation | None,
    after: HeadingObservation | None,
) -> float | None:
    """Return the shortest sourced measurement change in ``[-180, 180)``."""
    before_measurement = _observed_heading(before)
    after_measurement = _observed_heading(after)
    if before_measurement is None or after_measurement is None:
        return None
    return (after_measurement - before_measurement + 180.0) % 360.0 - 180.0


def is_useful_turn(
    before: HeadingObservation | None,
    after: HeadingObservation | None,
    minimum_abs_delta_deg: float,
) -> bool:
    """Require a sourced heading delta whose magnitude meets a positive threshold."""
    minimum = _finite_threshold(minimum_abs_delta_deg)
    if minimum is None or minimum <= 0:
        raise ValueError("minimum_abs_delta_deg must be a positive finite number")

    delta = signed_heading_delta_deg(before, after)
    return delta is not None and abs(delta) >= minimum
