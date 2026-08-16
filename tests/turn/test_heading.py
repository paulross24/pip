from __future__ import annotations

import math

import pytest

from pip_robot.turn.heading import is_useful_turn, signed_heading_delta_deg
from pip_robot.turn.models import HeadingObservation


def observed(heading_deg: float | None, source: str = "compass") -> HeadingObservation:
    return HeadingObservation(heading_deg=heading_deg, source=source)


@pytest.mark.parametrize(
    ("before", "after", "expected"),
    [
        (10.0, 35.0, 25.0),
        (35.0, 10.0, -25.0),
        (358.0, 3.0, 5.0),
        (3.0, 358.0, -5.0),
        (0.0, 180.0, -180.0),
    ],
)
def test_signed_heading_delta_uses_the_shortest_range(
    before: float, after: float, expected: float
) -> None:
    assert signed_heading_delta_deg(observed(before), observed(after)) == expected


@pytest.mark.parametrize(
    ("before", "after"),
    [
        (None, observed(25.0)),
        (observed(25.0), None),
        (None, None),
        (observed(None), observed(25.0)),
        (observed(25.0), observed(None)),
    ],
)
def test_signed_heading_delta_requires_available_measurements(
    before: HeadingObservation | None, after: HeadingObservation | None
) -> None:
    assert signed_heading_delta_deg(before, after) is None


def test_heading_delta_rejects_anonymous_numeric_yaw() -> None:
    with pytest.raises(TypeError, match="HeadingObservation"):
        signed_heading_delta_deg(10.0, observed(30.0))  # type: ignore[arg-type]


def test_useful_turn_requires_a_measured_delta_at_or_above_the_minimum() -> None:
    assert is_useful_turn(observed(10.0), observed(30.0), minimum_abs_delta_deg=20.0) is True
    assert is_useful_turn(observed(30.0), observed(10.0), minimum_abs_delta_deg=20.0) is True
    assert is_useful_turn(observed(10.0), observed(29.0), minimum_abs_delta_deg=20.0) is False
    assert is_useful_turn(None, observed(30.0), minimum_abs_delta_deg=20.0) is False


@pytest.mark.parametrize("threshold", [0.0, -1.0, math.nan, math.inf, True])
def test_useful_turn_rejects_a_non_positive_or_invalid_threshold(
    threshold: object,
) -> None:
    with pytest.raises(ValueError):
        is_useful_turn(
            observed(10.0),
            observed(30.0),
            minimum_abs_delta_deg=threshold,  # type: ignore[arg-type]
        )
