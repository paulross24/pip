from __future__ import annotations

import math

import pytest

from pip_robot.turn.heading import is_useful_turn, signed_heading_delta_deg


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
    assert signed_heading_delta_deg(before, after) == expected


@pytest.mark.parametrize(
    ("before", "after"),
    [(None, 25.0), (25.0, None), (None, None), (math.nan, 25.0), (25.0, math.inf)],
)
def test_signed_heading_delta_requires_finite_measurements(
    before: float | None, after: float | None
) -> None:
    assert signed_heading_delta_deg(before, after) is None


def test_useful_turn_requires_a_measured_delta_at_or_above_the_minimum() -> None:
    assert is_useful_turn(10.0, 30.0, minimum_abs_delta_deg=20.0) is True
    assert is_useful_turn(30.0, 10.0, minimum_abs_delta_deg=20.0) is True
    assert is_useful_turn(10.0, 29.0, minimum_abs_delta_deg=20.0) is False
    assert is_useful_turn(None, 30.0, minimum_abs_delta_deg=20.0) is False
