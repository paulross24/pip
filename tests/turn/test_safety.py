from __future__ import annotations

import math

import pytest

from pip_robot.turn.models import ImuSample
from pip_robot.turn.safety import ImuBaseline, evaluate_imu_safety, settled_baseline


def test_settled_baseline_uses_the_median_to_preserve_standing_bias() -> None:
    baseline = settled_baseline(
        [
            ImuSample(roll_deg=2.0, pitch_deg=-1.0),
            ImuSample(roll_deg=2.3, pitch_deg=-0.8),
            ImuSample(roll_deg=1.9, pitch_deg=-1.1),
        ]
    )

    assert baseline == ImuBaseline(roll_deg=2.0, pitch_deg=-1.0)


def test_small_jitter_around_the_settled_bias_is_allowed() -> None:
    decision = evaluate_imu_safety(
        ImuSample(roll_deg=2.4, pitch_deg=-1.3),
        ImuBaseline(roll_deg=2.0, pitch_deg=-1.0),
        roll_limit_deg=1.0,
        pitch_limit_deg=1.0,
    )

    assert decision.allowed is True
    assert decision.reason_code == "ok"
    assert decision.roll_error_deg == pytest.approx(0.4)
    assert decision.pitch_error_deg == pytest.approx(-0.3)


def test_large_roll_deviation_is_rejected_before_pitch_for_stable_precedence() -> None:
    decision = evaluate_imu_safety(
        ImuSample(roll_deg=12.0, pitch_deg=-12.0),
        ImuBaseline(roll_deg=2.0, pitch_deg=-1.0),
        roll_limit_deg=8.0,
        pitch_limit_deg=8.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == "roll_error_limit"
    assert decision.roll_error_deg == 10.0
    assert decision.pitch_error_deg == -11.0


def test_large_pitch_deviation_is_rejected() -> None:
    decision = evaluate_imu_safety(
        ImuSample(roll_deg=2.0, pitch_deg=8.0),
        ImuBaseline(roll_deg=2.0, pitch_deg=-1.0),
        roll_limit_deg=8.0,
        pitch_limit_deg=8.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == "pitch_error_limit"


def test_missing_sample_fails_closed() -> None:
    decision = evaluate_imu_safety(
        None,
        ImuBaseline(roll_deg=2.0, pitch_deg=-1.0),
        roll_limit_deg=8.0,
        pitch_limit_deg=8.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == "imu_missing"
    assert decision.roll_error_deg is None
    assert decision.pitch_error_deg is None


@pytest.mark.parametrize(
    "sample",
    [
        ImuSample(roll_deg=None, pitch_deg=0.0),
        ImuSample(roll_deg=0.0, pitch_deg=None),
        ImuSample(roll_deg=math.nan, pitch_deg=0.0),
        ImuSample(roll_deg=0.0, pitch_deg=math.inf),
        ImuSample(roll_deg=0.0, pitch_deg=0.0, valid=False),
    ],
)
def test_missing_invalid_or_non_finite_attitude_fails_closed(sample: ImuSample) -> None:
    decision = evaluate_imu_safety(
        sample,
        ImuBaseline(roll_deg=0.0, pitch_deg=0.0),
        roll_limit_deg=8.0,
        pitch_limit_deg=8.0,
    )

    assert decision.allowed is False
    assert decision.reason_code == "imu_invalid"


def test_settled_baseline_rejects_empty_or_invalid_sample_windows() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        settled_baseline([])

    with pytest.raises(ValueError, match="valid finite"):
        settled_baseline([ImuSample(roll_deg=0.0, pitch_deg=math.nan)])
