from __future__ import annotations

from dataclasses import is_dataclass
import math

import pytest

from pip_robot.turn.models import (
    HeadingObservation,
    ImuSample,
    PoseObservation,
    SafetyDecision,
    TurnParameters,
    TurnResult,
)


BASELINE = {
    "direction": "right",
    "unload_pair": "FL_RR",
    "unload_mm": 4.0,
    "tangential_mm": 3.0,
    "hold_s": 0.35,
    "settle_s": 0.35,
    "replant_s": 0.35,
    "cycles": 1,
    "speed": 20,
}


def test_from_mapping_accepts_the_literal_right_turn_baseline() -> None:
    parameters = TurnParameters.from_mapping(BASELINE)

    assert parameters == TurnParameters(
        direction="right",
        unload_pair="FL_RR",
        unload_mm=4.0,
        tangential_mm=3.0,
        hold_s=0.35,
        settle_s=0.35,
        replant_s=0.35,
        cycles=1,
        speed=20,
    )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("direction", "left"),
        ("unload_pair", "FR_RL"),
        ("unload_mm", 0.0),
        ("tangential_mm", math.nan),
        ("hold_s", -0.1),
        ("settle_s", math.inf),
        ("replant_s", 0.0),
        ("cycles", True),
        ("speed", 101),
    ],
)
def test_direct_constructor_enforces_every_turn_parameter_invariant(
    field: str, value: object
) -> None:
    values = {**BASELINE, field: value}

    with pytest.raises(ValueError, match=field):
        TurnParameters(**values)  # type: ignore[arg-type]


@pytest.mark.parametrize("field", sorted(BASELINE))
def test_from_mapping_rejects_each_missing_baseline_field(field: str) -> None:
    mapping = dict(BASELINE)
    del mapping[field]

    with pytest.raises(ValueError, match="exactly"):
        TurnParameters.from_mapping(mapping)


def test_from_mapping_rejects_unexpected_baseline_field() -> None:
    mapping = {**BASELINE, "extra": "not-approved"}

    with pytest.raises(ValueError, match="exactly"):
        TurnParameters.from_mapping(mapping)


@pytest.mark.parametrize("field", ["unload_mm", "tangential_mm", "hold_s", "settle_s", "replant_s"])
@pytest.mark.parametrize("value", [0.0, -0.01, math.inf, -math.inf, math.nan])
def test_from_mapping_rejects_non_positive_or_non_finite_distances_and_timings(
    field: str, value: float
) -> None:
    mapping = {**BASELINE, field: value}

    with pytest.raises(ValueError, match=field):
        TurnParameters.from_mapping(mapping)


@pytest.mark.parametrize("value", [0, -1, 1.5, True])
def test_from_mapping_rejects_invalid_cycles(value: object) -> None:
    mapping = {**BASELINE, "cycles": value}

    with pytest.raises(ValueError, match="cycles"):
        TurnParameters.from_mapping(mapping)


@pytest.mark.parametrize("value", [0, -1, 101, 20.5, True])
def test_from_mapping_rejects_speed_outside_the_approved_integer_range(value: object) -> None:
    mapping = {**BASELINE, "speed": value}

    with pytest.raises(ValueError, match="speed"):
        TurnParameters.from_mapping(mapping)


@pytest.mark.parametrize(
    "field,value",
    [("direction", "left"), ("direction", "RIGHT"), ("unload_pair", "FR_RL")],
)
def test_from_mapping_rejects_an_unapproved_direction_or_unload_pair(field: str, value: str) -> None:
    mapping = {**BASELINE, field: value}

    with pytest.raises(ValueError, match=field):
        TurnParameters.from_mapping(mapping)


def test_turn_parameters_are_immutable() -> None:
    parameters = TurnParameters.from_mapping(BASELINE)

    with pytest.raises(Exception):
        parameters.speed = 30  # type: ignore[misc]


@pytest.mark.parametrize("source", [None, "", "   "])
def test_heading_observation_rejects_a_measurement_without_provenance(
    source: str | None,
) -> None:
    with pytest.raises(ValueError, match="source"):
        HeadingObservation(heading_deg=90.0, source=source)


@pytest.mark.parametrize("heading_deg", [math.nan, math.inf, -math.inf, True])
def test_heading_observation_rejects_invalid_measurements(heading_deg: object) -> None:
    with pytest.raises(ValueError, match="heading_deg"):
        HeadingObservation(heading_deg=heading_deg, source="compass")  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "model",
    [TurnParameters, ImuSample, PoseObservation, HeadingObservation, SafetyDecision, TurnResult],
)
def test_all_boundary_models_are_frozen_dataclasses(model: type[object]) -> None:
    assert is_dataclass(model)
    assert model.__dataclass_params__.frozen is True
