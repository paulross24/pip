from __future__ import annotations

import math

import pytest

from pip_robot.turn.models import TurnParameters
from sim.model import FinalPose, FootContact, SimulationResult
from sim.score import CandidateScore, aggregate_candidate_score, score_result


PARAMETERS = TurnParameters(
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
POSE = FinalPose((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0)
CONTACTS = tuple(FootContact(foot, True, 1.0) for foot in ("FL", "FR", "RL", "RR"))


def result(surface: str = "nominal", **changes: object) -> SimulationResult:
    values: dict[str, object] = {
        "parameters": PARAMETERS,
        "surface_name": surface,
        "friction": 0.70,
        "yaw_delta_deg": 12.0,
        "translation_x_m": 0.03,
        "translation_y_m": 0.04,
        "translation_m": 0.05,
        "max_roll_deviation_deg": 2.0,
        "max_pitch_deviation_deg": 3.0,
        "fell": False,
        "contact_instability": 0.25,
        "elapsed_sim_s": 1.0,
        "final_pose": POSE,
        "foot_contacts": CONTACTS,
    }
    values.update(changes)
    return SimulationResult(**values)  # type: ignore[arg-type]


def test_score_uses_the_visible_safe_candidate_formula() -> None:
    assert score_result(result()) == pytest.approx(10 * 12 - 1000 * 0.05 - 2 * 2 - 2 * 3 - 5 * 0.25)


@pytest.mark.parametrize(
    "changes",
    [
        {"fell": True},
        {"aborted": True},
        {"invalid_reason": "non-finite measurement"},
        {"yaw_delta_deg": 0.0},
        {"yaw_delta_deg": -0.01},
    ],
)
def test_score_disqualifies_falls_aborts_invalid_results_and_non_positive_yaw(changes: dict[str, object]) -> None:
    assert score_result(result(**changes)) == -math.inf


def test_aggregate_uses_the_worst_surface_score_and_records_deterministic_tie_data() -> None:
    aggregate = aggregate_candidate_score(
        (
            result("high", yaw_delta_deg=14.0),
            result("nominal", yaw_delta_deg=9.0),
            result("low", yaw_delta_deg=11.0),
        )
    )

    assert isinstance(aggregate, CandidateScore)
    assert aggregate.score == pytest.approx(score_result(result("nominal", yaw_delta_deg=9.0)))
    assert aggregate.worst_surface_name == "nominal"
    assert aggregate.surface_scores == (
        ("low", pytest.approx(score_result(result("low", yaw_delta_deg=11.0)))),
        ("nominal", pytest.approx(score_result(result("nominal", yaw_delta_deg=9.0)))),
        ("high", pytest.approx(score_result(result("high", yaw_delta_deg=14.0)))),
    )
    assert aggregate.tie_break_data == (0, 0.05, 5.0, (4.0, 3.0, 0.35, 0.35, 0.35, 1, 20))


def test_aggregate_breaks_equal_worst_scores_by_fixed_surface_order() -> None:
    aggregate = aggregate_candidate_score(
        (
            result("high", yaw_delta_deg=12.0),
            result("nominal", yaw_delta_deg=12.0),
            result("low", yaw_delta_deg=12.0),
        )
    )

    assert aggregate.worst_surface_name == "low"


def test_aggregate_rejects_mixed_parameter_candidates() -> None:
    other_parameters = TurnParameters(
        direction="right",
        unload_pair="FL_RR",
        unload_mm=5.0,
        tangential_mm=3.0,
        hold_s=0.35,
        settle_s=0.35,
        replant_s=0.35,
        cycles=1,
        speed=20,
    )

    with pytest.raises(ValueError, match="parameters"):
        aggregate_candidate_score((result("low"), result("nominal", parameters=other_parameters)))
