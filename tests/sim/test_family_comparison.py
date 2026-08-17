import json

from sim.family_comparison import (
    CandidateComparison,
    SurfaceComparison,
    comparison_json_bytes,
    rank_candidates,
)


def candidate(identifier, yaws, *, falls=False, translation=0.01, slip=0.02):
    surfaces = tuple(
        SurfaceComparison(name, yaw, translation, 1.0, 1.0, 0.0, 0.2, -0.1, 0.05, slip, falls)
        for name, yaw in zip(("low", "nominal", "high"), yaws)
    )
    return CandidateComparison("family", identifier, {"magnitude_mm": 3.0}, surfaces)


def test_consistent_small_positive_yaw_outranks_sign_reversing_high_yaw():
    consistent = candidate("consistent", (0.02, 0.03, 0.02))
    reversing = candidate("reversing", (8.0, -3.0, 8.0))
    ranked = rank_candidates((reversing, consistent))
    assert ranked[0].candidate_id == "consistent"
    assert ranked[0].promotable is True
    assert ranked[1].promotable is False


def test_falls_and_wrong_sign_are_not_promotable():
    assert candidate("fall", (1.0, 1.0, 1.0), falls=True).promotable is False
    assert candidate("wrong", (-0.1, -0.1, -0.1)).promotable is False


def test_comparison_serialization_is_byte_stable_and_finite_json():
    values = rank_candidates((candidate("b", (0.1, 0.1, 0.1)), candidate("a", (0.2, 0.2, 0.2))))
    first = comparison_json_bytes(values)
    assert first == comparison_json_bytes(values)
    assert len(json.loads(first)["candidates"]) == 2
