import json

from sim.family_comparison import (
    CandidateComparison,
    SurfaceComparison,
    comparison_json_bytes,
    rank_candidates,
    rank_families,
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


def test_promotion_rejects_excess_translation_attitude_slip_or_recovery_cancellation():
    base = candidate("base", (0.1, 0.1, 0.1)).surfaces
    for field, value in (
        ("translation_m", 0.021), ("max_roll_deg", 8.1),
        ("max_pitch_deg", 8.1), ("contact_instability", 0.51),
        ("slip_m", 0.101), ("recovery_cancellation_fraction", 0.51),
    ):
        from dataclasses import replace
        surfaces = tuple(replace(item, **{field: value}) for item in base)
        assert CandidateComparison("family", field, {}, surfaces).promotable is False


def test_family_ranking_is_separate_and_selects_best_promotable_candidate():
    candidates = (
        CandidateComparison("a", "a1", {}, candidate("x", (0.1, 0.1, 0.1)).surfaces),
        CandidateComparison("b", "b1", {}, candidate("x", (0.2, 0.2, 0.2), translation=0.015).surfaces),
    )
    families = rank_families(candidates)
    assert tuple(item.primitive_family for item in families) == ("a", "b")
    assert families[0].selected_candidate_id == "a1"


def test_comparison_serialization_is_byte_stable_and_finite_json():
    values = rank_candidates((candidate("b", (0.1, 0.1, 0.1)), candidate("a", (0.2, 0.2, 0.2))))
    first = comparison_json_bytes(values)
    assert first == comparison_json_bytes(values)
    document = json.loads(first)
    assert len(document["candidates"]) == 2
    assert len(document["families"]) == 1
