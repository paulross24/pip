from sim.pivot_runner import load_turn_parameters
from tools.compare_turn_families import build_candidates, comparison_markdown


def test_each_family_has_three_conservative_deterministic_candidates():
    baseline = load_turn_parameters("config/turn_right_baseline.json")
    first = build_candidates(baseline)
    assert first == build_candidates(baseline)
    assert [primitive.family for primitive, _, _ in first] == [
        family for family in ("diagonal_unload", "same_side_shear", "differential_fore_aft", "staged_pivot")
        for _ in range(3)
    ]
    assert len(first) == 12


def test_markdown_has_family_candidate_and_required_surface_columns():
    text = comparison_markdown(())
    assert "Primitive family" in text
    assert "Low yaw" in text
    assert "Nominal yaw" in text
    assert "High yaw" in text
