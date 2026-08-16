from __future__ import annotations

from pathlib import Path

from tools.summarize_turn import render_summary, write_summary


def sweep_document() -> dict[str, object]:
    candidates = []
    for rank in range(1, 13):
        candidates.append(
            {
                "rank": rank,
                "parameters": {
                    "direction": "right",
                    "unload_pair": "FL_RR",
                    "unload_mm": 2.0,
                    "tangential_mm": 1.0,
                    "hold_s": 0.20 + rank / 100.0,
                    "settle_s": 0.35,
                    "replant_s": 0.35,
                    "cycles": 1,
                    "speed": 20,
                },
                "score": 100.0 - rank,
                "worst_surface": "low",
                "fell": False,
                "surface_results": [
                    {"surface": "low", "yaw_delta_deg": 7.0, "score": 90.0},
                    {"surface": "nominal", "yaw_delta_deg": 8.0, "score": 95.0},
                    {"surface": "high", "yaw_delta_deg": 9.0, "score": 99.0},
                ],
            }
        )
    return {
        "schema": "pip-sim-turn-sweep/v1",
        "model_sha256": "a" * 64,
        "turn_config_sha256": "b" * 64,
        "simulation_config_sha256": "c" * 64,
        "pybullet_version": "3.2.7",
        "search_candidate_count": 125,
        "evaluated_candidate_count": 12,
        "surface_run_count": 36,
        "safe_candidate_count": 12,
        "fall_candidate_count": 0,
        "ranked_candidates": candidates,
    }


def test_summary_is_concise_deterministic_and_includes_required_decision_evidence() -> None:
    summary = render_summary(sweep_document())

    assert summary.startswith("# Stationary Turn Sweep\n")
    assert "pip-sim-turn-sweep/v1" in summary
    assert "a" * 64 in summary
    assert "b" * 64 in summary
    assert "c" * 64 in summary
    assert "PyBullet version: 3.2.7" in summary
    assert "Search candidates: 125" in summary
    assert "Evaluated candidates: 12" in summary
    assert "Safe candidates: 12" in summary
    assert "Fall candidates: 0" in summary
    assert "Best worst-surface score: 99" in summary
    assert "Best worst-surface yaw: 7" in summary
    assert "Consistency: yaw range 7 to 9 deg (spread 2 deg)" in summary
    assert "Rationale:" in summary
    rows = [line for line in summary.splitlines() if line.startswith("| ") and line[2:3].isdigit()]
    assert [row.split("|")[1].strip() for row in rows] == [str(rank) for rank in range(1, 11)]
    assert "timestamp" not in summary.lower()
    assert "raw step" not in summary.lower()
    assert render_summary(sweep_document()).encode() == summary.encode()
    assert len(summary.splitlines()) < 40


def test_summary_handles_no_safe_candidate_and_creates_output_directories(tmp_path: Path) -> None:
    document = sweep_document()
    document["safe_candidate_count"] = 0
    document["fall_candidate_count"] = 12
    for candidate in document["ranked_candidates"]:  # type: ignore[union-attr]
        candidate["score"] = None
        candidate["fell"] = True

    summary = render_summary(document)
    output = tmp_path / "nested" / "latest-summary.md"
    write_summary(summary, output)

    assert "No safe candidate produced a finite positive-yaw score." in summary
    assert "Ranked fallback parameters:" in summary
    assert "Worst-surface yaw:" in summary
    assert "Disqualification:" in summary
    assert output.read_text(encoding="utf-8") == summary
    assert output.read_bytes().endswith(b"\n")
