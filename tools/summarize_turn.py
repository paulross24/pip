"""Render a compact deterministic decision summary for a turn sweep."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping, Sequence


def _number(value: object) -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        number = float(value)
        if math.isfinite(number):
            return f"{number:.12g}"
    return "disqualified"


def render_summary(document: Mapping[str, object]) -> str:
    """Render hashes, counts, the best decision, and at most ten ranked rows."""
    ranked = document.get("ranked_candidates", [])
    if not isinstance(ranked, Sequence) or isinstance(ranked, (str, bytes)):
        raise ValueError("ranked_candidates must be a sequence")
    lines = [
        "# Stationary Turn Sweep",
        "",
        f"- Schema: {document.get('schema', '')}",
        f"- Model SHA-256: {document.get('model_sha256', '')}",
        f"- Turn config SHA-256: {document.get('turn_config_sha256', '')}",
        f"- Simulation config SHA-256: {document.get('simulation_config_sha256', '')}",
        f"- PyBullet version: {document.get('pybullet_version', '')}",
        f"- Search candidates: {document.get('search_candidate_count', 0)}",
        f"- Evaluated candidates: {document.get('evaluated_candidate_count', 0)}",
        f"- Surface runs: {document.get('surface_run_count', 0)}",
        f"- Safe candidates: {document.get('safe_candidate_count', 0)}",
        f"- Fall candidates: {document.get('fall_candidate_count', 0)}",
        "",
        "## Best candidate",
        "",
    ]

    safe = [item for item in ranked if isinstance(item, Mapping) and item.get("score") is not None]
    if safe:
        best = safe[0]
        parameters = best.get("parameters", {})
        results = best.get("surface_results", [])
        if not isinstance(parameters, Mapping) or not isinstance(results, Sequence):
            raise ValueError("ranked candidate has malformed parameters or surface results")
        worst_surface = best.get("worst_surface")
        worst = next(
            (item for item in results if isinstance(item, Mapping) and item.get("surface") == worst_surface),
            None,
        )
        yaw_values = [
            float(item["yaw_delta_deg"])
            for item in results
            if isinstance(item, Mapping) and isinstance(item.get("yaw_delta_deg"), (int, float))
        ]
        worst_yaw = worst.get("yaw_delta_deg") if isinstance(worst, Mapping) else None
        lines.extend(
            [
                f"- Parameters: unload {_number(parameters.get('unload_mm'))} mm; tangential "
                f"{_number(parameters.get('tangential_mm'))} mm; hold {_number(parameters.get('hold_s'))} s",
                f"- Best worst-surface score: {_number(best.get('score'))}",
                f"- Best worst-surface yaw: {_number(worst_yaw)} deg ({worst_surface})",
                f"- Consistency: yaw range {_number(min(yaw_values))} to {_number(max(yaw_values))} deg "
                f"(spread {_number(max(yaw_values) - min(yaw_values))} deg)",
                "- Rationale: highest worst-surface score; ties prefer fewer falls, lower translation, "
                "lower orientation deviation, then lexicographic parameters.",
            ]
        )
    else:
        lines.append("No safe candidate produced a finite positive-yaw score.")
        if ranked and isinstance(ranked[0], Mapping):
            fallback = ranked[0]
            parameters = fallback.get("parameters", {})
            results = fallback.get("surface_results", [])
            if not isinstance(parameters, Mapping) or not isinstance(results, Sequence):
                raise ValueError("ranked candidate has malformed parameters or surface results")
            worst_surface = fallback.get("worst_surface")
            worst = next(
                (item for item in results if isinstance(item, Mapping) and item.get("surface") == worst_surface),
                None,
            )
            worst_yaw = worst.get("yaw_delta_deg") if isinstance(worst, Mapping) else None
            reasons = sorted(
                {
                    str(item.get("invalid_reason") or ("fall" if item.get("fell") else "non-positive yaw"))
                    for item in results
                    if isinstance(item, Mapping) and item.get("score") is None
                }
            )
            lines.extend(
                [
                    f"- Ranked fallback parameters: unload {_number(parameters.get('unload_mm'))} mm; "
                    f"tangential {_number(parameters.get('tangential_mm'))} mm; hold {_number(parameters.get('hold_s'))} s",
                    f"- Worst-surface yaw: {_number(worst_yaw)} deg ({worst_surface})",
                    f"- Disqualification: {', '.join(reasons) or 'no finite aggregate score'}.",
                    "- Decision: do not promote a candidate to physical testing.",
                ]
            )

    lines.extend(
        [
            "",
            "## Top 10",
            "",
            "| Rank | Unload mm | Tangential mm | Hold s | Worst surface | Score |",
            "| ---: | ---: | ---: | ---: | :--- | ---: |",
        ]
    )
    for item in ranked[:10]:
        if not isinstance(item, Mapping):
            raise ValueError("ranked candidate must be a mapping")
        parameters = item.get("parameters", {})
        if not isinstance(parameters, Mapping):
            raise ValueError("ranked candidate parameters must be a mapping")
        lines.append(
            f"| {item.get('rank')} | {_number(parameters.get('unload_mm'))} | "
            f"{_number(parameters.get('tangential_mm'))} | {_number(parameters.get('hold_s'))} | "
            f"{item.get('worst_surface', '')} | {_number(item.get('score'))} |"
        )
    return "\n".join(lines) + "\n"


def write_summary(summary: str, path: str | Path) -> Path:
    """Write a normalized Markdown summary and create missing parents."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(summary.rstrip("\r\n") + "\n", encoding="utf-8", newline="\n")
    return destination
