"""Run and serialize the bounded deterministic stationary-turn sweep."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
from itertools import product
import json
import math
from pathlib import Path
from typing import Callable, Iterator, Mapping, Sequence

from pip_robot.turn.models import TurnParameters
from sim.model import SimulationResult
from sim.pivot_runner import (
    SimulationSettings,
    direct_client_factory,
    load_simulation_settings,
    load_turn_parameters,
    run_candidate,
)
from sim.score import aggregate_candidate_score, score_result
from sim.surfaces import Surface, required_surfaces


SCHEMA = "pip-sim-turn-sweep/v1"
UNLOAD_VALUES_MM = (2.0, 3.0, 4.0, 5.0, 6.0)
TANGENTIAL_VALUES_MM = (1.0, 2.0, 3.0, 4.0, 5.0)
HOLD_VALUES_S = (0.25, 0.30, 0.35, 0.40, 0.45)

Runner = Callable[[TurnParameters, Surface, SimulationSettings, object], SimulationResult]


def candidate_parameters(baseline: TurnParameters) -> Iterator[TurnParameters]:
    """Yield the exact fixed search product while preserving baseline fields."""
    if not isinstance(baseline, TurnParameters):
        raise TypeError("baseline must be TurnParameters")
    for unload_mm, tangential_mm, hold_s in product(
        UNLOAD_VALUES_MM,
        TANGENTIAL_VALUES_MM,
        HOLD_VALUES_S,
    ):
        yield replace(
            baseline,
            unload_mm=unload_mm,
            tangential_mm=tangential_mm,
            hold_s=hold_s,
        )


def run_sweep(
    baseline: TurnParameters,
    settings: SimulationSettings,
    *,
    runner: Runner = run_candidate,
    client_factory: object = direct_client_factory,
    limit: int | None = None,
    model_sha256: str = "",
    config_sha256: str = "",
) -> dict[str, object]:
    """Evaluate candidates on every canonical surface and rank deterministically."""
    if not isinstance(settings, SimulationSettings):
        raise TypeError("settings must be SimulationSettings")
    if limit is not None and (isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0):
        raise ValueError("limit must be a positive integer or None")

    all_candidates = tuple(candidate_parameters(baseline))
    selected = all_candidates if limit is None else all_candidates[:limit]
    ranked: list[tuple[tuple[object, ...], dict[str, object]]] = []
    fall_candidate_count = 0
    safe_candidate_count = 0

    for parameters in selected:
        results = tuple(
            runner(parameters, surface, settings, client_factory)
            for surface in required_surfaces()
        )
        if any(result.parameters != parameters for result in results):
            raise ValueError("runner results must preserve candidate parameters")
        aggregate = aggregate_candidate_score(results)
        safe = math.isfinite(aggregate.score)
        fell = any(result.fell for result in results)
        safe_candidate_count += int(safe)
        fall_candidate_count += int(fell)
        rank_key = (
            0 if safe else 1,
            -aggregate.score if safe else 0.0,
            *aggregate.tie_break_data,
        )
        ranked.append(
            (
                rank_key,
                {
                    "parameters": asdict(parameters),
                    "score": aggregate.score if safe else None,
                    "worst_surface": aggregate.worst_surface_name,
                    "fell": fell,
                    "surface_results": [
                        _surface_result_mapping(result, score_result(result))
                        for result in results
                    ],
                },
            )
        )

    ranked.sort(key=lambda item: item[0])
    ranked_candidates = []
    for rank, (_, candidate) in enumerate(ranked, start=1):
        ranked_candidates.append({"rank": rank, **candidate})

    return {
        "schema": SCHEMA,
        "model_sha256": model_sha256,
        "config_sha256": config_sha256,
        "search_candidate_count": len(all_candidates),
        "evaluated_candidate_count": len(selected),
        "surface_run_count": len(selected) * len(required_surfaces()),
        "safe_candidate_count": safe_candidate_count,
        "fall_candidate_count": fall_candidate_count,
        "ranked_candidates": ranked_candidates,
    }


def _surface_result_mapping(result: SimulationResult, score: float) -> dict[str, object]:
    return {
        "surface": result.surface_name,
        "friction": result.friction,
        "score": score if math.isfinite(score) else None,
        "yaw_delta_deg": result.yaw_delta_deg,
        "translation_m": result.translation_m,
        "max_roll_deviation_deg": result.max_roll_deviation_deg,
        "max_pitch_deviation_deg": result.max_pitch_deviation_deg,
        "contact_instability": result.contact_instability,
        "fell": result.fell,
        "aborted": result.aborted,
        "invalid_reason": result.invalid_reason,
    }


def write_ranked_json(document: Mapping[str, object], path: str | Path) -> Path:
    """Write valid canonical JSON, creating its parent directory as needed."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        document,
        allow_nan=False,
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
    ) + "\n"
    destination.write_text(payload, encoding="utf-8", newline="\n")
    return destination


def sha256_file(path: str | Path) -> str:
    """Return the lowercase SHA-256 digest of one input file."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/turn_right_baseline.json")
    parser.add_argument("--simulation-config", default="config/simulation.json")
    parser.add_argument("--ranked-output", default="runs/sim/latest-ranked.json")
    parser.add_argument("--summary-output", default="runs/sim/latest-summary.md")
    parser.add_argument("--limit", type=int, help="evaluate only the first N candidates for diagnostics")
    args = parser.parse_args(argv)

    baseline = load_turn_parameters(args.config)
    settings = load_simulation_settings(args.simulation_config)
    document = run_sweep(
        baseline,
        settings,
        limit=args.limit,
        model_sha256=sha256_file(settings.model_path),
        config_sha256=sha256_file(args.config),
    )
    write_ranked_json(document, args.ranked_output)

    from .summarize_turn import render_summary, write_summary

    write_summary(render_summary(document), args.summary_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
