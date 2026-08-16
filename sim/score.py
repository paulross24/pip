"""Transparent safety-first scoring for measured simulation results."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .model import SimulationResult


YAW_WEIGHT = 10.0
TRANSLATION_WEIGHT = 1000.0
ROLL_WEIGHT = 2.0
PITCH_WEIGHT = 2.0
CONTACT_INSTABILITY_WEIGHT = 5.0
_SURFACE_ORDER = {"low": 0, "nominal": 1, "high": 2}


@dataclass(frozen=True)
class CandidateScore:
    """Worst-surface score plus stable data used to break ranking ties."""

    score: float
    worst_surface_name: str
    surface_scores: tuple[tuple[str, float], ...]
    tie_break_data: tuple[int, float, float, tuple[float, float, float, float, float, int, int]]


def score_result(result: SimulationResult) -> float:
    """Score a safe positive-right-yaw result, otherwise disqualify it."""
    if not isinstance(result, SimulationResult):
        raise TypeError("result must be a SimulationResult")
    metrics = (
        result.yaw_delta_deg,
        result.translation_m,
        result.max_roll_deviation_deg,
        result.max_pitch_deviation_deg,
        result.contact_instability,
    )
    if (
        result.fell
        or result.aborted
        or result.invalid_reason is not None
        or result.yaw_delta_deg <= 0.0
        or any(not math.isfinite(metric) for metric in metrics)
    ):
        return -math.inf
    return (
        YAW_WEIGHT * result.yaw_delta_deg
        - TRANSLATION_WEIGHT * result.translation_m
        - ROLL_WEIGHT * result.max_roll_deviation_deg
        - PITCH_WEIGHT * result.max_pitch_deviation_deg
        - CONTACT_INSTABILITY_WEIGHT * result.contact_instability
    )


def aggregate_candidate_score(results: Iterable[SimulationResult]) -> CandidateScore:
    """Reduce one candidate's surface measurements to its worst-surface score."""
    candidates = tuple(results)
    if not candidates or any(not isinstance(result, SimulationResult) for result in candidates):
        raise ValueError("results must contain SimulationResult values")
    parameters = candidates[0].parameters
    if any(result.parameters != parameters for result in candidates[1:]):
        raise ValueError("results must share identical parameters")
    if len({result.surface_name for result in candidates}) != len(candidates):
        raise ValueError("results must contain one result per surface")

    scored = tuple((result, score_result(result)) for result in candidates)
    ordered = tuple(sorted(scored, key=lambda item: (_surface_rank(item[0].surface_name), item[0].surface_name)))
    worst_result, worst_score = min(
        ordered,
        key=lambda item: (item[1], _surface_rank(item[0].surface_name), item[0].surface_name),
    )
    tie_break_data = (
        sum(result.fell for result, _ in ordered),
        max(result.translation_m for result, _ in ordered),
        max(result.max_roll_deviation_deg + result.max_pitch_deviation_deg for result, _ in ordered),
        (
            parameters.unload_mm,
            parameters.tangential_mm,
            parameters.hold_s,
            parameters.settle_s,
            parameters.replant_s,
            parameters.cycles,
            parameters.speed,
        ),
    )
    return CandidateScore(
        score=worst_score,
        worst_surface_name=worst_result.surface_name,
        surface_scores=tuple((result.surface_name, score) for result, score in ordered),
        tie_break_data=tie_break_data,
    )


def _surface_rank(name: str) -> int:
    return _SURFACE_ORDER.get(name, len(_SURFACE_ORDER))
