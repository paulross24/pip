"""Deterministic promotion and ranking for stationary-turn families."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from numbers import Real
from typing import Mapping


SURFACES = ("low", "nominal", "high")


def _finite(value, field_name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")
    return float(value)


@dataclass(frozen=True)
class SurfaceComparison:
    surface_name: str
    yaw_delta_deg: float
    translation_m: float
    max_roll_deg: float
    max_pitch_deg: float
    contact_instability: float
    peak_positive_right_torque_nm: float
    peak_negative_right_torque_nm: float
    cumulative_right_torque_proxy: float
    slip_m: float
    fell: bool
    mechanism_consistent: bool = True
    recovery_cancellation_fraction: float = 0.0

    def __post_init__(self):
        if self.surface_name not in SURFACES:
            raise ValueError("surface_name must be canonical")
        for name, value in vars(self).items():
            if name not in {"surface_name", "fell", "mechanism_consistent"}:
                _finite(value, name)


@dataclass(frozen=True)
class CandidateComparison:
    primitive_family: str
    candidate_id: str
    parameters: Mapping[str, object]
    surfaces: tuple[SurfaceComparison, ...]
    promotable: bool = field(init=False)

    def __post_init__(self):
        if tuple(surface.surface_name for surface in self.surfaces) != SURFACES:
            raise ValueError("candidate must contain low, nominal, high in order")
        promoted = all(
            not surface.fell
            and surface.yaw_delta_deg > 0.0
            and surface.mechanism_consistent
            and surface.translation_m <= 0.020
            and surface.max_roll_deg <= 8.0
            and surface.max_pitch_deg <= 8.0
            and surface.contact_instability <= 0.50
            and surface.slip_m <= 0.100
            and surface.recovery_cancellation_fraction <= 0.50
            for surface in self.surfaces
        )
        object.__setattr__(self, "promotable", promoted)


def rank_candidates(candidates: tuple[CandidateComparison, ...]) -> tuple[CandidateComparison, ...]:
    def key(candidate):
        worst_yaw = min(surface.yaw_delta_deg for surface in candidate.surfaces)
        max_translation = max(surface.translation_m for surface in candidate.surfaces)
        max_attitude = max(max(surface.max_roll_deg, surface.max_pitch_deg) for surface in candidate.surfaces)
        return (
            0 if candidate.promotable else 1,
            max_translation,
            max_attitude,
            -worst_yaw if candidate.promotable else 0.0,
            candidate.primitive_family,
            candidate.candidate_id,
        )
    return tuple(sorted(candidates, key=key))


@dataclass(frozen=True)
class FamilyComparison:
    primitive_family: str
    candidate_ids: tuple[str, ...]
    selected_candidate_id: str | None
    promotable: bool


def rank_families(candidates: tuple[CandidateComparison, ...]) -> tuple[FamilyComparison, ...]:
    grouped: dict[str, list[CandidateComparison]] = {}
    for candidate in candidates:
        grouped.setdefault(candidate.primitive_family, []).append(candidate)
    values = []
    for family, family_candidates in grouped.items():
        ranked = rank_candidates(tuple(family_candidates))
        selected = next((item for item in ranked if item.promotable), None)
        values.append(
            FamilyComparison(
                family,
                tuple(item.candidate_id for item in ranked),
                selected.candidate_id if selected else None,
                selected is not None,
            )
        )
    def family_key(value):
        selected = next(
            (item for item in candidates if item.primitive_family == value.primitive_family and item.candidate_id == value.selected_candidate_id),
            None,
        )
        if selected is None:
            return (1, value.primitive_family)
        return (
            0,
            max(surface.translation_m for surface in selected.surfaces),
            max(max(surface.max_roll_deg, surface.max_pitch_deg) for surface in selected.surfaces),
            -min(surface.yaw_delta_deg for surface in selected.surfaces),
            value.primitive_family,
        )
    return tuple(sorted(values, key=family_key))


def comparison_json_bytes(candidates: tuple[CandidateComparison, ...]) -> bytes:
    document = {
        "schema_version": 1,
        "families": [asdict(value) for value in rank_families(candidates)],
        "candidates": [asdict(value) for value in candidates],
    }
    return (json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
