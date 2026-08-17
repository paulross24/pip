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
            -worst_yaw if candidate.promotable else 0.0,
            max_translation,
            max_attitude,
            candidate.primitive_family,
            candidate.candidate_id,
        )
    return tuple(sorted(candidates, key=key))


def comparison_json_bytes(candidates: tuple[CandidateComparison, ...]) -> bytes:
    document = {"schema_version": 1, "candidates": [asdict(value) for value in candidates]}
    return (json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
