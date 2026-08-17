"""Typed, client-free contract shared by every simulated turn primitive."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Mapping, Protocol, TypeVar

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg


def _finite(value: object, field: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    result = float(value)
    if positive and result <= 0.0:
        raise ValueError(f"{field} must be positive")
    return result


@dataclass(frozen=True)
class FootTarget:
    x_mm: float
    down_mm: float

    def __post_init__(self) -> None:
        _finite(self.x_mm, "x_mm")
        _finite(self.down_mm, "down_mm", positive=True)


@dataclass(frozen=True)
class PhaseAction:
    name: str
    targets: Mapping[Leg, FootTarget]
    duration_s: float
    expected_support: frozenset[Leg]
    unloaded_feet: frozenset[Leg]

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("phase name must be nonblank")
        if set(self.targets) != set(Leg):
            raise ValueError("targets must contain every canonical leg")
        if any(not isinstance(value, FootTarget) for value in self.targets.values()):
            raise ValueError("targets must be FootTarget values")
        _finite(self.duration_s, "duration_s", positive=True)
        if not self.expected_support <= frozenset(Leg) or not self.unloaded_feet <= frozenset(Leg):
            raise ValueError("support and unloaded feet must be canonical")
        if self.expected_support & self.unloaded_feet:
            raise ValueError("expected support and unloaded feet must be disjoint")


def validate_actions(actions: tuple[PhaseAction, ...]) -> tuple[PhaseAction, ...]:
    if not actions:
        raise ValueError("primitive actions must be nonempty")
    if actions[-1].name != "RECOVER":
        raise ValueError("primitive actions must end in RECOVER")
    for action in actions:
        action.__post_init__()
    return actions


P = TypeVar("P")


class TurnPrimitive(Protocol[P]):
    family: str

    def build_actions(self, parameters: P) -> tuple[PhaseAction, ...]: ...

