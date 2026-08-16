"""The fixed, deterministic contact-surface family for simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


@dataclass(frozen=True)
class Surface:
    """One named plane friction setting."""

    name: str
    friction: float

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("name must be a nonblank string")
        if isinstance(self.friction, bool) or not isinstance(self.friction, Real) or not math.isfinite(self.friction):
            raise ValueError("friction must be a positive finite number")
        if self.friction <= 0.0:
            raise ValueError("friction must be a positive finite number")
        object.__setattr__(self, "name", self.name.strip())
        object.__setattr__(self, "friction", float(self.friction))


_REQUIRED_SURFACES = (
    Surface("low", 0.45),
    Surface("nominal", 0.70),
    Surface("high", 0.95),
)


def required_surfaces() -> tuple[Surface, Surface, Surface]:
    """Return the immutable low, nominal, high friction family in fixed order."""
    return _REQUIRED_SURFACES
