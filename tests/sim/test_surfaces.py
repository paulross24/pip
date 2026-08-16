from __future__ import annotations

from dataclasses import is_dataclass
import math

import pytest

from sim.surfaces import Surface, required_surfaces


def test_required_surfaces_are_ordered_and_have_the_exact_friction_family() -> None:
    surfaces = required_surfaces()

    assert [(surface.name, surface.friction) for surface in surfaces] == [
        ("low", 0.45),
        ("nominal", 0.70),
        ("high", 0.95),
    ]
    assert isinstance(surfaces, tuple)


def test_surface_is_a_frozen_contract() -> None:
    assert is_dataclass(Surface)
    assert Surface.__dataclass_params__.frozen is True

    surface = Surface("low", 0.45)
    with pytest.raises(Exception):
        surface.friction = 0.50  # type: ignore[misc]


@pytest.mark.parametrize("friction", [0.0, -0.1, math.nan, math.inf, True])
def test_surface_rejects_non_positive_or_non_finite_friction(friction: object) -> None:
    with pytest.raises(ValueError, match="friction"):
        Surface("test", friction)  # type: ignore[arg-type]


@pytest.mark.parametrize("name", ["", "   ", 1])
def test_surface_requires_a_nonblank_name(name: object) -> None:
    with pytest.raises(ValueError, match="name"):
        Surface(name, 0.45)  # type: ignore[arg-type]
