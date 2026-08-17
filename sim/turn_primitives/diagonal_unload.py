"""Milestone 2 diagonal-unload control primitive."""

from dataclasses import dataclass

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg

from .base import FootTarget, PhaseAction, validate_actions


@dataclass(frozen=True)
class DiagonalUnloadPrimitive:
    family = "diagonal_unload"

    def build_actions(self, parameters: TurnParameters) -> tuple[PhaseAction, ...]:
        if not isinstance(parameters, TurnParameters):
            raise ValueError("parameters must be TurnParameters")
        # Import lazily to preserve the control implementation without creating
        # a module-import cycle while pivot_runner becomes a compatibility shim.
        from sim.pivot_runner import phase_endpoint_targets

        endpoints = phase_endpoint_targets(parameters)
        durations = {
            "STAND": parameters.settle_s,
            "SETTLE": parameters.settle_s,
            "SHIFT_UNLOAD": parameters.settle_s,
            "DRIVE_TURN": parameters.settle_s,
            "REPLANT": parameters.replant_s,
            "RECOVER": parameters.settle_s,
        }
        actions = []
        for name, values in endpoints.items():
            unloaded = frozenset({Leg.FL, Leg.RR}) if name in {"SHIFT_UNLOAD", "DRIVE_TURN"} else frozenset()
            support = frozenset(Leg) - unloaded
            actions.append(
                PhaseAction(
                    name,
                    {leg: FootTarget(*values[leg]) for leg in Leg},
                    durations[name],
                    support,
                    unloaded,
                    parameters.hold_s if name == "DRIVE_TURN" else 0.0,
                )
            )
        return validate_actions(tuple(actions))
