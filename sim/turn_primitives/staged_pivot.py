"""Conservative one-foot-at-a-time stepping pivot family."""

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg

from ._targets import stance
from .base import PhaseAction, validate_actions


class StagedPivotPrimitive:
    family = "staged_pivot"

    def build_actions(self, parameters: TurnParameters):
        full = frozenset(Leg)
        d = parameters.tangential_mm
        offsets = {}
        actions = [
            PhaseAction("STAND", stance(), parameters.settle_s, full, frozenset()),
            PhaseAction("SETTLE", stance(), parameters.settle_s, full, frozenset()),
        ]
        for leg, direction in ((Leg.FL, -1.0), (Leg.RR, 1.0)):
            unloaded = frozenset({leg})
            actions.append(PhaseAction(f"LIFT_{leg.value}", stance(x_offsets=offsets, lifts={leg: parameters.unload_mm}), parameters.settle_s, full - unloaded, unloaded))
            offsets = {**offsets, leg: direction * d}
            actions.append(PhaseAction(f"REPOSITION_{leg.value}", stance(x_offsets=offsets, lifts={leg: parameters.unload_mm}), parameters.settle_s, full - unloaded, unloaded))
            actions.append(PhaseAction(f"REPLANT_{leg.value}", stance(x_offsets=offsets), parameters.replant_s, full, frozenset()))
        actions.append(PhaseAction("RECOVER", stance(), parameters.settle_s, full, frozenset()))
        return validate_actions(tuple(actions))
