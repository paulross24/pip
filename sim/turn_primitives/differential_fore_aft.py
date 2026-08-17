"""Loaded left/right differential fore-aft force-couple family."""

from dataclasses import dataclass

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg

from ._targets import stance
from .base import PhaseAction, validate_actions


@dataclass(frozen=True)
class DifferentialForeAftPrimitive:
    family = "differential_fore_aft"

    def build_actions(self, parameters: TurnParameters):
        d = parameters.tangential_mm
        full = frozenset(Leg)
        unload = frozenset({Leg.FR, Leg.RR})
        offsets = {Leg.FL: -d, Leg.RL: -d, Leg.FR: d, Leg.RR: d}
        actions = (
            PhaseAction("STAND", stance(), parameters.settle_s, full, frozenset()),
            PhaseAction("SETTLE", stance(), parameters.settle_s, full, frozenset()),
            PhaseAction("TRANSFER_LEFT", stance(lifts={leg: parameters.unload_mm for leg in unload}), parameters.settle_s, full - unload, unload),
            PhaseAction("LOAD_ALL", stance(), parameters.replant_s, full, frozenset()),
            PhaseAction("DRIVE_FORCE_COUPLE", stance(x_offsets=offsets), parameters.settle_s, full, frozenset(), parameters.hold_s),
            PhaseAction("REPLANT", stance(x_offsets=offsets), parameters.replant_s, full, frozenset()),
            PhaseAction("RECOVER", stance(), parameters.settle_s, full, frozenset()),
        )
        return validate_actions(actions)
