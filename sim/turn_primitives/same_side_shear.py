"""Same-side prewind/shear family inspired by the completed historical proof."""

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg

from ._targets import stance
from .base import PhaseAction, validate_actions


class SameSideShearPrimitive:
    family = "same_side_shear"

    def build_actions(self, parameters: TurnParameters):
        d = parameters.tangential_mm
        unload = frozenset({Leg.FL, Leg.RR})
        full = frozenset(Leg)
        prewind_offsets = {Leg.FL: d, Leg.RL: d, Leg.FR: -d, Leg.RR: -d}
        drive_offsets = {Leg.FL: -d, Leg.RL: -d, Leg.FR: d, Leg.RR: d}
        actions = (
            PhaseAction("STAND", stance(), parameters.settle_s, full, frozenset()),
            PhaseAction("SETTLE", stance(), parameters.settle_s, full, frozenset()),
            PhaseAction("PREWIND", stance(x_offsets=prewind_offsets, lifts={leg: parameters.unload_mm for leg in unload}), parameters.settle_s, full - unload, unload),
            PhaseAction("LOAD_PREWIND", stance(x_offsets=prewind_offsets), parameters.replant_s, full, frozenset()),
            PhaseAction("DRIVE_SHEAR", stance(x_offsets=drive_offsets), parameters.settle_s, full, frozenset(), parameters.hold_s),
            PhaseAction("REPLANT", stance(x_offsets=drive_offsets), parameters.replant_s, full, frozenset()),
            PhaseAction("RECOVER", stance(), parameters.settle_s, full, frozenset()),
        )
        return validate_actions(actions)
