"""Small target-building helpers shared by pure primitive generators."""

from sim.kinematics import Leg, factory_stance_mm

from .base import FootTarget


def stance(*, x_offsets=None, lifts=None):
    x_offsets = x_offsets or {}
    lifts = lifts or {}
    return {
        leg: FootTarget(
            factory_stance_mm[leg][0] + x_offsets.get(leg, 0.0),
            factory_stance_mm[leg][1] - lifts.get(leg, 0.0),
        )
        for leg in Leg
    }
