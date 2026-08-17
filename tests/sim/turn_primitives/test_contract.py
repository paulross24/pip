import pytest

from sim.kinematics import Leg
from sim.turn_primitives.base import FootTarget, PhaseAction, validate_actions


def targets():
    return {leg: FootTarget(0.0, 90.0) for leg in Leg}


def test_action_requires_every_leg_and_disjoint_support_unload():
    with pytest.raises(ValueError, match="every canonical leg"):
        PhaseAction("BAD", {Leg.FL: FootTarget(0.0, 90.0)}, 0.1, frozenset(), frozenset())
    with pytest.raises(ValueError, match="support.*unloaded"):
        PhaseAction("BAD", targets(), 0.1, frozenset({Leg.FL}), frozenset({Leg.FL}))


def test_action_sequence_is_nonempty_and_ends_in_recovery():
    action = PhaseAction("RECOVER", targets(), 0.1, frozenset(Leg), frozenset())
    assert validate_actions((action,)) == (action,)
    with pytest.raises(ValueError, match="nonempty"):
        validate_actions(())

