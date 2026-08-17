from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg
from sim.pivot_runner import phase_endpoint_targets
from sim.turn_primitives.diagonal_unload import DiagonalUnloadPrimitive


def baseline():
    return TurnParameters("right", "FL_RR", 4.0, 3.0, 0.35, 0.35, 0.35, 1, 20)


def test_control_family_matches_milestone_two_endpoints_and_phase_order():
    parameters = baseline()
    actions = DiagonalUnloadPrimitive().build_actions(parameters)
    expected = phase_endpoint_targets(parameters)
    assert tuple(action.name for action in actions) == tuple(expected)
    for action in actions:
        assert {leg: (target.x_mm, target.down_mm) for leg, target in action.targets.items()} == expected[action.name]
    assert actions[2].unloaded_feet == frozenset({Leg.FL, Leg.RR})
    assert actions[-1].expected_support == frozenset(Leg)


def test_control_sequence_is_deterministic():
    primitive = DiagonalUnloadPrimitive()
    assert primitive.build_actions(baseline()) == primitive.build_actions(baseline())
