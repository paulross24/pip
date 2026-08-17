import pytest

from pip_robot.turn.models import TurnParameters
from sim.kinematics import Leg, factory_stance_mm
from sim.turn_primitives.differential_fore_aft import DifferentialForeAftPrimitive
from sim.turn_primitives.same_side_shear import SameSideShearPrimitive
from sim.turn_primitives.staged_pivot import StagedPivotPrimitive


def parameters():
    return TurnParameters("right", "FL_RR", 4.0, 3.0, 0.35, 0.35, 0.35, 1, 20)


@pytest.mark.parametrize(
    "primitive",
    [SameSideShearPrimitive(), DifferentialForeAftPrimitive(), StagedPivotPrimitive()],
)
def test_redesigned_families_are_deterministic_complete_and_recover_neutral(primitive):
    first = primitive.build_actions(parameters())
    assert first == primitive.build_actions(parameters())
    assert 5 <= len(first) <= 10
    assert first[-1].name == "RECOVER"
    if not isinstance(primitive, DifferentialForeAftPrimitive):
        assert {leg: (target.x_mm, target.down_mm) for leg, target in first[-1].targets.items()} == factory_stance_mm


def test_same_side_shear_prewinds_then_loads_before_opposite_shear():
    actions = SameSideShearPrimitive().build_actions(parameters())
    names = tuple(action.name for action in actions)
    assert names.index("PREWIND") < names.index("LOAD_PREWIND") < names.index("DRIVE_SHEAR")
    drive = next(action for action in actions if action.name == "DRIVE_SHEAR")
    assert drive.targets[Leg.FL].x_mm < factory_stance_mm[Leg.FL][0]
    assert drive.targets[Leg.FR].x_mm > factory_stance_mm[Leg.FR][0]


def test_differential_family_forms_right_turn_fore_rear_opposed_trajectory():
    actions = DifferentialForeAftPrimitive().build_actions(parameters())
    drive = next(
        action for action in actions
        if action.name == "DRIVE_FORCE_COUPLE"
    )
    assert drive.targets[Leg.FL].x_mm < factory_stance_mm[Leg.FL][0]
    assert drive.targets[Leg.FR].x_mm < factory_stance_mm[Leg.FR][0]
    assert drive.targets[Leg.RL].x_mm > factory_stance_mm[Leg.RL][0]
    assert drive.targets[Leg.RR].x_mm > factory_stance_mm[Leg.RR][0]
    recover = actions[-1]
    assert recover.name == "RECOVER"
    assert recover.targets == drive.targets
    assert recover.expected_support == frozenset(Leg)


def test_staged_pivot_never_expects_a_lifted_foot_to_support():
    actions = StagedPivotPrimitive().build_actions(parameters())
    lifted = [action for action in actions if action.unloaded_feet]
    assert len(lifted) >= 2
    assert all(not (action.expected_support & action.unloaded_feet) for action in lifted)
    for index, action in enumerate(actions):
        if action.name.startswith("LIFT_"):
            leg = next(iter(action.unloaded_feet))
            assert actions[index + 1].unloaded_feet == frozenset({leg})
            assert actions[index + 2].name.startswith("REPLANT_")
