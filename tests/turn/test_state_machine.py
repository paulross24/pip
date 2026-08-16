from __future__ import annotations

import pytest

from pip_robot.turn.state_machine import TransitionError, TurnState, TurnStateMachine


def test_machine_starts_in_precheck_and_follows_the_canonical_path() -> None:
    machine = TurnStateMachine()

    assert machine.state is TurnState.PRECHECK

    for state in (
        TurnState.STAND,
        TurnState.SETTLE,
        TurnState.SHIFT_UNLOAD,
        TurnState.DRIVE_TURN,
        TurnState.REPLANT,
        TurnState.RECOVER,
        TurnState.VERIFY,
    ):
        machine.transition(state)
        assert machine.state is state


def test_illegal_transition_raises_and_preserves_the_current_state() -> None:
    machine = TurnStateMachine()

    with pytest.raises(TransitionError):
        machine.transition(TurnState.SETTLE)

    assert machine.state is TurnState.PRECHECK


def test_non_state_transition_raises_and_preserves_the_current_state() -> None:
    machine = TurnStateMachine()

    with pytest.raises(TransitionError):
        machine.transition("STAND")  # type: ignore[arg-type]

    assert machine.state is TurnState.PRECHECK


@pytest.mark.parametrize("starting_state", [TurnState.STAND, TurnState.DRIVE_TURN])
def test_abort_from_movement_preparation_or_drive_requires_safe_replant(
    starting_state: TurnState,
) -> None:
    machine = TurnStateMachine()

    while machine.state is not starting_state:
        machine.transition(
            {
                TurnState.PRECHECK: TurnState.STAND,
                TurnState.STAND: TurnState.SETTLE,
                TurnState.SETTLE: TurnState.SHIFT_UNLOAD,
                TurnState.SHIFT_UNLOAD: TurnState.DRIVE_TURN,
            }[machine.state]
        )

    machine.abort()

    assert machine.state is TurnState.ABORT
    with pytest.raises(TransitionError):
        machine.transition(TurnState.DRIVE_TURN)
    assert machine.state is TurnState.ABORT


def test_abort_path_finishes_with_safe_replant_recovery_and_verification() -> None:
    machine = TurnStateMachine(TurnState.DRIVE_TURN)

    machine.abort()
    machine.transition(TurnState.SAFE_REPLANT)
    machine.transition(TurnState.RECOVER)
    machine.transition(TurnState.VERIFY)

    assert machine.state is TurnState.VERIFY


def test_abort_is_rejected_after_verification_and_keeps_the_final_state() -> None:
    machine = TurnStateMachine(TurnState.VERIFY)

    with pytest.raises(TransitionError):
        machine.abort()

    assert machine.state is TurnState.VERIFY
