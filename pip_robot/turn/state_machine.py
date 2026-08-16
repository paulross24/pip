"""Fail-closed state transitions for a stationary right-turn run."""

from __future__ import annotations

from enum import Enum, auto


class TurnState(Enum):
    """Phases in the normal and abort paths of a turn run."""

    PRECHECK = auto()
    STAND = auto()
    SETTLE = auto()
    SHIFT_UNLOAD = auto()
    DRIVE_TURN = auto()
    REPLANT = auto()
    RECOVER = auto()
    VERIFY = auto()
    ABORT = auto()
    SAFE_REPLANT = auto()


class TransitionError(RuntimeError):
    """Raised when a requested state transition is not permitted."""


_NEXT_STATE = {
    TurnState.PRECHECK: TurnState.STAND,
    TurnState.STAND: TurnState.SETTLE,
    TurnState.SETTLE: TurnState.SHIFT_UNLOAD,
    TurnState.SHIFT_UNLOAD: TurnState.DRIVE_TURN,
    TurnState.DRIVE_TURN: TurnState.REPLANT,
    TurnState.REPLANT: TurnState.RECOVER,
    TurnState.RECOVER: TurnState.VERIFY,
    TurnState.ABORT: TurnState.SAFE_REPLANT,
    TurnState.SAFE_REPLANT: TurnState.RECOVER,
}


class TurnStateMachine:
    """Track the only permitted state changes without commanding hardware."""

    def __init__(self, state: TurnState = TurnState.PRECHECK) -> None:
        self.state = state

    def transition(self, next_state: TurnState) -> None:
        """Advance to the sole permitted successor of the current state."""
        if not isinstance(next_state, TurnState):
            raise TransitionError("next state must be a TurnState")
        if _NEXT_STATE.get(self.state) is not next_state:
            raise TransitionError(f"cannot transition from {self.state.name} to {next_state.name}")
        self.state = next_state

    def abort(self) -> None:
        """Enter the abort path unless verification has already completed."""
        if self.state is TurnState.VERIFY:
            raise TransitionError("cannot abort after verification")
        self.state = TurnState.ABORT
