from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json

from pip_robot.turn.models import HeadingObservation, ImuSample, SafetyDecision, TurnParameters, TurnResult
from pip_robot.turn.state_machine import TurnState
from pip_robot.turn.telemetry import TurnTelemetryWriter, new_run_id


BASELINE = {
    "direction": "right",
    "unload_pair": "FL_RR",
    "unload_mm": 4.0,
    "tangential_mm": 3.0,
    "hold_s": 0.35,
    "settle_s": 0.35,
    "replant_s": 0.35,
    "cycles": 1,
    "speed": 20,
}


class BatteryStatus(Enum):
    READY = "ready"


@dataclass(frozen=True)
class Target:
    yaw_deg: float


def test_record_event_appends_normalized_before_and_after_phase_records(tmp_path) -> None:
    path = tmp_path / "turn.jsonl"
    timestamps = iter(["2026-08-16T12:00:00+00:00", "2026-08-16T12:00:01+00:00"])
    clock = iter([101.25, 102.5])
    writer = TurnTelemetryWriter(path, "abc123", utc_now=lambda: next(timestamps), monotonic=lambda: next(clock))
    parameters = TurnParameters.from_mapping(BASELINE)
    run_id = "run-123"

    before = writer.record_event(
        run_id,
        parameters,
        TurnState.SETTLE,
        "before_phase",
        target=Target(yaw_deg=90.0),
        imu=ImuSample(roll_deg=1.0, pitch_deg=2.0),
        battery=BatteryStatus.READY,
        measured_heading=HeadingObservation(heading_deg=None, source="compass"),
        heading_delta_deg=None,
    )
    after = writer.record_event(run_id, parameters, TurnState.SHIFT_UNLOAD, "after_phase")

    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert [json.loads(line) for line in lines] == [before, after]
    assert before == {
        "schema": "pip-turn-run/v1",
        "run_id": "run-123",
        "utc_timestamp": "2026-08-16T12:00:00+00:00",
        "monotonic_s": 101.25,
        "git_revision": "abc123",
        "parameters": BASELINE,
        "state": "SETTLE",
        "event_type": "before_phase",
        "target": {"yaw_deg": 90.0},
        "imu": {
            "roll_deg": 1.0,
            "pitch_deg": 2.0,
            "accel_xyz": None,
            "gyro_xyz": None,
            "monotonic_s": None,
            "utc_timestamp": None,
            "valid": True,
        },
        "battery": "ready",
        "measured_heading": {
            "heading_deg": None,
            "source": "compass",
            "observed_at": None,
        },
        "heading_delta_deg": None,
    }
    assert after["utc_timestamp"] == "2026-08-16T12:00:01+00:00"
    assert after["monotonic_s"] == 102.5
    assert after["state"] == "SHIFT_UNLOAD"
    assert set(after) == {
        "schema",
        "run_id",
        "utc_timestamp",
        "monotonic_s",
        "git_revision",
        "parameters",
        "state",
        "event_type",
    }


def test_record_event_preserves_abort_recovery_and_final_result_fields(tmp_path) -> None:
    path = tmp_path / "existing.jsonl"
    path.write_text('{"previous":"record"}\n', encoding="utf-8")
    writer = TurnTelemetryWriter(path, "rev", utc_now=lambda: "2026-08-16T12:02:00+00:00", monotonic=lambda: 7.0)

    event = writer.record_event(
        "run-abort",
        TurnParameters.from_mapping(BASELINE),
        TurnState.ABORT,
        "abort",
        abort_phase=TurnState.DRIVE_TURN,
        abort_reason="pitch_error_limit",
        safe_replant={"completed": True},
        recovery={"outcome": "stable"},
        final_result=TurnResult(False, True, "VERIFY", "pitch_error_limit", None),
        safety=SafetyDecision(False, "pitch_error_limit", pitch_error_deg=9.0),
        pose={"roll_deg": 1.0, "pitch_deg": 2.0},
    )

    lines = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert lines[0] == {"previous": "record"}
    assert lines[1] == event
    assert event["abort_phase"] == "DRIVE_TURN"
    assert event["abort_reason"] == "pitch_error_limit"
    assert event["safe_replant"] == {"completed": True}
    assert event["recovery"] == {"outcome": "stable"}
    assert event["final_result"]["measured_heading_delta_deg"] is None
    assert event["safety"]["allowed"] is False
    assert event["pose"] == {"roll_deg": 1.0, "pitch_deg": 2.0}


def test_new_run_id_returns_distinct_non_empty_strings() -> None:
    first = new_run_id()
    second = new_run_id()

    assert isinstance(first, str) and first
    assert isinstance(second, str) and second
    assert first != second
