"""Append-only JSONL telemetry for stationary right-turn runs."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import json
from pathlib import Path
import time
from typing import Any
from uuid import uuid4


TELEMETRY_SCHEMA = "pip-turn-run/v1"
_REQUIRED_EVENT_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "utc_timestamp",
        "monotonic_s",
        "git_revision",
        "parameters",
        "state",
        "event_type",
    }
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def new_run_id() -> str:
    """Return a new opaque identifier for one telemetry run."""
    return str(uuid4())


def _normalize(value: Any) -> Any:
    """Convert dataclasses and enums into JSON-compatible values."""
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: _normalize(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Enum):
        return _normalize(value.value)
    if isinstance(value, Mapping):
        return {str(key): _normalize(item) for key, item in value.items()}
    if isinstance(value, tuple | list):
        return [_normalize(item) for item in value]
    return value


class TurnTelemetryWriter:
    """Write normalized turn events as one appended JSON object per line."""

    def __init__(
        self,
        path: str | Path,
        git_revision: str,
        utc_now: Callable[[], str] = _utc_now,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._path = Path(path)
        self._git_revision = git_revision
        self._utc_now = utc_now
        self._monotonic = monotonic

    def record_event(
        self,
        run_id: str,
        parameters: Any,
        state: Any,
        event_type: str,
        **optional_fields: Any,
    ) -> dict[str, Any]:
        """Append an event without changing any existing telemetry lines."""
        reserved_fields = set(optional_fields) & _REQUIRED_EVENT_FIELDS
        if reserved_fields:
            raise ValueError(f"optional fields cannot use reserved names: {sorted(reserved_fields)}")

        event = {
            "schema": TELEMETRY_SCHEMA,
            "run_id": run_id,
            "utc_timestamp": self._utc_now(),
            "monotonic_s": self._monotonic(),
            "git_revision": self._git_revision,
            "parameters": _normalize(parameters),
            "state": state.name if isinstance(state, Enum) else _normalize(state),
            "event_type": event_type,
        }
        event.update(
            {
                key: value.name if key == "abort_phase" and isinstance(value, Enum) else _normalize(value)
                for key, value in optional_fields.items()
            }
        )

        serialized_event = json.dumps(event, allow_nan=False, separators=(",", ":"))
        needs_separator = False
        if self._path.exists() and self._path.stat().st_size:
            with self._path.open("rb") as existing_file:
                existing_file.seek(-1, 2)
                needs_separator = existing_file.read(1) != b"\n"

        self._path.parent.mkdir(parents=True, exist_ok=True)
        with self._path.open("a", encoding="utf-8", newline="\n") as telemetry_file:
            if needs_separator:
                telemetry_file.write("\n")
            telemetry_file.write(serialized_event)
            telemetry_file.write("\n")

        return event
