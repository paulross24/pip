"""Real, headless PyBullet integration coverage."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys

import pybullet

from sim.pivot_runner import (
    direct_client_factory,
    load_simulation_settings,
    load_turn_parameters,
    run_candidate,
)
from sim.surfaces import required_surfaces


ROOT = Path(__file__).parents[2]


def _assert_finite_metrics(value: object) -> None:
    if isinstance(value, dict):
        for item in value.values():
            _assert_finite_metrics(item)
    elif isinstance(value, (tuple, list)):
        for item in value:
            _assert_finite_metrics(item)
    elif isinstance(value, float):
        assert math.isfinite(value)


def test_real_direct_smoke_returns_finite_structured_metrics() -> None:
    client = direct_client_factory()
    try:
        assert client.getConnectionInfo()["connectionMethod"] == pybullet.DIRECT
    finally:
        client.disconnect()

    result = run_candidate(
        load_turn_parameters(ROOT / "config" / "turn_right_baseline.json"),
        required_surfaces()[1],
        load_simulation_settings(ROOT / "config" / "simulation.json"),
        direct_client_factory,
    )

    metrics = asdict(result)
    assert set(metrics) == {
        "parameters",
        "surface_name",
        "friction",
        "yaw_delta_deg",
        "translation_x_m",
        "translation_y_m",
        "translation_m",
        "max_roll_deviation_deg",
        "max_pitch_deviation_deg",
        "fell",
        "contact_instability",
        "elapsed_sim_s",
        "final_pose",
        "foot_contacts",
        "aborted",
        "invalid_reason",
    }
    _assert_finite_metrics(metrics)
    assert result.invalid_reason is None


def test_smoke_cli_stdout_is_one_structured_json_result() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "sim.pivot_runner",
            "--config",
            "config/turn_right_baseline.json",
            "--smoke",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    result = json.loads(completed.stdout)
    _assert_finite_metrics(result)
    assert result["invalid_reason"] is None
