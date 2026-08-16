from __future__ import annotations

import json
import math
from pathlib import Path

from pip_robot.turn.models import TurnParameters
from sim.model import FinalPose, FootContact, SimulationResult
from sim.pivot_runner import SimulationSettings
from sim.surfaces import Surface
from tools.sweep_turn import candidate_parameters, run_sweep, write_ranked_json


BASELINE = TurnParameters(
    direction="right",
    unload_pair="FL_RR",
    unload_mm=4.0,
    tangential_mm=3.0,
    hold_s=0.35,
    settle_s=0.35,
    replant_s=0.35,
    cycles=1,
    speed=20,
)
SETTINGS = SimulationSettings(
    time_step_s=1.0 / 240.0,
    solver_iterations=80,
    gravity_xyz=(0.0, 0.0, -9.81),
    spawn_height_m=0.14,
    initial_settle_s=1.5,
    fall_roll_deg=10.0,
    fall_pitch_deg=12.0,
    min_height_m=0.075,
    support_loss_duration_s=0.10,
    model_path="models/pip.urdf",
)
POSE = FinalPose((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0)
CONTACTS = tuple(FootContact(foot, True, 1.0) for foot in ("FL", "FR", "RL", "RR"))


def measured(
    parameters: TurnParameters,
    surface: Surface,
    *,
    yaw: float = 8.0,
    translation: float = 0.01,
    roll: float = 1.0,
    pitch: float = 2.0,
    fell: bool = False,
) -> SimulationResult:
    return SimulationResult(
        parameters=parameters,
        surface_name=surface.name,
        friction=surface.friction,
        yaw_delta_deg=yaw,
        translation_x_m=translation,
        translation_y_m=0.0,
        translation_m=translation,
        max_roll_deviation_deg=roll,
        max_pitch_deviation_deg=pitch,
        fell=fell,
        contact_instability=0.1,
        elapsed_sim_s=1.0,
        final_pose=POSE,
        foot_contacts=CONTACTS,
        aborted=fell,
        invalid_reason="fall-detected" if fell else None,
    )


def test_candidate_parameters_are_the_exact_125_item_product_and_preserve_baseline_fields() -> None:
    candidates = tuple(candidate_parameters(BASELINE))

    assert len(candidates) == 125
    assert all(type(candidate) is TurnParameters for candidate in candidates)
    assert [(item.unload_mm, item.tangential_mm, item.hold_s) for item in candidates[:7]] == [
        (2.0, 1.0, 0.25),
        (2.0, 1.0, 0.30),
        (2.0, 1.0, 0.35),
        (2.0, 1.0, 0.40),
        (2.0, 1.0, 0.45),
        (2.0, 2.0, 0.25),
        (2.0, 2.0, 0.30),
    ]
    assert (candidates[-1].unload_mm, candidates[-1].tangential_mm, candidates[-1].hold_s) == (
        6.0,
        5.0,
        0.45,
    )
    assert {
        (
            item.direction,
            item.unload_pair,
            item.settle_s,
            item.replant_s,
            item.cycles,
            item.speed,
        )
        for item in candidates
    } == {("right", "FL_RR", 0.35, 0.35, 1, 20)}


def test_run_sweep_evaluates_each_limited_candidate_on_all_three_surfaces() -> None:
    calls: list[tuple[TurnParameters, str, object]] = []

    def fake_runner(
        parameters: TurnParameters,
        surface: Surface,
        settings: SimulationSettings,
        client_factory: object,
    ) -> SimulationResult:
        assert settings is SETTINGS
        calls.append((parameters, surface.name, client_factory))
        return measured(parameters, surface, yaw=10.0 - surface.friction)

    sentinel_factory = object()
    sweep = run_sweep(
        BASELINE,
        SETTINGS,
        runner=fake_runner,
        client_factory=sentinel_factory,
        limit=2,
        model_sha256="model-hash",
        turn_config_sha256="turn-hash",
        simulation_config_sha256="simulation-hash",
        pybullet_version="3.2.7",
    )

    assert [(item[0].hold_s, item[1]) for item in calls] == [
        (0.25, "low"),
        (0.25, "nominal"),
        (0.25, "high"),
        (0.30, "low"),
        (0.30, "nominal"),
        (0.30, "high"),
    ]
    assert all(item[2] is sentinel_factory for item in calls)
    assert sweep["schema"] == "pip-sim-turn-sweep/v1"
    assert sweep["search_candidate_count"] == 125
    assert sweep["evaluated_candidate_count"] == 2
    assert sweep["surface_run_count"] == 6
    assert sweep["model_sha256"] == "model-hash"
    assert sweep["turn_config_sha256"] == "turn-hash"
    assert sweep["simulation_config_sha256"] == "simulation-hash"
    assert sweep["pybullet_version"] == "3.2.7"


def test_ranking_uses_worst_score_then_safety_translation_orientation_and_parameters() -> None:
    def fake_runner(
        parameters: TurnParameters,
        surface: Surface,
        settings: SimulationSettings,
        client_factory: object,
    ) -> SimulationResult:
        del settings, client_factory
        hold = parameters.hold_s
        if hold == 0.25:
            return measured(parameters, surface, yaw=100.0, fell=surface.name == "high")
        if hold == 0.30:
            return measured(parameters, surface, yaw=9.0, translation=0.020)
        if hold == 0.35:
            return measured(parameters, surface, yaw=8.0, translation=0.010)
        if hold == 0.40:
            return measured(parameters, surface, yaw=7.0, translation=0.005, roll=2.0, pitch=2.0)
        return measured(parameters, surface, yaw=6.6, translation=0.005, roll=1.0, pitch=1.0)

    ranked = run_sweep(BASELINE, SETTINGS, runner=fake_runner, client_factory=object(), limit=5)[
        "ranked_candidates"
    ]

    assert [item["parameters"]["hold_s"] for item in ranked] == [0.35, 0.30, 0.45, 0.40, 0.25]
    assert ranked[-1]["score"] is None
    assert ranked[-1]["fell"] is True

    def equal_runner(
        parameters: TurnParameters,
        surface: Surface,
        settings: SimulationSettings,
        client_factory: object,
    ) -> SimulationResult:
        del settings, client_factory
        return measured(parameters, surface, yaw=8.0, translation=0.001)

    equal_ranked = run_sweep(
        BASELINE,
        SETTINGS,
        runner=equal_runner,
        client_factory=object(),
        limit=3,
    )["ranked_candidates"]
    assert [item["parameters"]["hold_s"] for item in equal_ranked] == [0.25, 0.30, 0.35]


def test_ranked_json_is_valid_byte_stable_and_contains_no_nonfinite_numbers_or_raw_steps(tmp_path: Path) -> None:
    def falling_runner(
        parameters: TurnParameters,
        surface: Surface,
        settings: SimulationSettings,
        client_factory: object,
    ) -> SimulationResult:
        del settings, client_factory
        return measured(parameters, surface, fell=parameters.hold_s == 0.25)

    first_sweep = run_sweep(BASELINE, SETTINGS, runner=falling_runner, client_factory=object(), limit=2)
    second_sweep = run_sweep(BASELINE, SETTINGS, runner=falling_runner, client_factory=object(), limit=2)
    first = tmp_path / "nested" / "first.json"
    second = tmp_path / "other" / "second.json"

    write_ranked_json(first_sweep, first)
    write_ranked_json(second_sweep, second)

    assert first.read_bytes() == second.read_bytes()
    raw = first.read_text(encoding="utf-8")
    parsed = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    assert parsed == first_sweep == second_sweep
    assert raw.endswith("\n")
    assert "timestamp" not in raw.lower()
    assert "raw_steps" not in raw
    assert "Infinity" not in raw
    assert first_sweep["safe_candidate_count"] == 1
    assert first_sweep["fall_candidate_count"] == 1
    assert math.isfinite(first_sweep["ranked_candidates"][0]["score"])
