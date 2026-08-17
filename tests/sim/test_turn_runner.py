import inspect
from dataclasses import replace

import pytest

from sim.pivot_runner import direct_client_factory, load_simulation_settings, load_turn_parameters, _run_candidate_legacy
from sim.surfaces import required_surfaces
from sim.turn_primitives.diagonal_unload import DiagonalUnloadPrimitive
from sim.turn_runner import run_primitive


def test_real_instrumented_control_run_captures_required_mechanics():
    run = run_primitive(
        DiagonalUnloadPrimitive(),
        load_turn_parameters("config/turn_right_baseline.json"),
        required_surfaces()[1],
        load_simulation_settings("config/simulation.json"),
        direct_client_factory,
        run_id="instrumented-test",
    )
    assert run.result.invalid_reason is None
    assert run.trace is not None
    assert {sample.phase for sample in run.trace.samples} >= {"SHIFT_UNLOAD", "DRIVE_TURN", "REPLANT", "RECOVER"}
    sample = run.trace.samples[-1]
    assert len(sample.feet) == 4
    assert sample.support_foot_count == sum(foot.in_contact for foot in sample.feet)
    assert sample.total_contact_normal_force_n >= 0.0
    assert all(foot.world_position_xyz_m is not None for foot in sample.feet)


def test_executor_has_only_one_initial_spawn_base_reset_and_no_velocity_reset():
    source = inspect.getsource(run_primitive)
    assert source.count("resetBasePositionAndOrientation") == 1
    assert "resetBaseVelocity" not in source


def test_fall_detection_is_active_during_initial_stand_before_baseline():
    settings = replace(load_simulation_settings("config/simulation.json"), min_height_m=0.20)
    run = run_primitive(
        DiagonalUnloadPrimitive(), load_turn_parameters("config/turn_right_baseline.json"),
        required_surfaces()[1], settings, direct_client_factory, run_id="early-fall",
    )
    assert run.result.fell is True
    assert run.result.aborted is True
    assert run.result.invalid_reason == "fall-detected"
    assert run.trace is not None
    assert {sample.phase for sample in run.trace.samples} == {"STAND"}


def test_control_matches_legacy_motion_with_explicit_physical_right_sign_conversion():
    parameters = load_turn_parameters("config/turn_right_baseline.json")
    settings = load_simulation_settings("config/simulation.json")
    surface = required_surfaces()[1]
    legacy = _run_candidate_legacy(parameters, surface, settings, direct_client_factory)
    instrumented = run_primitive(
        DiagonalUnloadPrimitive(), parameters, surface, settings, direct_client_factory,
        run_id="parity-test",
    ).result
    assert instrumented.yaw_delta_deg == pytest.approx(-legacy.yaw_delta_deg, abs=1e-9)
    assert instrumented.translation_m == pytest.approx(legacy.translation_m, abs=1e-9)
