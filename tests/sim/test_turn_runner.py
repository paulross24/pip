import inspect

from sim.pivot_runner import direct_client_factory, load_simulation_settings, load_turn_parameters
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
