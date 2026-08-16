from __future__ import annotations

from dataclasses import is_dataclass
import math

import pytest

from pip_robot.turn.models import HeadingObservation, TurnParameters
from sim.model import (
    FinalPose,
    FootContact,
    SimulationResult,
    detect_fall,
    heading_from_quaternion,
    yaw_delta_from_quaternions,
    yaw_from_quaternion,
)


PARAMETERS = TurnParameters(
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
FINAL_POSE = FinalPose(
    position_xyz=(0.02, -0.01, 0.10),
    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    roll_deg=1.0,
    pitch_deg=2.0,
    yaw_deg=3.0,
)
CONTACTS = (
    FootContact("FL", True, 2.0),
    FootContact("FR", True, 2.0),
    FootContact("RL", True, 2.0),
    FootContact("RR", True, 2.0),
)


def result(**changes: object) -> SimulationResult:
    values: dict[str, object] = {
        "parameters": PARAMETERS,
        "surface_name": "nominal",
        "friction": 0.70,
        "yaw_delta_deg": 12.0,
        "translation_x_m": 0.03,
        "translation_y_m": 0.04,
        "translation_m": 0.05,
        "max_roll_deviation_deg": 2.0,
        "max_pitch_deviation_deg": 3.0,
        "fell": False,
        "contact_instability": 0.25,
        "elapsed_sim_s": 1.0,
        "final_pose": FINAL_POSE,
        "foot_contacts": CONTACTS,
    }
    values.update(changes)
    return SimulationResult(**values)  # type: ignore[arg-type]


def test_simulation_boundary_models_are_frozen_dataclasses() -> None:
    for model in (FootContact, FinalPose, SimulationResult):
        assert is_dataclass(model)
        assert model.__dataclass_params__.frozen is True


def test_result_preserves_measured_fields_and_shared_parameters_identity() -> None:
    measured = result()

    assert measured.parameters is PARAMETERS
    assert measured.translation_m == pytest.approx(0.05)
    assert measured.foot_contacts == CONTACTS
    assert measured.aborted is False
    assert measured.invalid_reason is None


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("friction", math.nan),
        ("yaw_delta_deg", math.inf),
        ("translation_x_m", -math.inf),
        ("max_roll_deviation_deg", -0.1),
        ("max_pitch_deviation_deg", math.nan),
        ("contact_instability", 1.01),
        ("elapsed_sim_s", -0.01),
    ],
)
def test_result_rejects_nonfinite_or_inconsistent_measured_metrics(field: str, value: object) -> None:
    with pytest.raises(ValueError, match=field):
        result(**{field: value})


def test_result_rejects_translation_that_disagrees_with_measured_xy_vector() -> None:
    with pytest.raises(ValueError, match="translation_m"):
        result(translation_m=0.049)


@pytest.mark.parametrize(
    ("surface_name", "friction"),
    [
        ("low", 0.70),
        ("nominal", 0.45),
        ("unknown", 0.70),
    ],
)
def test_result_rejects_unknown_or_mismatched_canonical_surface_friction_pair(
    surface_name: str, friction: float
) -> None:
    with pytest.raises(ValueError, match="surface"):
        result(surface_name=surface_name, friction=friction)


@pytest.mark.parametrize(
    ("position", "quaternion", "angle"),
    [
        ((math.nan, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), 0.0),
        ((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 0.0), 0.0),
        ((0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0), math.inf),
    ],
)
def test_final_pose_rejects_nonfinite_position_quaternion_or_attitude(
    position: tuple[float, float, float], quaternion: tuple[float, float, float, float], angle: float
) -> None:
    with pytest.raises(ValueError):
        FinalPose(position, quaternion, angle, 0.0, 0.0)


@pytest.mark.parametrize("force", [math.nan, -0.01])
def test_foot_contact_rejects_invalid_normal_force(force: float) -> None:
    with pytest.raises(ValueError, match="normal_force_n"):
        FootContact("FL", True, force)


def test_yaw_from_quaternion_extracts_a_positive_right_turn_heading() -> None:
    half_angle = math.radians(45.0)

    assert yaw_from_quaternion((0.0, 0.0, math.sin(half_angle), math.cos(half_angle))) == pytest.approx(90.0)


def test_quaternion_heading_delta_uses_the_existing_sourced_wrap_contract() -> None:
    before = (0.0, 0.0, math.sin(math.radians(179.0 / 2.0)), math.cos(math.radians(179.0 / 2.0)))
    after = (0.0, 0.0, math.sin(math.radians(-179.0 / 2.0)), math.cos(math.radians(-179.0 / 2.0)))

    observed = heading_from_quaternion(after, source="pybullet-base")

    assert observed == HeadingObservation(heading_deg=-179.0, source="pybullet-base")
    assert yaw_delta_from_quaternions(before, after, source="pybullet-base") == pytest.approx(2.0)


@pytest.mark.parametrize(
    "measurements",
    [
        {"corrected_roll_deg": 10.01},
        {"pitch_deviation_deg": -12.01},
        {"height_m": 0.0749},
        {"torso_contact": True},
        {"supported_feet": 1, "support_loss_duration_s": 0.10},
    ],
)
def test_detect_fall_trips_each_documented_safety_boundary(measurements: dict[str, object]) -> None:
    baseline: dict[str, object] = {
        "corrected_roll_deg": 10.0,
        "pitch_deviation_deg": 12.0,
        "height_m": 0.075,
        "torso_contact": False,
        "supported_feet": 2,
        "support_loss_duration_s": 0.10,
    }
    baseline.update(measurements)

    assert detect_fall(**baseline) is True  # type: ignore[arg-type]


def test_detect_fall_keeps_exact_attitude_and_height_thresholds_safe_with_brief_support_loss() -> None:
    assert (
        detect_fall(
            corrected_roll_deg=-10.0,
            pitch_deviation_deg=12.0,
            height_m=0.075,
            torso_contact=False,
            supported_feet=1,
            support_loss_duration_s=0.099,
        )
        is False
    )


def test_detect_fall_uses_caller_supplied_safety_thresholds() -> None:
    assert detect_fall(
        corrected_roll_deg=6.0,
        pitch_deviation_deg=7.0,
        height_m=0.09,
        torso_contact=False,
        supported_feet=4,
        support_loss_duration_s=0.0,
        fall_roll_deg=5.0,
        fall_pitch_deg=8.0,
        min_height_m=0.08,
        max_support_loss_duration_s=0.2,
    ) is True
