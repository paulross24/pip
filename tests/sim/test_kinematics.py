from __future__ import annotations

import math

import pytest

from sim.kinematics import (
    Leg,
    factory_stance_mm,
    forward_kinematics_mm,
    inverse_kinematics_deg,
    leg_angles_deg,
    stance_joint_targets_rad,
)


def test_factory_legs_are_in_canonical_order_with_the_calibrated_foot_coordinates() -> None:
    assert tuple(Leg) == (Leg.FL, Leg.FR, Leg.RL, Leg.RR)
    assert factory_stance_mm == {
        Leg.FL: (-15.0, 95.0),
        Leg.FR: (-15.0, 95.0),
        Leg.RL: (5.0, 90.0),
        Leg.RR: (5.0, 90.0),
    }


def test_inverse_kinematics_matches_the_hand_checked_factory_angles_and_negative_knee_bend() -> None:
    assert leg_angles_deg[Leg.FL] == pytest.approx((45.692012, -78.737020), abs=1e-6)
    assert leg_angles_deg[Leg.FR] == pytest.approx((45.692012, -78.737020), abs=1e-6)
    assert leg_angles_deg[Leg.RL] == pytest.approx((65.722071, -88.880871), abs=1e-6)
    assert leg_angles_deg[Leg.RR] == pytest.approx((65.722071, -88.880871), abs=1e-6)
    assert inverse_kinematics_deg(-15.0, 95.0) == pytest.approx((45.692012, -78.737020), abs=1e-6)
    assert inverse_kinematics_deg(5.0, 90.0) == pytest.approx((65.722071, -88.880871), abs=1e-6)


def test_factory_angles_place_each_foot_at_its_requested_coordinate_using_40_and_80_mm_links() -> None:
    for leg in Leg:
        hip_deg, knee_deg = leg_angles_deg[leg]
        assert forward_kinematics_mm(hip_deg, knee_deg) == pytest.approx(factory_stance_mm[leg], abs=1e-9)


def test_stance_targets_are_ordered_by_leg_and_use_all_eight_urdf_joint_names() -> None:
    assert tuple(stance_joint_targets_rad) == (
        "P2_FL_HIP",
        "P3_FL_KNEE",
        "P7_FR_HIP",
        "P8_FR_KNEE",
        "P0_RL_HIP",
        "P1_RL_KNEE",
        "P10_RR_HIP",
        "P11_RR_KNEE",
    )
    assert stance_joint_targets_rad == pytest.approx(
        {
            "P2_FL_HIP": math.radians(45.692012),
            "P3_FL_KNEE": math.radians(-78.737020),
            "P7_FR_HIP": math.radians(45.692012),
            "P8_FR_KNEE": math.radians(-78.737020),
            "P0_RL_HIP": math.radians(65.722071),
            "P1_RL_KNEE": math.radians(-88.880871),
            "P10_RR_HIP": math.radians(65.722071),
            "P11_RR_KNEE": math.radians(-88.880871),
        },
        abs=1e-7,
    )


@pytest.mark.parametrize("endpoint", [(0.0, 39.999), (0.0, 120.001), (0.0, 0.0)])
def test_inverse_kinematics_rejects_endpoints_outside_the_two_link_workspace(endpoint: tuple[float, float]) -> None:
    with pytest.raises(ValueError, match="unreachable"):
        inverse_kinematics_deg(*endpoint)
