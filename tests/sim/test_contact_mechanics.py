import math

import pytest

from sim.contact_mechanics import (
    aggregate_foot_mechanics,
    reconstruct_contact_force,
    yaw_moment_z_nm,
)
from sim.kinematics import Leg


def contact(
    *,
    link=2,
    point=(1.0, 0.0, 0.0),
    normal=(0.0, 0.0, 1.0),
    normal_force=10.0,
    friction_1=2.0,
    direction_1=(0.0, 1.0, 0.0),
    friction_2=-3.0,
    direction_2=(1.0, 0.0, 0.0),
):
    value = [None] * 14
    value[3] = link
    value[5] = point
    value[7] = normal
    value[9] = normal_force
    value[10] = friction_1
    value[11] = direction_1
    value[12] = friction_2
    value[13] = direction_2
    return tuple(value)


def test_reconstructs_normal_and_two_lateral_force_components():
    force = reconstruct_contact_force(contact())
    assert force.normal_xyz_n == (0.0, 0.0, 10.0)
    assert force.tangential_xyz_n == (-3.0, 2.0, 0.0)
    assert force.total_xyz_n == (-3.0, 2.0, 10.0)


def test_missing_optional_lateral_fields_preserves_normal_but_not_fake_tangent():
    point = contact()[:10]
    force = reconstruct_contact_force(point)
    assert force.normal_xyz_n == (0.0, 0.0, 10.0)
    assert force.tangential_xyz_n is None
    assert force.total_xyz_n is None


def test_native_and_right_positive_yaw_moment_sign():
    native, right = yaw_moment_z_nm((0.0, 0.0), (1.0, 0.0), (0.0, -2.0))
    assert native == -2.0
    assert right == 2.0


def test_aggregates_multiple_points_per_foot_and_separates_torso():
    points = (
        contact(link=2, point=(1.0, 0.0, 0.0), friction_1=-1.0, friction_2=0.0),
        contact(link=2, point=(1.0, 0.0, 0.0), normal_force=4.0, friction_1=-2.0, friction_2=0.0),
        contact(link=-1, normal_force=5.0, friction_1=0.0, friction_2=0.0),
    )
    mechanics, torso_contact = aggregate_foot_mechanics(
        points, {Leg.FL: 2, Leg.FR: 3, Leg.RL: 4, Leg.RR: 5}, (0.0, 0.0, 0.2)
    )
    fl = next(item for item in mechanics if item.leg == "FL")
    assert fl.in_contact is True
    assert fl.normal_force_n == 14.0
    assert fl.tangential_force_xyz_n == (0.0, -3.0, 0.0)
    assert fl.bullet_tau_z_nm == -3.0
    assert fl.right_yaw_torque_nm == 3.0
    assert torso_contact is True
    assert sum(item.in_contact for item in mechanics) == 1


def test_rejects_nonfinite_consumed_contact_values():
    with pytest.raises(ValueError, match="finite"):
        reconstruct_contact_force(contact(normal_force=math.nan))
