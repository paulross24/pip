import json
import math

import pytest

from sim.diagnostics import (
    DiagnosticTrace,
    FootTraceSample,
    PhaseTraceSample,
    summarize_trace,
    trace_json_bytes,
    write_trace_atomic,
)


def foot(leg, *, contact=True, torque=1.0, slip=0.001):
    return FootTraceSample(
        leg=leg,
        in_contact=contact,
        normal_force_n=4.0 if contact else 0.0,
        tangential_force_xyz_n=(0.0, -1.0, 0.0) if contact else None,
        world_position_xyz_m=(0.1, 0.1, 0.0),
        displacement_xyz_m=(0.001, 0.0, 0.0),
        cumulative_slip_m=slip,
        bullet_tau_z_nm=-torque if contact else None,
        right_yaw_torque_nm=torque if contact else None,
    )


def sample(time_s, yaw, phase="DRIVE_TURN", torque=1.0):
    feet = tuple(foot(leg, torque=torque if leg == "FL" else 0.25) for leg in ("FL", "FR", "RL", "RR"))
    return PhaseTraceSample(
        phase=phase,
        elapsed_s=time_s,
        body_yaw_deg=yaw,
        body_roll_deg=1.0 + time_s,
        body_pitch_deg=-0.5,
        body_position_xyz_m=(0.0, 0.0, 0.1),
        angular_velocity_xyz_rad_s=(0.0, 0.0, 0.1),
        linear_velocity_xyz_m_s=(0.0, 0.0, 0.0),
        feet=feet,
        support_foot_count=4,
        total_contact_normal_force_n=16.0,
        total_bullet_tau_z_nm=-(torque + 0.75),
        total_right_yaw_torque_nm=torque + 0.75,
        torso_contact=False,
    )


def trace():
    return DiagnosticTrace(
        schema_version=1,
        run_id="test-run",
        primitive_family="diagonal_unload",
        candidate_id="baseline",
        surface_name="nominal",
        samples=(sample(0.1, 359.0), sample(0.2, 1.0)),
    )


def test_trace_serialization_is_deterministic_and_has_required_fields():
    first = trace_json_bytes(trace())
    second = trace_json_bytes(trace())
    assert first == second
    document = json.loads(first)
    assert document["run_id"] == "test-run"
    assert document["samples"][0]["feet"][0]["leg"] == "FL"
    assert document["samples"][0]["angular_velocity_xyz_rad_s"] == [0.0, 0.0, 0.1]


def test_trace_rejects_nonfinite_before_touching_destination(tmp_path):
    bad = trace()
    object.__setattr__(bad.samples[0], "body_roll_deg", math.nan)
    destination = tmp_path / "trace.json"
    destination.write_bytes(b"preserve")
    with pytest.raises(ValueError, match="finite"):
        write_trace_atomic(destination, bad)
    assert destination.read_bytes() == b"preserve"


def test_summary_handles_heading_wrap_and_reports_dominant_foot():
    summary = summarize_trace(trace())
    assert len(summary) == 1
    phase = summary[0]
    assert phase.phase == "DRIVE_TURN"
    assert phase.yaw_delta_deg == pytest.approx(2.0)
    assert phase.dominant_yaw_torque_foot == "FL"
    assert phase.active_contacts == ("FL", "FR", "RL", "RR")
    assert phase.peak_positive_right_yaw_torque_nm == pytest.approx(1.75)
    assert phase.total_slip_m == pytest.approx(0.008)


def test_trace_requires_canonical_foot_order():
    bad_sample = sample(0.1, 0.0)
    object.__setattr__(bad_sample, "feet", tuple(reversed(bad_sample.feet)))
    with pytest.raises(ValueError, match="canonical"):
        DiagnosticTrace(1, "run", "family", "candidate", "low", (bad_sample,))
