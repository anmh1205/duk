"""Numerical and invariant tests for canonical DUK leg kinematics."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from duk_core.modeling import (
    KinematicSingularityError,
    UnreachableConfigurationError,
    leg_jacobian,
    load_leg_geometry_json,
    sample_leg_trajectory,
    solve_leg_pose,
)


CONFIG_PATH = (
    Path(__file__).resolve().parents[3]
    / "config"
    / "physical"
    / "duk_leg_geometry.json"
)


def geometry():
    return load_leg_geometry_json(CONFIG_PATH)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def test_config_uses_canonical_robot_frame_and_named_topology() -> None:
    data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    assert data["schema_version"] == 4
    assert data["robot_frame"] == {
        "origin": "A",
        "x_axis": "+Ox points right",
        "y_axis": "+Oy points up",
        "positive_rotation": "counter_clockwise",
        "reference_vector": "DA",
        "da_angle_deg": -30,
    }
    assert data["four_bar_assembly"] == {
        "b_side_of_directed_ad": "right",
        "c_side_of_directed_bd": "right",
    }
    assert data["bco_rigid_body"] == {
        "BC_mm": 42,
        "BH_mm": 119,
        "OH_mm": 55,
        "o_side_of_directed_bc": "left",
    }
    assert "assembly_branch" not in data


def test_only_a_is_fixed_and_other_points_are_derived() -> None:
    g = geometry()
    alpha = math.radians(75.0)
    pose = solve_leg_pose(g, alpha)

    assert pose.a == (0.0, 0.0)
    assert pose.d == pytest.approx(
        (
            g.ad_mm * math.cos(g.ad_angle_rad),
            g.ad_mm * math.sin(g.ad_angle_rad),
        )
    )
    assert pose.b == pytest.approx(
        (
            g.ab_mm * math.cos(alpha),
            g.ab_mm * math.sin(alpha),
        )
    )

    assert pose.d != (-70.14805770653953, 40.5)
    assert pose.b != (33.905294908430214, 126.53628324386794)


def test_robot_frame_directed_da_ad_relation() -> None:
    g = geometry()
    assert math.degrees(g.da_angle_rad) == pytest.approx(-30.0)
    assert math.degrees(g.ad_angle_rad) == pytest.approx(150.0)
    assert g.d_point[1] > 0.0


def test_derived_wheel_carrier_distances() -> None:
    g = geometry()
    assert g.bo_mm == pytest.approx(math.hypot(119.0, 55.0))
    assert g.bo_mm == pytest.approx(131.0953851)
    assert g.co_mm == pytest.approx(math.hypot(161.0, 55.0))
    assert g.co_mm == pytest.approx(170.1352403)


def test_reference_pose_at_75_degrees_is_direct_robot_frame() -> None:
    pose = solve_leg_pose(
        geometry(),
        math.radians(75.0),
    )

    assert pose.a == (0.0, 0.0)
    assert pose.d == pytest.approx(
        (-70.1480577, 40.5),
        abs=1e-6,
    )
    assert pose.b == pytest.approx(
        (33.9052949, 126.5362832),
        abs=1e-6,
    )
    assert pose.c == pytest.approx(
        (2.4221051, 154.3357214),
        abs=1e-6,
    )
    assert pose.h == pytest.approx(
        (123.1076659, 47.7712085),
        abs=1e-6,
    )
    assert pose.o == pytest.approx(
        (86.7036398, 6.5432219),
        abs=1e-6,
    )


def test_named_topology_relations_hold() -> None:
    g = geometry()
    pose = solve_leg_pose(g, math.radians(75.0))

    ad = sub(pose.d, pose.a)
    ab = sub(pose.b, pose.a)
    bd = sub(pose.d, pose.b)
    bc = sub(pose.c, pose.b)
    bo = sub(pose.o, pose.b)

    assert cross(ad, ab) < 0.0
    assert g.b_side_of_ad == "right"

    assert cross(bd, bc) < 0.0
    assert g.c_side_of_bd == "right"

    assert cross(bc, bo) > 0.0
    assert g.bco_body.o_side_of_bc == "left"


def test_loop_closure_and_rigid_bco_geometry_hold() -> None:
    g = geometry()
    pose = solve_leg_pose(g, math.radians(75.0))

    assert distance(pose.a, pose.b) == pytest.approx(g.ab_mm)
    assert distance(pose.b, pose.c) == pytest.approx(g.bc_mm)
    assert distance(pose.c, pose.d) == pytest.approx(g.cd_mm)
    assert distance(pose.d, pose.a) == pytest.approx(g.ad_mm)
    assert distance(pose.b, pose.h) == pytest.approx(g.bh_mm)
    assert distance(pose.h, pose.o) == pytest.approx(g.oh_mm)
    assert distance(pose.b, pose.o) == pytest.approx(g.bo_mm)
    assert distance(pose.c, pose.o) == pytest.approx(g.co_mm)


def test_bco_rigid_triangle_invariants_hold_over_motion() -> None:
    g = geometry()
    lower, upper = g.geometric_ab_angle_limits_rad()
    margin = math.radians(2.0)
    poses = sample_leg_trajectory(
        g,
        lower + margin,
        upper - margin,
        41,
    )

    expected_area2 = g.bco_body.signed_double_area_mm2
    for pose in poses:
        assert distance(pose.b, pose.c) == pytest.approx(
            g.bc_mm,
            abs=1e-8,
        )
        assert distance(pose.b, pose.o) == pytest.approx(
            g.bo_mm,
            abs=1e-8,
        )
        assert distance(pose.c, pose.o) == pytest.approx(
            g.co_mm,
            abs=1e-8,
        )
        assert cross(
            sub(pose.c, pose.b),
            sub(pose.o, pose.b),
        ) == pytest.approx(
            expected_area2,
            abs=1e-7,
        )


def test_bco_velocity_constraints_hold() -> None:
    g = geometry()
    pose = solve_leg_pose(g, math.radians(75.0))
    jac = leg_jacobian(g, pose)

    assert dot(
        sub(pose.c, pose.b),
        sub(jac.d_c, jac.d_b),
    ) == pytest.approx(0.0, abs=1e-7)
    assert dot(
        sub(pose.o, pose.b),
        sub(jac.d_o, jac.d_b),
    ) == pytest.approx(0.0, abs=1e-7)
    assert dot(
        sub(pose.o, pose.c),
        sub(jac.d_o, jac.d_c),
    ) == pytest.approx(0.0, abs=1e-7)


def test_absolute_ab_workspace_limits() -> None:
    lower, upper = geometry().geometric_ab_angle_limits_rad()
    assert math.degrees(lower) == pytest.approx(
        38.9950392,
        abs=1e-6,
    )
    assert math.degrees(upper) == pytest.approx(
        105.2555454,
        abs=1e-6,
    )


def test_outside_physical_ab_workspace_is_rejected() -> None:
    g = geometry()
    with pytest.raises(UnreachableConfigurationError):
        solve_leg_pose(g, math.radians(20.0))


def test_wrong_side_of_ad_is_rejected_even_if_circle_geometry_exists() -> None:
    g = geometry()
    mirror_angle = (
        g.ad_angle_rad
        + math.radians(75.0)
    )
    with pytest.raises(UnreachableConfigurationError):
        solve_leg_pose(g, mirror_angle)


def test_analytic_wheel_jacobian_matches_central_difference() -> None:
    g = geometry()
    alpha = math.radians(75.0)
    pose = solve_leg_pose(g, alpha)
    analytic = leg_jacobian(g, pose).d_o

    step = 1e-6
    plus = solve_leg_pose(g, alpha + step).o
    minus = solve_leg_pose(g, alpha - step).o
    numeric = (
        (plus[0] - minus[0]) / (2 * step),
        (plus[1] - minus[1]) / (2 * step),
    )
    assert analytic == pytest.approx(
        numeric,
        rel=1e-7,
        abs=1e-7,
    )


def test_singular_boundary_is_detected_by_jacobian() -> None:
    g = geometry()
    lower, _ = g.geometric_ab_angle_limits_rad()
    pose = solve_leg_pose(
        g,
        lower,
        tolerance_mm=1e-7,
    )
    with pytest.raises(KinematicSingularityError):
        leg_jacobian(
            g,
            pose,
            singularity_tolerance=1e-7,
        )


def test_trajectory_sampling_uses_absolute_robot_angle() -> None:
    g = geometry()
    poses = sample_leg_trajectory(
        g,
        math.radians(55.0),
        math.radians(100.0),
        11,
    )
    assert len(poses) == 11
    assert poses[0].ab_angle_rad == pytest.approx(
        math.radians(55.0)
    )
    assert poses[-1].ab_angle_rad == pytest.approx(
        math.radians(100.0)
    )
