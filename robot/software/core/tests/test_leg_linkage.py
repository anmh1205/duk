"""Equation and invariant tests for canonical DUK leg kinematics."""

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
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def test_config_describes_geometry_not_solved_coordinates() -> None:
    data = json.loads(
        CONFIG_PATH.read_text(encoding="utf-8")
    )

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
        "b_side_of_directed_ad": "left",
        "c_side_of_directed_bd": "left",
    }
    assert data["bco_rigid_body"] == {
        "BC_mm": 42,
        "BH_mm": 119,
        "OH_mm": 55,
        "o_side_of_directed_bc": "left",
    }

    serialized = json.dumps(data)
    for point_name in ("B", "C", "D", "H", "O"):
        assert f'"{point_name}_x"' not in serialized
        assert f'"{point_name}_y"' not in serialized


def test_only_a_is_fixed_and_d_b_are_derived_by_equations() -> None:
    g = geometry()
    q = math.radians(75.0)
    pose = solve_leg_pose(g, q)

    assert pose.a == (0.0, 0.0)

    expected_d = (
        g.ad_mm * math.cos(g.ad_angle_rad),
        g.ad_mm * math.sin(g.ad_angle_rad),
    )
    assert pose.d == pytest.approx(expected_d)

    theta_ab = g.da_angle_rad - q
    expected_b = (
        g.ab_mm * math.cos(theta_ab),
        g.ab_mm * math.sin(theta_ab),
    )
    assert pose.b == pytest.approx(expected_b)


def test_linkage_coordinate_definition_is_consistent() -> None:
    g = geometry()
    q = math.radians(75.0)

    theta_ab = g.ab_angle_rad(q)
    directed_ab_to_da = (
        g.da_angle_rad - theta_ab
    )

    assert math.degrees(g.da_angle_rad) == pytest.approx(-30.0)
    assert math.degrees(g.ad_angle_rad) == pytest.approx(150.0)
    assert directed_ab_to_da == pytest.approx(q)


def test_named_physical_topology_relations_hold() -> None:
    g = geometry()
    pose = solve_leg_pose(
        g,
        math.radians(75.0),
    )

    ad = sub(pose.d, pose.a)
    ab = sub(pose.b, pose.a)
    bd = sub(pose.d, pose.b)
    bc = sub(pose.c, pose.b)
    bo = sub(pose.o, pose.b)

    assert cross(ad, ab) > 0.0
    assert g.b_side_of_ad == "left"

    assert cross(bd, bc) > 0.0
    assert g.c_side_of_bd == "left"

    assert cross(bc, bo) > 0.0
    assert g.bco_body.o_side_of_bc == "left"


def test_loop_closure_and_rigid_bco_geometry_hold() -> None:
    g = geometry()
    pose = solve_leg_pose(
        g,
        math.radians(75.0),
    )

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
    lower, upper = (
        g.geometric_linkage_angle_limits_rad()
    )
    margin = math.radians(2.0)
    poses = sample_leg_trajectory(
        g,
        lower + margin,
        upper - margin,
        41,
    )

    expected_area2 = (
        g.bco_body.signed_double_area_mm2
    )

    for pose in poses:
        assert distance(
            pose.b,
            pose.c,
        ) == pytest.approx(
            g.bc_mm,
            abs=1e-8,
        )
        assert distance(
            pose.b,
            pose.o,
        ) == pytest.approx(
            g.bo_mm,
            abs=1e-8,
        )
        assert distance(
            pose.c,
            pose.o,
        ) == pytest.approx(
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
    pose = solve_leg_pose(
        g,
        math.radians(75.0),
    )
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


def test_geometric_linkage_angle_limits() -> None:
    lower, upper = (
        geometry().geometric_linkage_angle_limits_rad()
    )

    assert math.degrees(lower) == pytest.approx(
        68.9950392,
        abs=1e-6,
    )
    assert math.degrees(upper) == pytest.approx(
        135.2555454,
        abs=1e-6,
    )


def test_outside_geometric_workspace_is_rejected() -> None:
    g = geometry()

    with pytest.raises(
        UnreachableConfigurationError
    ):
        solve_leg_pose(
            g,
            math.radians(50.0),
        )


def test_wrong_mirror_side_is_rejected() -> None:
    g = geometry()

    with pytest.raises(
        UnreachableConfigurationError
    ):
        solve_leg_pose(
            g,
            math.radians(-75.0),
        )


def test_analytic_wheel_jacobian_matches_central_difference() -> None:
    g = geometry()
    q = math.radians(75.0)
    pose = solve_leg_pose(g, q)
    analytic = leg_jacobian(
        g,
        pose,
    ).d_o

    step = 1e-6
    plus = solve_leg_pose(
        g,
        q + step,
    ).o
    minus = solve_leg_pose(
        g,
        q - step,
    ).o
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
    lower, _ = (
        g.geometric_linkage_angle_limits_rad()
    )
    pose = solve_leg_pose(
        g,
        lower,
        tolerance_mm=1e-7,
    )

    with pytest.raises(
        KinematicSingularityError
    ):
        leg_jacobian(
            g,
            pose,
            singularity_tolerance=1e-7,
        )


def test_trajectory_sampling_uses_linkage_coordinate_q() -> None:
    g = geometry()
    poses = sample_leg_trajectory(
        g,
        math.radians(75.0),
        math.radians(120.0),
        11,
    )

    assert len(poses) == 11
    assert poses[0].linkage_angle_rad == pytest.approx(
        math.radians(75.0)
    )
    assert poses[-1].linkage_angle_rad == pytest.approx(
        math.radians(120.0)
    )
