"""Numerical and invariant tests for the established DUK leg geometry."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from duk_core.modeling import (
    KinematicSingularityError,
    UnreachableConfigurationError,
    jacobian_in_sketch_frame,
    leg_jacobian,
    load_leg_geometry_json,
    pose_in_sketch_frame,
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
    return load_leg_geometry_json(
        CONFIG_PATH
    )


def distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def test_config_keeps_assembly_branch_explicitly_unresolved() -> None:
    data = json.loads(
        CONFIG_PATH.read_text(
            encoding="utf-8"
        )
    )
    assert data["assembly_branch"] is None


def test_derived_wheel_center_distances() -> None:
    g = geometry()
    assert g.bo_mm == pytest.approx(
        math.hypot(119.0, 55.0)
    )
    assert g.bo_mm == pytest.approx(
        131.0953851
    )
    assert g.co_mm == pytest.approx(
        math.hypot(161.0, 55.0)
    )
    assert g.co_mm == pytest.approx(
        170.1352403
    )


@pytest.mark.parametrize(
    "branch",
    (-1, 1),
)
def test_loop_closure_and_wheel_offset_hold_for_both_branches(
    branch: int,
) -> None:
    g = geometry()
    pose = solve_leg_pose(
        g,
        math.radians(75.0),
        assembly_branch=branch,
    )
    assert distance(
        pose.a,
        pose.b,
    ) == pytest.approx(g.ab_mm)
    assert distance(
        pose.b,
        pose.c,
    ) == pytest.approx(g.bc_mm)
    assert distance(
        pose.c,
        pose.d,
    ) == pytest.approx(g.cd_mm)
    assert distance(
        pose.d,
        pose.a,
    ) == pytest.approx(g.ad_mm)
    assert distance(
        pose.b,
        pose.h,
    ) == pytest.approx(g.bh_mm)
    assert distance(
        pose.h,
        pose.o,
    ) == pytest.approx(g.oh_mm)
    assert distance(
        pose.b,
        pose.o,
    ) == pytest.approx(g.bo_mm)
    assert distance(
        pose.c,
        pose.o,
    ) == pytest.approx(g.co_mm)

    hb = sub(
        pose.b,
        pose.h,
    )
    ho = sub(
        pose.o,
        pose.h,
    )
    assert dot(
        hb,
        ho,
    ) == pytest.approx(
        0.0,
        abs=1e-8,
    )
    assert dot(
        sub(pose.h, pose.b),
        pose.e_bc,
    ) == pytest.approx(
        -g.bh_mm
    )
    assert dot(
        sub(pose.o, pose.h),
        pose.left_normal_bc,
    ) == pytest.approx(
        g.wheel_side * g.oh_mm
    )


def test_reference_branch_pose_at_75_degrees() -> None:
    g = geometry()
    pose = solve_leg_pose(
        g,
        math.radians(75.0),
        assembly_branch=1,
    )
    assert pose.b == pytest.approx(
        (33.9052949, 126.5362832),
        abs=1e-6,
    )
    assert pose.c == pytest.approx(
        (75.0702561, 134.8697080),
        abs=1e-6,
    )
    assert pose.h == pytest.approx(
        (-82.7287619, 102.9249131),
        abs=1e-6,
    )
    assert pose.o == pytest.approx(
        (-71.8159437, 49.0184163),
        abs=1e-6,
    )


def test_geometric_workspace_limits() -> None:
    lower, upper = (
        geometry().geometric_angle_limits_rad()
    )
    assert (
        math.degrees(lower)
        == pytest.approx(
            44.7444546,
            abs=1e-6,
        )
    )
    assert (
        math.degrees(upper)
        == pytest.approx(
            111.0049608,
            abs=1e-6,
        )
    )


def test_outside_workspace_is_rejected() -> None:
    g = geometry()
    with pytest.raises(
        UnreachableConfigurationError
    ):
        solve_leg_pose(
            g,
            math.radians(20.0),
            assembly_branch=1,
        )


def test_analytic_wheel_jacobian_matches_central_difference() -> None:
    g = geometry()
    alpha = math.radians(75.0)
    pose = solve_leg_pose(
        g,
        alpha,
        assembly_branch=1,
    )
    analytic = leg_jacobian(
        g,
        pose,
    ).d_o
    step = 1e-6
    plus = solve_leg_pose(
        g,
        alpha + step,
        assembly_branch=1,
    ).o
    minus = solve_leg_pose(
        g,
        alpha - step,
        assembly_branch=1,
    ).o
    numeric = (
        (plus[0] - minus[0])
        / (2 * step),
        (plus[1] - minus[1])
        / (2 * step),
    )
    assert analytic == pytest.approx(
        numeric,
        rel=1e-7,
        abs=1e-7,
    )


def test_singular_boundary_is_detected_by_jacobian() -> None:
    g = geometry()
    lower, _ = (
        g.geometric_angle_limits_rad()
    )
    pose = solve_leg_pose(
        g,
        lower,
        assembly_branch=1,
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


def test_trajectory_sampling_and_sketch_rotation() -> None:
    g = geometry()
    poses = sample_leg_trajectory(
        g,
        math.radians(55.0),
        math.radians(100.0),
        11,
        assembly_branch=1,
    )
    assert len(poses) == 11
    assert (
        poses[0].input_angle_rad
        == pytest.approx(
            math.radians(55.0)
        )
    )
    assert (
        poses[-1].input_angle_rad
        == pytest.approx(
            math.radians(100.0)
        )
    )

    sketch = pose_in_sketch_frame(
        g,
        poses[3],
    )
    assert distance(
        sketch.a,
        sketch.d,
    ) == pytest.approx(g.ad_mm)
    expected_d = (
        g.ad_mm
        * math.cos(
            g.ad_orientation_rad
        ),
        g.ad_mm
        * math.sin(
            g.ad_orientation_rad
        ),
    )
    assert sketch.d == pytest.approx(
        expected_d
    )
    ad_vector = sub(sketch.d, sketch.a)
    ad_angle = math.atan2(ad_vector[1], ad_vector[0])
    assert math.degrees(ad_angle) == pytest.approx(30.0)


def test_sketch_frame_jacobian_matches_transformed_numeric_derivative() -> None:
    g = geometry()
    alpha = math.radians(75.0)
    local_pose = solve_leg_pose(
        g,
        alpha,
        assembly_branch=1,
    )
    local_jacobian = leg_jacobian(
        g,
        local_pose,
    )
    robot_jacobian = jacobian_in_sketch_frame(
        g,
        local_jacobian,
    )

    step = 1e-6
    plus = pose_in_sketch_frame(
        g,
        solve_leg_pose(
            g,
            alpha + step,
            assembly_branch=1,
        ),
    ).o
    minus = pose_in_sketch_frame(
        g,
        solve_leg_pose(
            g,
            alpha - step,
            assembly_branch=1,
        ),
    ).o
    numeric = (
        (plus[0] - minus[0]) / (2 * step),
        (plus[1] - minus[1]) / (2 * step),
    )
    assert robot_jacobian.d_o == pytest.approx(
        numeric,
        rel=1e-7,
        abs=1e-7,
    )
