"""Independent finite differences and closed-loop checks of leg derivatives."""

import math
from pathlib import Path

import pytest
from duk_core.modeling import (
    KinematicSingularityError,
    UnreachableConfigurationError,
    inverse_wheel_position,
    leg_jacobian,
    leg_second_derivative,
    load_leg_geometry_json,
    point_acceleration,
    solve_leg_pose,
)

GEOMETRY = Path(__file__).resolve().parents[3] / "config/physical/duk_leg_geometry.json"


@pytest.fixture
def geometry():
    return load_leg_geometry_json(GEOMETRY)


@pytest.mark.parametrize("q_deg", [75, 85, 95, 115])
def test_second_derivatives_match_jacobian_differences(geometry, q_deg):
    q = math.radians(q_deg)
    epsilon = 1e-5
    pose = solve_leg_pose(geometry, q)
    second = leg_second_derivative(geometry, pose)
    left = leg_jacobian(geometry, solve_leg_pose(geometry, q - epsilon))
    right = leg_jacobian(geometry, solve_leg_pose(geometry, q + epsilon))
    for point in ("B", "C", "H", "O"):
        reference = tuple(
            (b - a) / (2 * epsilon)
            for a, b in zip(left.point(point), right.point(point))
        )
        assert second.point(point) == pytest.approx(reference, abs=2e-5, rel=2e-7)


def test_time_acceleration_includes_velocity_squared_term(geometry):
    q, rate, acceleration, dt = math.radians(75), 0.4, -0.3, 1e-4
    pose = solve_leg_pose(geometry, q)
    previous = solve_leg_pose(geometry, q - rate * dt + 0.5 * acceleration * dt**2)
    following = solve_leg_pose(geometry, q + rate * dt + 0.5 * acceleration * dt**2)
    for point in ("B", "C", "H", "O"):
        reference = tuple(
            (a - 2 * b + c) / dt**2
            for a, b, c in zip(
                previous.point(point), pose.point(point), following.point(point)
            )
        )
        assert point_acceleration(
            geometry, pose, point, rate, acceleration
        ) == pytest.approx(reference, abs=1e-4)


def test_inverse_round_trip_over_physical_workspace(geometry):
    lower, upper = geometry.geometric_linkage_angle_limits_rad()
    for index in range(21):
        q = lower + 0.1 + index / 20 * (upper - lower - 0.2)
        pose = solve_leg_pose(geometry, q)
        solutions = inverse_wheel_position(geometry, pose.o)
        assert any(abs(solution.linkage_angle_rad - q) < 1e-8 for solution in solutions)
        for solution in solutions:
            assert solution.o == pytest.approx(pose.o, abs=1e-6)


def test_inverse_rejects_off_curve_target_and_nonfinite_input(geometry):
    for target in ((1000, 1000), (0, 0), (10, 10)):
        with pytest.raises(UnreachableConfigurationError):
            inverse_wheel_position(geometry, target)
    with pytest.raises(ValueError):
        inverse_wheel_position(geometry, (math.nan, 0))
    with pytest.raises(ValueError):
        point_acceleration(geometry, solve_leg_pose(geometry, 1.3), "O", math.inf, 0)


def test_second_derivative_rejects_loop_singularity(geometry):
    lower, _ = geometry.geometric_linkage_angle_limits_rad()
    with pytest.raises(KinematicSingularityError):
        leg_second_derivative(geometry, solve_leg_pose(geometry, lower))
