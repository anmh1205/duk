"""Bench profiles remain geometrically valid; no physical mass data required."""

import math
from pathlib import Path

import pytest
from duk_core.modeling import load_leg_geometry_json
from duk_core.modeling.kinematic_bench import KinematicBench

CONFIG = Path(__file__).resolve().parents[3] / "config/physical/duk_leg_geometry.json"


@pytest.fixture
def bench():
    return KinematicBench(load_leg_geometry_json(CONFIG))


def test_manual_angles_are_independent_and_invalid_update_is_atomic(bench):
    bench.set_angles(math.radians(78), math.radians(95))
    previous = bench.samples
    assert previous[0].pose.linkage_angle_rad != previous[1].pose.linkage_angle_rad
    with pytest.raises(ValueError):
        bench.set_angles(math.radians(0), math.radians(85))
    assert bench.samples is previous
    for sample in previous:
        assert sample.closure_error_mm < 1e-9
        assert sample.carrier_error_mm < 1e-9
        assert sample.carrier_area_error_mm2 < 1e-8


def test_auto_profile_respects_bounds_pause_and_resume(bench):
    bench.set_angles(bench.bounds[0], bench.bounds[1])
    bench.set_playing(True)
    for _ in range(250):
        for sample in bench.advance(0.05):
            assert (
                bench.bounds[0] - 1e-12
                <= sample.pose.linkage_angle_rad
                <= bench.bounds[1] + 1e-12
            )
            assert sample.boundary_margin_rad >= bench.reserve_rad - 1e-12
            assert sample.loop_sine > 0
            assert sample.closure_error_mm < 1e-8
    angles = tuple(s.pose.linkage_angle_rad for s in bench.samples)
    bench.set_playing(False)
    bench.advance(2)
    assert tuple(s.pose.linkage_angle_rad for s in bench.samples) == angles
    assert all(s.q_dot_rad_s == s.q_ddot_rad_s2 == 0 for s in bench.samples)
    bench.set_playing(True)
    bench.advance(0)
    assert tuple(s.pose.linkage_angle_rad for s in bench.samples) == angles


def test_reported_wheel_velocity_and_acceleration_match_time_differences(bench):
    bench.set_playing(True)
    bench.advance(1.2)
    dt = 1e-4
    previous = bench.samples
    middle = bench.advance(dt)
    following = bench.advance(dt)
    for first, center, last in zip(previous, middle, following):
        velocity = tuple((b - a) / (2 * dt) for a, b in zip(first.pose.o, last.pose.o))
        acceleration = tuple(
            (a - 2 * b + c) / dt**2
            for a, b, c in zip(first.pose.o, center.pose.o, last.pose.o)
        )
        assert center.wheel_velocity_mm_s == pytest.approx(velocity, abs=1e-5)
        assert center.wheel_acceleration_mm_s2 == pytest.approx(acceleration, abs=1e-4)


def test_speed_changes_do_not_jump_the_pose_and_bad_clock_is_rejected(bench):
    bench.set_playing(True)
    bench.advance(0.5)
    previous = tuple(s.pose.o for s in bench.samples)
    bench.set_speed(2)
    bench.advance(0)
    assert tuple(s.pose.o for s in bench.samples) == previous
    for value in (float("nan"), -1):
        with pytest.raises(ValueError):
            bench.advance(value)
    for value in (float("inf"), 0, 3):
        with pytest.raises(ValueError):
            bench.set_speed(value)
