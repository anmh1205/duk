"""Tests for mass-property helpers used by future DUK balance models."""

import math

import pytest

from duk_core.modeling import (
    PlanarRigidBody,
    combine_planar_mass_properties,
    equivalent_pendulum_2d,
)


def test_combined_center_of_mass_and_parallel_axis_inertia() -> None:
    bodies = (
        PlanarRigidBody(
            3.0,
            (0.0, 250.0),
            0.02,
        ),
        PlanarRigidBody(
            1.0,
            (40.0, 250.0),
            0.005,
        ),
    )
    combined = combine_planar_mass_properties(
        bodies
    )
    assert combined.mass_kg == pytest.approx(4.0)
    assert combined.center_mm == pytest.approx(
        (10.0, 250.0)
    )
    expected_parallel_axis = (
        3.0 * 0.010**2
        + 1.0 * 0.030**2
    )
    assert (
        combined.inertia_pitch_at_center_kg_m2
        == pytest.approx(
            0.025 + expected_parallel_axis
        )
    )


def test_equivalent_pendulum_geometry_and_inertia() -> None:
    combined = combine_planar_mass_properties(
        (
            PlanarRigidBody(
                3.0,
                (0.0, 250.0),
                0.02,
            ),
            PlanarRigidBody(
                1.0,
                (40.0, 250.0),
                0.005,
            ),
        )
    )
    pendulum = equivalent_pendulum_2d(
        combined,
        (0.0, 0.0),
    )
    expected_length = math.hypot(
        0.010,
        0.250,
    )
    assert pendulum.length_m == pytest.approx(
        expected_length
    )
    assert (
        pendulum.angle_from_vertical_rad
        == pytest.approx(
            math.atan2(0.010, 0.250)
        )
    )
    assert (
        pendulum.inertia_about_support_kg_m2
        == pytest.approx(
            combined.inertia_pitch_at_center_kg_m2
            + combined.mass_kg * expected_length**2
        )
    )


def test_invalid_mass_is_rejected() -> None:
    with pytest.raises(ValueError):
        PlanarRigidBody(
            0.0,
            (0.0, 0.0),
        )
