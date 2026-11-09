"""Check the locked-hip projection and signs against the canonical model."""

import math

import pytest

np = pytest.importorskip("numpy")
sp = pytest.importorskip("sympy")

from test_planar_biped import GEOMETRY
from duk_core.modeling import build_planar_biped_model, load_leg_geometry_json
from duk_core.modeling.rolling_reduction import (
    lift_symmetric_rolling,
    symmetric_forward_wheel_input,
)


def test_lift_respects_rolling_and_projects_reaction_torque():
    geometry = load_leg_geometry_json(GEOMETRY)
    model = build_planar_biped_model(geometry)
    q = math.radians(105)
    lift = lift_symmetric_rolling(
        geometry, 0.1, 0.05, 0.2, 0.1, q, mount_xz_m=(0.0, 0.0), pitch_offset=0.0
    )
    substitutions = model.bind_configuration(lift.configuration)
    substitutions.update(
        {
            symbol: 0
            for name, symbol in model.parameters.items()
            if name.startswith("a_")
        }
    )
    jac = np.asarray(model.contact_jacobian.subs(substitutions), dtype=float)
    inputs = np.asarray(model.input_matrix.subs(substitutions), dtype=float)
    assert np.max(abs(jac @ lift.tangent)) < 1e-12
    assert np.linalg.matrix_rank(jac) == 4
    assert np.linalg.matrix_rank(np.vstack((jac, np.eye(7)[3:5]))) == 5
    assert lift.tangent.T @ inputs @ symmetric_forward_wheel_input(
        1.0
    ) == pytest.approx([1000.0 / geometry.wheel_radius_mm, -1.0])


def test_prescribed_hip_motion_matches_finite_differences():
    geometry = load_leg_geometry_json(GEOMETRY)
    q, dq, ddq = math.radians(105), 0.2, -0.4
    speed, lean_rate = 0.3, -0.5

    def lift(t):
        return lift_symmetric_rolling(
            geometry,
            speed * t,
            lean_rate * t,
            speed,
            lean_rate,
            q + dq * t + ddq * t**2 / 2,
            mount_xz_m=(0.03, -0.01),
            pitch_offset=0.2,
            q_rate=dq + ddq * t,
            q_acceleration=ddq,
        )

    step = 1e-5
    left, right, center = lift(-step), lift(step), lift(0.0)
    assert center.velocity == pytest.approx(
        (right.configuration - left.configuration) / (2 * step)
    )
    assert center.bias == pytest.approx(
        (right.velocity - left.velocity) / (2 * step), abs=1e-8
    )
