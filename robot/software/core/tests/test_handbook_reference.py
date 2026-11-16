"""Regress corrected DUK v3 conventions against independent model identities."""

import pytest

np = pytest.importorskip("numpy")
signal = pytest.importorskip("scipy.signal")
pytest.importorskip("sympy")

from duk_core.control.lqr import linearize
from duk_core.estimation.sagittal import ideal_planar_contact_estimate
from duk_core.modeling import build_planar_biped_model, load_leg_geometry_json
from duk_core.modeling.planar_numeric import NumericPlanarBiped
from duk_core.modeling.rolling_reduction import (
    lift_symmetric_rolling,
    symmetric_forward_wheel_input,
)
from duk_core.modeling.wheeled_pendulum import WheeledPendulum


@pytest.mark.parametrize(
    "name,expected_a43,expected_b4",
    [
        ("handbook_linear_reference", 74.26, -34.97),
        ("v2_balance", 74.28, -34.96),
    ],
)
def test_exact_and_rounded_references_are_distinct_and_reproducible(
    research_inputs, name, expected_a43, expected_b4
):
    data = research_inputs[name]
    model = WheeledPendulum(
        **{key: data[key] for key in WheeledPendulum.__dataclass_fields__}
    )
    a, b = model.linearization()
    a_numeric, b_numeric = linearize(
        lambda state, torque: np.r_[
            state[1],
            model.rolling(state[2], state[3], torque).acceleration[0],
            state[3],
            model.rolling(state[2], state[3], torque).acceleration[1],
        ],
        np.zeros(4),
        0.0,
    )
    np.testing.assert_allclose(a_numeric, a, atol=1e-8, rtol=1e-8)
    np.testing.assert_allclose(b_numeric, b, atol=1e-8, rtol=1e-8)
    # Published rounded values tolerate only their stated decimal precision.
    assert a[3, 2] == pytest.approx(expected_a43, abs=0.005)
    assert b[3, 0] == pytest.approx(expected_b4, abs=0.005)
    diagnostics = model.upright_characteristics()
    positive_pole = max(np.linalg.eigvals(a).real)
    assert diagnostics.unstable_pole_rad_s == pytest.approx(positive_pole)
    assert diagnostics.angle_efolding_time_s * positive_pole == pytest.approx(1)
    numerator, _ = signal.ss2tf(a, b, np.eye(4)[0:1], np.zeros((1, 1)))
    # Remove numerical round-off in theoretically zero leading coefficients.
    coefficients = numerator[0]
    coefficients = coefficients[np.flatnonzero(abs(coefficients) > 1e-10)[0] :]
    roots = np.roots(coefficients)
    assert np.sort(roots) == pytest.approx(
        [
            -diagnostics.axle_position_rhp_zero_rad_s,
            diagnostics.axle_position_rhp_zero_rad_s,
        ],
        abs=1e-10,
    )
    assert (
        np.linalg.matrix_rank(
            np.hstack([np.linalg.matrix_power(a, i) @ b for i in range(4)])
        )
        == 4
    )
    if name == "handbook_linear_reference":
        assert data["wheel_inertia_kg_m2"] == pytest.approx(
            0.5 * data["wheel_mass_kg"] * data["radius_m"] ** 2
        )


@pytest.mark.parametrize("torque", [-0.3, 0.3])
def test_canonical_to_pendulum_conversion_preserves_work_and_contact(
    geometry_path, research_inputs, torque
):
    geometry = load_leg_geometry_json(geometry_path)
    model = NumericPlanarBiped(
        build_planar_biped_model(geometry), research_inputs["planar"]
    )
    q, lean, rate, speed = np.deg2rad(95), 0.04, -0.12, 0.2
    offset = model.equivalent_upper_body(q).angle_from_vertical_rad
    lift = lift_symmetric_rolling(
        geometry,
        0.1,
        lean,
        speed,
        rate,
        q,
        mount_xz_m=(0.0, 0.0),
        pitch_offset=offset,
    )
    assert offset - lift.configuration[2] == pytest.approx(lean)
    assert -lift.velocity[5:] * model.radius == pytest.approx([speed, speed])
    terms = model.evaluate(lift.configuration, lift.velocity)
    world = model.kinematics(lift.configuration)
    upper = [i for i in range(len(model.masses)) if i not in model.wheels]
    direction = (
        np.average(world.centers[upper], axis=0, weights=model.masses[upper])
        - world.centers[model.wheels[0]]
    )
    assert np.arctan2(direction[0], direction[1]) == pytest.approx(lean)
    effort = symmetric_forward_wheel_input(torque)
    assert -sum(effort[2:]) == pytest.approx(torque)
    assert lift.velocity @ terms.input @ effort == pytest.approx(
        torque * (speed / model.radius - rate)
    )
    pendulum = WheeledPendulum.from_biped(model, q)
    response = pendulum.rolling(lean, rate, torque)
    tangent, _ = ideal_planar_contact_estimate(
        effort[2:],
        pendulum.wheel_inertia_kg_m2 / 2,
        np.full(2, -response.acceleration[2]),
        pendulum.radius_m,
        pendulum.upper_mass_kg + pendulum.wheel_mass_kg,
        pendulum.gravity_m_s2,
        0.0,
    )
    assert sum(tangent) == pytest.approx(response.tangent, abs=1e-12)
