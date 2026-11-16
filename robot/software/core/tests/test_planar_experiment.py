"""Analytic guard cases with independent constraint rows and constant inertia."""

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from duk_core.modeling.planar_numeric import PlanarDynamics
from duk_core.simulation.planar import simulate_planar


class GuardModel:
    radius = 0.075
    masses = np.array([1.0])
    parameters = {"g_m_s2": 10.0}

    def center_of_mass_acceleration(self, q, v, a):
        # V = m*g*(q[1] + q[3]) for this analytic guard fixture.
        return np.array([0.0, a[1] + a[3]])

    def evaluate(self, q, v):
        gravity = np.array([0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 0.0])
        return PlanarDynamics(
            np.eye(7),
            gravity,
            np.eye(7)[:, [0, 2, 4, 5]],
            q[[1, 3]],
            np.eye(7)[[1, 3, 0, 2]],
            np.zeros(4),
            gravity @ q,
            gravity,
        )


def test_zero_friction_with_zero_tangent_force_does_not_trigger_a_guard():
    trace = simulate_planar(
        GuardModel(),
        np.zeros(14),
        [0.0, 0.01, 0.2],
        lambda *_: np.zeros(4),
        [0.0, 0.0],
        mode="rolling",
        max_step=np.inf,
        rtol=1e-7,
        atol=1e-9,
    )
    assert trace.termination == "completed"
    assert trace.time[-1] == pytest.approx(0.2)
    np.testing.assert_allclose(trace.contact_forces[:, 2:], 0)


def test_rolling_limit_and_touchdown_preserve_off_grid_event_samples():
    model = GuardModel()
    times = [0.0, 0.025, 0.1, 0.15]
    integration = dict(max_step=0.02, rtol=1e-7, atol=1e-9)
    rolling = simulate_planar(
        model,
        np.zeros(14),
        times,
        lambda t, _: [t, 0.0, 0.0, 0.0],
        [0.005, 0.005],
        mode="rolling",
        **integration,
    )
    assert rolling.termination == "rolling_limit"
    assert rolling.time[-1] == pytest.approx(0.05)
    assert abs(rolling.contact_forces[-1, 2]) == pytest.approx(0.05)
    airborne = np.zeros(14)
    airborne[[1, 3]] = 0.02
    flight = simulate_planar(
        model,
        airborne,
        times,
        lambda *_: np.zeros(4),
        [0.0, 0.0],
        mode="flight",
        **integration,
    )
    assert flight.termination == "touchdown"
    assert flight.time[-1] == pytest.approx(np.sqrt(0.04 / 10.0))
    np.testing.assert_allclose(flight.state[-1, [1, 3]], 0, atol=1e-12)
