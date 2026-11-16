"""CoM diagnostics agree with geometry and whole-system linear momentum."""

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
pytest.importorskip("sympy")

from duk_core.modeling import build_planar_biped_model, load_leg_geometry_json
from duk_core.modeling.planar_numeric import NumericPlanarBiped
from duk_core.modeling.rolling_reduction import lift_symmetric_rolling
from duk_core.simulation.planar import simulate_planar


@pytest.fixture(scope="module")
def planar(geometry_path, research_inputs):
    return NumericPlanarBiped(
        build_planar_biped_model(load_leg_geometry_json(geometry_path)),
        research_inputs["planar"],
    )


@pytest.mark.parametrize("seed", range(3))
def test_center_acceleration_matches_second_difference_of_center_position(planar, seed):
    rng = np.random.default_rng(seed)
    q = np.r_[0.1, 0.5, 0.17, np.deg2rad([92.0, 104.0]), 0.2, -0.1]
    velocity, acceleration = rng.normal(size=(2, 7))
    step = 1e-4

    def center(time):
        position = q + time * velocity + 0.5 * time**2 * acceleration
        return planar.masses @ planar.kinematics(position).centers / planar.masses.sum()

    numerical = (center(step) - 2 * center(0) + center(-step)) / step**2
    np.testing.assert_allclose(
        planar.center_of_mass_acceleration(q, velocity, acceleration),
        numerical,
        rtol=1e-6,
        atol=1e-7,
    )


@pytest.mark.parametrize("seed", range(3))
def test_internal_efforts_cannot_accelerate_the_flight_center_of_mass(planar, seed):
    rng = np.random.default_rng(seed)
    q = np.r_[0.1, 1.0, 0.17, np.deg2rad([92.0, 104.0]), 0.2, -0.1]
    velocity, effort = rng.normal(size=7), rng.normal(size=4)
    dynamics = planar.evaluate(q, velocity)
    acceleration = np.linalg.solve(
        dynamics.mass, dynamics.input @ effort - dynamics.bias
    )
    np.testing.assert_allclose(
        planar.center_of_mass_acceleration(q, velocity, acceleration),
        [0.0, -planar.parameters["g_m_s2"]],
        atol=1e-10,
        rtol=0,
    )


def test_contact_trace_satisfies_whole_system_vertical_momentum(planar):
    angle = np.deg2rad(95)
    lift = lift_symmetric_rolling(
        planar.model.geometry,
        0.0,
        0.0,
        0.0,
        0.0,
        angle,
        mount_xz_m=(0.0, 0.0),
        pitch_offset=planar.equivalent_upper_body(angle).angle_from_vertical_rad,
    )
    normal = planar.masses.sum() * planar.parameters["g_m_s2"] / 2
    hip_effort = planar.static_hip_effort(lift.configuration, [[0.0, normal]] * 2)
    trace = simulate_planar(
        planar,
        np.r_[lift.configuration, lift.velocity],
        [0.0, 0.01, 0.02],
        lambda *_: np.r_[hip_effort, -0.05, -0.05],
        [0.8, 0.8],
        mode="rolling",
        max_step=0.002,
        rtol=1e-7,
        atol=1e-9,
    )
    assert trace.termination == "completed"
    np.testing.assert_allclose(
        trace.com_vertical_acceleration,
        trace.contact_forces[:, :2].sum(axis=1) / planar.masses.sum()
        - planar.parameters["g_m_s2"],
        atol=1e-10,
        rtol=0,
    )


@pytest.mark.parametrize("hip", [3, 4], ids=["left", "right"])
@pytest.mark.parametrize("boundary", ["lower", "upper"])
def test_valid_boundary_flight_diagnostics_never_probe_an_unreachable_pose(
    planar, hip, boundary
):
    lower, upper = planar.model.geometry.geometric_linkage_angle_limits_rad()
    q = np.array([0.0, 1.0, 0.0, 1.6, 1.7, 0.0, 0.0])
    velocity = np.zeros(7)
    q[hip] = lower + 1e-6 if boundary == "lower" else upper - 1e-6
    velocity[hip] = 0.5 if boundary == "lower" else -0.5
    trace = simulate_planar(
        planar,
        np.r_[q, velocity],
        [0.0, 1e-8],
        lambda *_: np.zeros(4),
        [0.0, 0.0],
        mode="flight",
        max_step=1e-8,
        rtol=1e-7,
        atol=1e-9,
    )
    assert trace.termination == "completed"
    assert np.isfinite(trace.com_vertical_acceleration).all()
    np.testing.assert_allclose(
        trace.com_vertical_acceleration,
        -planar.parameters["g_m_s2"],
        atol=1e-5,
        rtol=0,
    )


@pytest.mark.parametrize("acceleration", [np.zeros(6), [np.nan] * 7, [np.inf] * 7])
def test_center_acceleration_rejects_invalid_generalized_acceleration(
    planar, acceleration
):
    with pytest.raises(ValueError, match="seven finite generalized accelerations"):
        planar.center_of_mass_acceleration(np.zeros(7), np.zeros(7), acceleration)


@pytest.mark.parametrize("angle", [80.0, 95.0, 120.0])
def test_symmetric_upper_angle_derivatives_match_composed_mass_properties(
    planar, angle
):
    q, step = np.deg2rad(angle), 1e-4
    left, middle, right = (
        planar.equivalent_upper_body(q + delta).angle_from_vertical_rad
        for delta in (-step, 0, step)
    )
    first, second = planar.upper_body_angle_derivatives(q)
    assert first == pytest.approx((right - left) / (2 * step), rel=2e-5, abs=1e-7)
    assert second == pytest.approx(
        (right - 2 * middle + left) / step**2, rel=2e-5, abs=1e-6
    )


@pytest.mark.parametrize("boundary", ["lower", "upper"])
def test_symmetric_upper_angle_derivatives_stay_on_the_valid_branch(planar, boundary):
    lower, upper = planar.model.geometry.geometric_linkage_angle_limits_rad()
    q = lower + 1e-6 if boundary == "lower" else upper - 1e-6
    assert np.isfinite(planar.upper_body_angle_derivatives(q)).all()


def test_near_boundary_height_study_runs_without_an_outside_probe(
    planar, research_inputs
):
    from duk_core.control.lqr import bryson_weights
    from duk_core.modeling.hip_load import LinearTorqueSpeed
    from duk_core.modeling.trajectories import HipTrajectory
    from duk_core.simulation.height import compare_height_policies

    lower, _ = planar.model.geometry.geometric_linkage_angle_limits_rad()
    q = lower + 1e-6
    envelope = research_inputs["hip_envelope"]
    motor = LinearTorqueSpeed(
        envelope["stall_torque_N_m"],
        envelope["no_load_speed_rad_s"],
        envelope["efficiency"],
    )
    settings, weights = (
        research_inputs["height_experiment"],
        research_inputs["v2_balance"],
    )
    cost, effort = bryson_weights(weights["state_scales"], weights["torque_scale_N_m"])
    traces = compare_height_policies(
        planar,
        HipTrajectory(q, q, 1.0),
        motor,
        period=settings["period_s"],
        duration=settings["period_s"],
        friction=settings["friction"],
        motor_bound=settings["total_wheel_limit_N_m"],
        q_weight=cost,
        r_weight=effort,
        initial_lean=0.0,
        horizon=1,
    )
    for trace in traces.values():
        assert trace.terminal_time == settings["period_s"]
        assert np.isfinite(trace.applied).all()
        assert np.isfinite(trace.terminal_lean)
