"""Spatial dynamics must reduce to the existing sagittal model."""

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
pytest.importorskip("sympy")

from duk_core.modeling import build_planar_biped_model, load_leg_geometry_json
from duk_core.modeling.planar_numeric import NumericPlanarBiped
from duk_core.modeling.spatial_biped import SpatialBiped


@pytest.fixture(scope="module")
def spatial(geometry_path, research_inputs):
    data = research_inputs
    planar = NumericPlanarBiped(
        build_planar_biped_model(load_leg_geometry_json(geometry_path)), data["planar"]
    )
    p = data["spatial"]
    return SpatialBiped(planar, p["lateral_mounts_m"], p["transverse_inertias_kg_m2"])


def test_spatial_sagittal_projection(spatial):
    q = np.r_[0.12, 0.42, 0.13, np.deg2rad([92.0, 101.0]), 0.3, -0.24]
    v = np.array([0.12, -0.03, 0.2, 0.5, -0.4, -0.7, 0.6])
    planar = spatial.planar.evaluate(q, v)
    state, lift = spatial.sagittal_state(q, v)
    model = spatial.evaluate(state)
    np.testing.assert_allclose(lift.T @ model.mass @ lift, planar.mass, atol=1e-11)
    np.testing.assert_allclose(lift.T @ model.actuation, planar.input, atol=1e-11)
    np.testing.assert_allclose(
        model.kinematics.contact[:, 0, :] @ lift, planar.contact[2:], atol=1e-11
    )
    np.testing.assert_allclose(
        model.kinematics.contact[:, 2, :] @ lift, planar.contact[:2], atol=1e-11
    )
    np.testing.assert_allclose(model.kinematics.gaps, planar.gaps, atol=1e-11)
    dt = 1e-5
    _, before = spatial.sagittal_state(q - dt * v, v)
    _, after = spatial.sagittal_state(q + dt * v, v)
    drift = (after - before) @ v / (2 * dt)
    np.testing.assert_allclose(
        lift.T @ (model.bias + model.mass @ drift), planar.bias, atol=1e-8
    )
    assert np.linalg.eigvalsh(model.mass)[0] > 0


def test_spatial_task_rates_match_finite_differences(spatial):
    q = np.r_[0.12, 0.42, 0.13, np.deg2rad([92.0, 101.0]), 0.3, -0.24]
    state, _ = spatial.sagittal_state(q, [0.12, -0.03, 0.2, 0.5, -0.4, -0.7, 0.6])
    v = state.velocity.copy()
    v[3:6] += [0.07, 0.01, -0.14]
    state = state.advance(v, 0.0)
    step = 1e-5
    before = spatial.kinematics(state.advance(v, -step))
    after = spatial.kinematics(state.advance(v, step))
    terms = spatial.evaluate(state)
    np.testing.assert_allclose(
        spatial.kinematics(state).task_jacobian @ v,
        (after.tasks - before.tasks) / (2 * step),
        atol=1e-8,
    )
    np.testing.assert_allclose(
        terms.task_bias,
        (after.task_jacobian - before.task_jacobian) @ v / (2 * step),
        atol=1e-8,
    )
    np.testing.assert_allclose(
        terms.contact_bias,
        (after.contact - before.contact) @ v / (2 * step),
        atol=1e-8,
    )


def test_valid_pose_near_boundary_needs_no_outside_probe(spatial):
    from duk_core.modeling.spatial_biped import SpatialState

    lower, _ = spatial.geometry.geometric_linkage_angle_limits_rad()
    state = SpatialState(
        np.array([0.0, 0.0, 0.4]),
        np.eye(3),
        np.array([lower + 1e-6, 1.7, 0.0, 0.0]),
        np.r_[np.zeros(6), -0.5, 0.0, 0.0, 0.0],
    )
    terms = spatial.evaluate(state)
    assert np.isfinite(
        np.r_[terms.bias, terms.contact_bias.ravel(), terms.task_bias]
    ).all()


def test_free_spatial_motion_preserves_energy_to_first_order(spatial):
    from scipy.spatial.transform import Rotation

    from duk_core.modeling.spatial_biped import SpatialState

    q = np.r_[0.0, 0.5, 0.10, np.deg2rad([94.0, 101.0]), 0.3, -0.24]
    initial, _ = spatial.sagittal_state(q, [0.1, -0.05, 0.16, 0.4, -0.3, -0.6, 0.7])
    rotation = Rotation.from_rotvec([0.13, -0.12, 0.18]).as_matrix()
    v = initial.velocity + np.array(
        [0.0, -0.1, 0.0, 0.2, 0.0, -0.15, 0.0, 0.0, 0.0, 0.0]
    )
    state = SpatialState(initial.position, rotation, initial.joints, v)
    dynamics = spatial.evaluate(state)
    acceleration = np.linalg.solve(dynamics.mass, -dynamics.bias)

    def energy(pose):
        terms = spatial.evaluate(pose)
        return 0.5 * pose.velocity @ terms.mass @ pose.velocity + (
            spatial.gravity * spatial.masses @ terms.kinematics.centers[:, 2]
        )

    step = 1e-5
    plus = state.advance(v, step)
    minus = state.advance(v, -step)
    plus = SpatialState(
        plus.position, plus.rotation, plus.joints, v + step * acceleration
    )
    minus = SpatialState(
        minus.position, minus.rotation, minus.joints, v - step * acceleration
    )
    assert (energy(plus) - energy(minus)) / (2 * step) == pytest.approx(0.0, abs=1e-8)


def test_hierarchy_preserves_first_task_and_rejects_invalid_limits(spatial):
    from duk_core.control.whole_body import AccelerationTask, allocate

    q = np.r_[0.0, 0.3, 0.0, np.deg2rad([95.0, 95.0]), 0.0, 0.0]
    state, _ = spatial.sagittal_state(q, np.zeros(7))
    terms = spatial.evaluate(state)
    row = terms.kinematics.task_jacobian[0]
    tasks = [
        AccelerationTask("first", row, 0.2, 1.0),
        AccelerationTask("second", row, -0.2, 1.0),
    ]
    limits = dict(
        friction=np.full(2, 0.6),
        effort_bounds=np.array([5.0, 5.0, 1.5, 1.5]),
        hip_limits=np.deg2rad([78.0, 125.0]),
        hip_speed_limit=7.0,
        lookahead=0.08,
        velocity_stabilization=7.0,
        deadline_s=2.0,
    )
    result = allocate(
        terms, state, tasks, ("rolling", "rolling"), method="hierarchical", **limits
    )
    assert result.status == "solved"
    assert row @ result.solution[:10] == pytest.approx(0.2, abs=2e-6)
    assert result.residual < 2e-6
    impossible = allocate(
        terms,
        state,
        tasks,
        ("rolling", "rolling"),
        method="weighted",
        **(limits | {"hip_limits": (2.8, 2.9)}),
    )
    assert impossible.status == "infeasible" and impossible.effort is None


def test_hierarchy_keeps_a_feasible_command_when_both_wheels_are_airborne(spatial):
    from duk_core.control.whole_body import AccelerationTask, allocate

    q = np.r_[0.0, 0.5, 0.0, np.deg2rad([95.0, 95.0]), 0.0, 0.0]
    state, _ = spatial.sagittal_state(q, np.zeros(7))
    terms = spatial.evaluate(state)
    # Unforced free fall is a feasible witness. Lower-priority tasks cannot
    # make earlier attained outputs infeasible, even with all six forces fixed.
    tasks = [AccelerationTask(str(i), np.eye(10)[i], 0.0, 1.0) for i in range(4)]
    result = allocate(
        terms,
        state,
        tasks,
        ("flight", "flight"),
        friction=np.full(2, 0.6),
        effort_bounds=np.array([5.0, 5.0, 1.5, 1.5]),
        hip_limits=np.deg2rad([78.0, 125.0]),
        hip_speed_limit=7.0,
        lookahead=0.08,
        velocity_stabilization=7.0,
        deadline_s=2.0,
        method="hierarchical",
    )
    assert result.status == "solved"
    np.testing.assert_allclose(result.solution[14:], 0.0, atol=1e-9)
    np.testing.assert_allclose(result.solution[:2], 0.0, atol=2e-6)
    np.testing.assert_allclose(
        terms.mass @ result.solution[:10] + terms.bias,
        terms.actuation @ result.effort,
        atol=2e-6,
    )


@pytest.mark.parametrize("fault", ["sensor", "deadline"])
def test_fault_discards_drive_request_before_a_partial_final_step(
    spatial, monkeypatch, fault, research_inputs
):
    from duk_core.modeling.hip_load import LinearTorqueSpeed
    from duk_core.simulation import whole_body as whole_body_experiment

    def unexpected_allocation(*args, **kwargs):
        pytest.fail("A rejected sample must not reach the allocator")

    monkeypatch.setattr(whole_body_experiment, "allocate", unexpected_allocation)
    fixture = research_inputs
    settings = fixture["whole_body"] | {"substeps": 2}
    scenario = settings["scenario"] | {
        "duration_s": 0.007,
        "feature_start_s": 0.0,
        "initial_lean_deg": 0.0,
        "fault": fault,
        "fault_start_s": 0.0,
        "fault_end_s": 0.02,
        "drive_speed_m_s": 0.1,
        "drive_end_s": 0.02,
    }
    limits = fixture["hip_envelope"]
    motor = LinearTorqueSpeed(
        limits["stall_torque_N_m"], limits["no_load_speed_rad_s"], limits["efficiency"]
    )
    trace = whole_body_experiment.run_whole_body(
        spatial,
        motor,
        fixture["v2_balance"],
        settings,
        scenario,
        method="weighted",
        priority="balance_first",
    )
    assert trace.requested_reference[0, 2] == 0.1
    np.testing.assert_allclose(trace.effective_reference[0, 1:], 0.0)
    np.testing.assert_allclose(trace.feature_amount[0], 0.0)
    np.testing.assert_allclose(trace.applied_effort[0, 2:], 0.0, atol=1e-10)
    if fault == "sensor":
        assert trace.command_source == ("stop",)
        assert trace.solver_status == ("skipped_sensor",)
        assert trace.terminal_time == 0.0
    else:
        assert trace.command_source == ("balance_fallback",)
        assert trace.terminal_time == pytest.approx(0.007)


@pytest.mark.parametrize("direction", [-1, 1])
def test_hip_bounds_exclude_an_interior_overshoot(spatial, direction):
    from duk_core.control.whole_body import _physical_constraints
    from duk_core.modeling.spatial_biped import SpatialState

    q = np.r_[0.0, 0.4, 0.0, [1.5, 1.5], 0.0, 0.0]
    state, _ = spatial.sagittal_state(q, np.zeros(7))
    joints, velocity = state.joints.copy(), state.velocity.copy()
    joints[0] = 1.99 if direction > 0 else 1.41
    velocity[6] = direction
    state = SpatialState(state.position, state.rotation, joints, velocity)
    *_, lower, upper = _physical_constraints(
        spatial.evaluate(state),
        state,
        ("flight", "flight"),
        np.ones(2),
        np.ones(4),
        np.array([1.4, 2.0]),
        7.0,
        0.08,
        0.0,
    )
    # The endpoint returns to the starting angle, but the vertex is 0.01 rad
    # beyond the travel limit. This acceleration must be excluded.
    overshoot = -25.0 * direction
    assert not lower[6] <= overshoot <= upper[6]
    braking = upper[6] if direction > 0 else lower[6]
    times = np.linspace(0, 0.08, 1001)
    angles = joints[0] + velocity[6] * times + 0.5 * braking * times**2
    assert angles.min() >= 1.4 - 1e-12
    assert angles.max() <= 2.0 + 1e-12


@pytest.mark.parametrize("event", ["fall", "hip_travel"])
def test_final_integration_step_cannot_hide_a_violation(
    spatial, monkeypatch, event, research_inputs
):
    from scipy.spatial.transform import Rotation

    from duk_core.modeling.hip_load import LinearTorqueSpeed
    from duk_core.modeling.spatial_biped import SpatialState
    from duk_core.simulation import whole_body as whole_body_experiment

    data = research_inputs
    settings = data["whole_body"] | {"substeps": 1}
    scenario = settings["scenario"] | {
        "duration_s": 0.007,
        "initial_lean_deg": 0.0,
        "fault": "deadline",
        "fault_start_s": 0.0,
        "fault_end_s": 0.02,
    }

    def final_fall(state, effort, friction, law, dt):
        joints = state.joints.copy()
        if event == "hip_travel":
            joints[0] = np.deg2rad(60)  # Outside the four-bar assembly domain.
        fallen = SpatialState(
            state.position,
            Rotation.from_euler("x", 35, degrees=True).as_matrix(),
            joints,
            state.velocity,
        )
        return fallen, np.array([[0.0, 0.0, 20.0]] * 2)

    monkeypatch.setattr(spatial, "compliant_step", final_fall)
    envelope = data["hip_envelope"]
    motor = LinearTorqueSpeed(
        envelope["stall_torque_N_m"],
        envelope["no_load_speed_rad_s"],
        envelope["efficiency"],
    )
    trace = whole_body_experiment.run_whole_body(
        spatial,
        motor,
        data["v2_balance"],
        settings,
        scenario,
        method="weighted",
        priority="balance_first",
    )
    assert trace.stop_reason == ("flight_or_fall" if event == "fall" else "hip_travel")
    assert trace.terminal_time == pytest.approx(0.007)
    if event == "fall":
        assert trace.terminal_pose[2] == pytest.approx(np.deg2rad(35))
        assert trace.peak_abs_lean_roll[1] == pytest.approx(np.deg2rad(35))
    else:
        assert np.isnan(trace.terminal_pose).all()


def test_rolling_odometry_compensates_tilt_and_ignores_world_translation(spatial):
    from scipy.spatial.transform import Rotation

    from duk_core.modeling.spatial_biped import SpatialState

    velocity = np.array([0.2, 0.1, 0, 1, 0.4, -0.6, 0.3, -0.2, 0, 0.0])
    state = SpatialState(
        np.array([0, 0, 0.4]),
        Rotation.from_euler("xyz", [0.4, -0.25, 0.9]).as_matrix(),
        np.r_[np.deg2rad([92, 103]), 0, 0],
        velocity,
    )
    k = spatial.kinematics(state)
    longitudinal = k.forward @ k.contact
    velocity[8:] = np.linalg.solve(longitudinal[:, 8:], -longitudinal @ velocity)
    np.testing.assert_allclose(longitudinal @ velocity, 0, atol=1e-12)
    axle_speed = (k.linear[spatial.wheels] @ velocity) @ k.forward
    np.testing.assert_allclose(
        spatial.rolling_odometry(k, velocity), axle_speed, atol=1e-12
    )
    changed_translation = velocity.copy()
    changed_translation[:3] += [0.3, -0.8, 1.0]
    np.testing.assert_allclose(
        spatial.rolling_odometry(k, changed_translation), axle_speed, atol=1e-12
    )


@pytest.fixture(scope="module")
def damped_spatial(spatial, research_inputs):
    data = research_inputs
    parameters = dict(spatial.planar.parameters) | {
        "b_q_L_N_m_s": 0.12,
        "b_q_R_N_m_s": 0.07,
        "b_w_L_N_m_s": 0.015,
        "b_w_R_N_m_s": 0.023,
    }
    return SpatialBiped(
        NumericPlanarBiped(spatial.planar.model, parameters),
        data["spatial"]["lateral_mounts_m"],
        data["spatial"]["transverse_inertias_kg_m2"],
    )


@pytest.mark.parametrize("seed", range(10))
def test_spatial_power_geometry_and_world_frame_invariants(damped_spatial, seed):
    from scipy.linalg import block_diag
    from scipy.spatial.transform import Rotation

    from duk_core.control.task_policy import wrap_angle
    from duk_core.modeling.spatial_biped import SpatialState

    model = damped_spatial
    rng = np.random.default_rng(seed)
    state = SpatialState(
        np.r_[rng.uniform(-1, 1, 2), 0.4],
        Rotation.from_euler("xyz", rng.uniform(-0.4, 0.4, 3)).as_matrix(),
        np.r_[np.deg2rad(rng.uniform(80, 120, 2)), rng.uniform(-3, 3, 2)],
        rng.uniform(-1, 1, 10),
    )
    terms = model.evaluate(state)
    v, k = state.velocity, terms.kinematics
    np.testing.assert_allclose(terms.mass, terms.mass.T, atol=1e-12)
    assert np.linalg.eigvalsh(terms.mass)[0] > 0
    step = 1e-5
    before = model.kinematics(state.advance(v, -step))
    after = model.kinematics(state.advance(v, step))
    np.testing.assert_allclose(
        (after.centers - before.centers) / (2 * step), k.linear @ v, atol=1e-9
    )
    np.testing.assert_allclose(
        (after.gaps - before.gaps) / (2 * step), (k.contact @ v)[:, 2], atol=1e-9
    )
    np.testing.assert_allclose(
        wrap_angle(after.tasks - before.tasks) / (2 * step),
        k.task_jacobian @ v,
        atol=1e-8,
    )

    def energy(kinematics):
        linear, angular = kinematics.linear @ v, kinematics.angular @ v
        return (
            0.5 * model.masses @ np.sum(linear**2, axis=1)
            + 0.5 * np.einsum("ij,ijk,ik->", angular, kinematics.inertias, angular)
            + model.gravity * model.masses @ kinematics.centers[:, 2]
        )

    effort = rng.uniform(-1, 1, 4)
    force = rng.uniform(-2, 2, (2, 3))
    force[:, 2] += 20
    acceleration = np.linalg.solve(
        terms.mass,
        terms.actuation @ effort
        - terms.bias
        + k.contact.reshape(6, 10).T @ force.ravel(),
    )
    energy_rate = (energy(after) - energy(before)) / (
        2 * step
    ) + v @ terms.mass @ acceleration
    damping = np.array([0.12, 0.07, 0.015, 0.023])
    power = effort @ v[6:] - damping @ v[6:] ** 2 + np.sum(force * (k.contact @ v))
    assert energy_rate == pytest.approx(power, abs=2e-8)

    # Rotating/translating the horizontal world axes cannot change the physics.
    yaw = Rotation.from_euler("z", 1.2).as_matrix()
    transform = block_diag(yaw, yaw, np.eye(4))
    moved = SpatialState(
        yaw @ state.position + [0.3, -0.7, 0],
        yaw @ state.rotation,
        state.joints,
        transform @ v,
    )
    other = model.evaluate(moved)
    np.testing.assert_allclose(
        other.mass, transform @ terms.mass @ transform.T, atol=1e-11
    )
    np.testing.assert_allclose(other.bias, transform @ terms.bias, atol=2e-8)
    np.testing.assert_allclose(
        other.kinematics.contact, yaw @ k.contact @ transform.T, atol=1e-11
    )
    np.testing.assert_allclose(other.kinematics.gaps, k.gaps, atol=1e-12)
