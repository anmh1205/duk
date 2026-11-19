"""Focused regressions for controller semantics and constrained effort."""

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from duk_core.control.lqr import bryson_weights, discrete_lqr
from duk_core.control.sagittal_balance import (
    balance_command,
    project_torque,
    traction_interval,
)
from duk_core.control.task_policy import FeatureTransition, Supervisor
from duk_core.estimation.sagittal import SlipGatedVelocity
from duk_core.modeling.contact_dynamics import (
    CompliantContact,
    constrained_acceleration,
)
from duk_core.modeling.wheeled_pendulum import WheeledPendulum


def test_unanchored_regulator_ignores_translated_position():
    model = WheeledPendulum(0.6, 0.00169, 3.5, 0.06, 0.18, 0.075, 9.81)
    a, b = model.linearization()
    q, r = bryson_weights([0.5, 0.05, 0.5], 1.5)
    design = discrete_lqr(a[1:, 1:], b[1:], q, r, 0.005)
    assert design.spectral_radius < 1
    state = [0.0, 0.05, 0.03, -0.02]
    assert balance_command(design.gain, state) == balance_command(
        design.gain, [2.0, *state[1:]]
    )
    with pytest.raises(ValueError, match="explicitly anchored"):
        balance_command(design.gain, state, anchor=0.0)


def test_traction_limit_is_checked_with_the_coupled_normal_force():
    model = WheeledPendulum(0.6, 0.00169, 3.5, 0.06, 0.18, 0.075, 9.81)
    lean, rate, motor, grip = 0.15, 0.4, 3.0, 0.18
    interval = traction_interval(model, lean, rate, motor, grip)
    assert interval is not None
    decision = project_torque(4.0, motor, interval)
    assert decision.reason == "traction"
    response = model.rolling(lean, rate, decision.applied)
    assert response.normal > 0
    assert abs(response.tangent) <= grip * response.normal + 1e-10
    assert project_torque(4.0, motor, None).applied is None


def test_contact_rejects_rank_deficiency_and_never_pulls():
    with pytest.raises(ValueError, match="Dependent"):
        constrained_acceleration(
            np.eye(2), np.zeros(2), np.array([[1.0, 0.0], [1.0, 0.0]]), np.zeros(2)
        )
    contact = CompliantContact(1000.0, 10.0, 0.01)
    speed = np.array([[0.2, -0.1], [0.4, 0.3], [-0.3, 0.2]])
    normal, force = contact.forces(
        np.array([0.001, -0.001, -0.002]), np.array([-1.0, 0.0, 1.0]), speed, 0.5
    )
    assert normal == pytest.approx([0.0, 1.0, 0.0])
    assert np.sum(force * speed) < 0
    assert np.linalg.norm(force, axis=1) == pytest.approx([0.0, 0.5, 0.0])


def test_slip_gate_does_not_absorb_a_spinning_wheel():
    observer = SlipGatedVelocity(0.0, 0.05, 2, 0.2)
    first = observer.step(0.01, 0.0, 1.0)
    second = observer.step(0.01, 0.0, 1.0)
    assert not first.odometry_accepted and not first.slip_detected
    assert not second.odometry_accepted and second.slip_detected
    assert second.speed == 0.0
    observer.step(0.01, 0.0, 0.0)
    recovered = observer.step(0.01, 0.0, 0.0)
    assert recovered.odometry_accepted and not recovered.slip_detected


@pytest.mark.parametrize("count", [1.5, np.nan, np.inf, 0, -1])
def test_slip_confirmation_requires_a_positive_sample_count(count):
    with pytest.raises(ValueError, match="confirmation count"):
        SlipGatedVelocity(0.0, 0.05, count, 0.2)


@pytest.mark.parametrize(
    ("times", "acceleration", "wheel", "contact"),
    [
        ([np.nan], [0], [0], [True]),
        ([0, 0], [0, 0], [0, 0], [True, True]),
        ([1, 0], [0, 0], [0, 0], [True, True]),
        ([0, 1, 2], [0, 1, np.nan], [0, 0, 0], [True] * 3),
        ([0, 1], [0, 0], [np.inf, 0], [True, True]),
        ([0, 1], [0, 0], [0, 0], [True, np.nan]),
    ],
)
def test_invalid_sensor_replay_is_rejected_before_updating_observer(
    times, acceleration, wheel, contact
):
    from duk_core.estimation.sagittal import replay_velocity

    observer = SlipGatedVelocity(0.3, 0.05, 2, 0.2)
    with pytest.raises(ValueError, match="finite signals"):
        replay_velocity(times, acceleration, wheel, contact, observer)
    assert observer.speed == 0.3


def test_velocity_replay_integrates_irregular_intervals_and_marks_initial_prior():
    from duk_core.estimation.sagittal import replay_velocity

    # Constant independent acceleration has an exact integral even on this grid.
    times = np.array([0.0, 0.1, 0.15, 0.4])
    truth = 0.3 + 2 * times
    records = replay_velocity(
        times,
        np.full(4, 2.0),
        truth,
        np.ones(4, dtype=bool),
        SlipGatedVelocity(0.3, 0.05, 2, 0.2),
    )
    np.testing.assert_allclose([record.speed for record in records], truth)
    assert not records[0].odometry_accepted
    assert all(record.odometry_accepted for record in records[1:])


def test_toggle_exit_retracts_actual_contribution_and_stop_latches():
    feature = FeatureTransition()
    for _ in range(10):
        feature.step(True, True, 0.6, 0.01, 0.1)
    assert feature.status == "Limited" and feature.amount == pytest.approx(0.6)
    assert feature.step(False, True, 1.0, 0.01, 0.1) == pytest.approx(0.5)
    assert feature.status == "Exit"
    assert feature.step(True, True, 1.0, 0.01, 0.1) == pytest.approx(0.6)
    assert feature.status == "Enter"
    assert feature.step(True, False, 1.0, 0.01, 0.1) == 0.0
    assert feature.status == "Paused"
    assert feature.rate == 0.0
    supervisor = Supervisor(2, 3)
    flags = dict(sensor_valid=True, supported=True, fallback_valid=True, fall=False)
    assert supervisor.step("deadline", **flags) == "reduced"
    assert supervisor.step("deadline", **flags) == "fallback"
    assert [supervisor.step("solved", **flags) for _ in range(3)] == [
        "reduced",
        "reduced",
        "normal",
    ]
    assert supervisor.step("solved", **(flags | {"sensor_valid": False})) == "stop"
    assert supervisor.step("solved", **flags) == "stop"


@pytest.mark.parametrize(
    "center,capacity_change",
    [(0.15, None), (0.40, None), (0.31, 0.28)],
    ids=["enter", "exit", "capacity-reduction"],
)
def test_height_reference_derivatives_follow_the_admitted_height(
    research_inputs, center, capacity_change
):
    from types import SimpleNamespace

    from duk_core.simulation._whole_body_tasks import ReferencePolicy

    # Compare derivatives with the actual height path, independently of its
    # product-rule implementation. Samples lie inside each linear ramp segment.
    dt = 0.001
    settings = research_inputs["whole_body"] | {"period_s": dt}
    scenario = settings["scenario"] | {
        "feature_start_s": 0.10,
        "bounce_off_s": 0.35,
        "height_offset_m": 0.008,
    }
    policy = ReferencePolicy(settings, scenario, 0.4)
    state = SimpleNamespace(position=np.zeros(3))
    terms = SimpleNamespace(kinematics=SimpleNamespace(tasks=np.zeros(4)))
    observation = SimpleNamespace(modes=("rolling", "rolling"))
    samples = []
    middle = round(center / dt)
    for tick in range(middle + 2):
        time = tick * dt
        if capacity_change is not None and time >= capacity_change:
            scenario["reference_capacity"] = 0.4
        reference = policy.update(
            state,
            terms,
            observation,
            time,
            supervisor_mode="normal",
            sensor_valid=True,
            fallen=False,
            previous_usage=0.0,
        )
        if tick >= middle - 1:
            samples.append(reference)
    previous, current, following = samples
    heights = [sample.effective[0] for sample in samples]
    assert current.height_rate == pytest.approx(
        (heights[2] - heights[0]) / (2 * dt), abs=1e-5
    )
    assert current.height_acceleration == pytest.approx(
        (heights[2] - 2 * heights[1] + heights[0]) / dt**2, abs=1e-4
    )


def test_projection_cannot_bypass_motor_limits_with_a_wide_interval():
    assert project_torque(9, 3, (-10, 10)).applied == 3
    assert project_torque(-9, 3, (-10, 10)).reason == "motor"
    assert project_torque(9, 3, (4, 10)).applied is None
    with pytest.raises(ValueError, match="NaN"):
        project_torque(9, 3, (np.nan, 10))


@pytest.mark.parametrize("violation", ["midpoint", "endpoint"])
def test_held_torque_step_rejects_intermediate_contact_failure(violation):
    from types import SimpleNamespace

    from duk_core.simulation.height import _rolling_step

    def response(normal):
        return SimpleNamespace(
            derivative=np.ones(4),
            hip_effort=np.zeros(2),
            normal_force=normal,
            tangent_force=np.zeros(2),
        )

    limit = 0.05 if violation == "midpoint" else 0.10
    trajectory = SimpleNamespace(evaluate=lambda time: (time, 0, 0))
    plant = SimpleNamespace(
        evaluate=lambda state, effort, motion: response(-1 if motion[0] >= limit else 1)
    )
    motor = SimpleNamespace(joint_limit=lambda speed: 1)
    state, status = _rolling_step(
        plant, trajectory, motor, 0.5, np.zeros(4), 0.1, 0, 0.1, response(1)
    )
    assert state is None
    assert status == violation + "_infeasible"


def test_infeasible_balance_interval_discards_queued_torque(monkeypatch):
    from duk_core.simulation import balance as sagittal_balance

    model = WheeledPendulum(0.6, 0.00169, 3.5, 0.06, 0.18, 0.075, 9.81)
    intervals = iter([(-3.0, 3.0), (-3.0, 3.0), None])
    monkeypatch.setattr(
        sagittal_balance, "traction_interval", lambda *_: next(intervals)
    )
    trace = sagittal_balance.simulate_balance(
        model,
        [0.0, -1.0, 0.0],
        0.02,
        period=0.005,
        duration=0.05,
        friction=0.8,
        motor_bound=3.0,
        slip_scale=0.01,
        fall_angle=0.5,
        feedback="truth",
        traction_budget=0.5,
        delay_samples=1,
    )
    assert trace.outcome == "traction_infeasible"
    assert trace.time[-1] == pytest.approx(0.01)
    assert trace.applied[-2] > 0
    assert trace.applied[-1] == 0
    assert trace.limit_reason[-1] == "infeasible"


@pytest.mark.parametrize(
    ("duration", "period", "count"),
    [(0.9, 0.015, 60), (0.007, 0.015, 1), (0.019, 0.015, 2), (2.3, 0.02, 115)],
)
def test_sample_grid_has_no_rounding_tick_and_keeps_partial_tail(
    duration, period, count
):
    from duk_core.simulation._sampled import sample_intervals

    starts, ends = sample_intervals(duration, period)
    assert len(starts) == count
    np.testing.assert_allclose(starts[1:], ends[:-1], atol=0, rtol=0)
    assert np.all(ends > starts)
    assert ends[-1] == duration


@pytest.mark.parametrize("seed", range(8))
def test_finite_slip_obeys_motor_power_and_friction_dissipation(seed):
    model = WheeledPendulum(0.6, 0.00169, 3.5, 0.06, 0.18, 0.075, 9.81)
    rng = np.random.default_rng(seed)
    lean, rate, speed, spin_rate, torque = rng.uniform(-0.4, 0.4, 5)
    slip = speed - model.radius_m * spin_rate
    response = model.finite_slip(lean, rate, torque, slip, 0.5, 0.02)
    assert response.normal > 0
    assert response.tangent * slip <= 0
    assert abs(response.tangent) <= 0.5 * response.normal
    c = model.upper_mass_kg * model.length_m
    mass = np.array(
        [
            [model.upper_mass_kg + model.wheel_mass_kg, c * np.cos(lean), 0],
            [c * np.cos(lean), model.moment, 0],
            [0, 0, model.wheel_inertia_kg_m2],
        ]
    )
    velocity = np.array([speed, rate, spin_rate])
    energy_rate = (
        velocity @ mass @ response.acceleration
        - c * np.sin(lean) * speed * rate**2
        - c * model.gravity_m_s2 * np.sin(lean) * rate
    )
    assert energy_rate == pytest.approx(
        torque * (spin_rate - rate) + response.tangent * slip, abs=1e-11
    )


@pytest.mark.parametrize(
    "friction,scale",
    [(-0.1, 0.01), (np.nan, 0.01), (0.5, 0.0), (0.5, -0.01), (0.5, np.inf)],
)
def test_pendulum_friction_cannot_be_active_or_undefined(
    research_inputs, friction, scale
):
    data = research_inputs["v2_balance"]
    model = WheeledPendulum(
        **{name: data[name] for name in WheeledPendulum.__dataclass_fields__}
    )
    with pytest.raises(ValueError, match="Slip law requires"):
        model.finite_slip(0.1, 0.0, 0.0, 0.2, friction, scale)


@pytest.mark.parametrize("horizon", [1, 5])
def test_unconstrained_mpc_matches_riccati_feedback(horizon):
    from duk_core.control.height_balance import HeightSample, height_mpc_command

    a, b = np.array([[0.0, 1.0], [1.0, 0.0]]), np.array([[0.0], [1.0]])
    q, r = np.diag([2.0, 0.5]), np.array([[0.3]])
    design = discrete_lqr(a, b, q, r, 0.05)
    sample = HeightSample(
        (0, 0, 0),
        np.zeros(2),
        0.0,
        design.a,
        design.b,
        np.array([-100.0]),
        np.zeros((1, 2)),
        np.zeros(1),
    )
    error = np.array([0.2, -0.1])
    command = height_mpc_command([sample] * horizon, error, q, r, design.cost, 100.0)
    # SLSQP stops on objective tolerance (1e-9), so compare control to O(sqrt(ftol)).
    assert command == pytest.approx(-(design.gain @ error)[0], abs=1e-4)
