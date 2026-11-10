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


def test_projection_cannot_bypass_motor_limits_with_a_wide_interval():
    assert project_torque(9, 3, (-10, 10)).applied == 3
    assert project_torque(-9, 3, (-10, 10)).reason == "motor"
    assert project_torque(9, 3, (4, 10)).applied is None
    with pytest.raises(ValueError, match="NaN"):
        project_torque(9, 3, (np.nan, 10))


@pytest.mark.parametrize("violation", ["midpoint", "endpoint"])
def test_held_torque_step_rejects_intermediate_contact_failure(violation):
    from types import SimpleNamespace

    from duk_core.control.height_balance import _rolling_step

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
    from duk_core.control import sagittal_balance

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
