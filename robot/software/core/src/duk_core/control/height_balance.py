"""Compare sagittal feedback policies along the same prescribed hip trajectory."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize

from .lqr import discrete_lqr, finite_horizon_lqr, linearize, zoh
from ..modeling.rolling_reduction import PrescribedSagittal
from ..modeling.wheeled_pendulum import WheeledPendulum


@dataclass(frozen=True)
class HipTrajectory:
    initial: float
    final: float
    duration: float

    def __post_init__(self):
        if (
            not np.isfinite([self.initial, self.final, self.duration]).all()
            or self.duration <= 0
        ):
            raise ValueError(
                "Hip trajectory requires finite endpoints and positive duration"
            )

    def evaluate(self, time):
        t = np.clip(time / self.duration, 0.0, 1.0)
        delta = self.final - self.initial
        position = self.initial + delta * t**3 * (10 - 15 * t + 6 * t**2)
        velocity = delta / self.duration * 30 * t**2 * (1 - t) ** 2
        acceleration = delta / self.duration**2 * 60 * t * (1 - t) * (1 - 2 * t)
        return position, velocity, acceleration


@dataclass(frozen=True)
class HeightSample:
    motion: tuple
    nominal: np.ndarray
    nominal_effort: float
    a: np.ndarray
    b: np.ndarray
    constraint_value: np.ndarray
    constraint_state: np.ndarray
    constraint_effort: np.ndarray


@dataclass(frozen=True)
class HeightTrace:
    time: np.ndarray
    state: np.ndarray
    requested: np.ndarray
    applied: np.ndarray
    hip_effort: np.ndarray
    grip_usage: np.ndarray
    lean: np.ndarray
    height: np.ndarray
    compute_time: np.ndarray
    status: tuple[str, ...]


def _force_constraints(response, hip_bound, friction):
    traction_budget = friction * response.normal_force / 2
    return np.r_[
        response.hip_effort - hip_bound,
        -response.hip_effort - hip_bound,
        response.tangent_force - traction_budget,
        -response.tangent_force - traction_budget,
        -response.normal_force,
    ]


def _interval(base, slope, motor_bound):
    lower, upper = -motor_bound, motor_bound
    for value, derivative in zip(base, slope):
        if derivative > 1e-10:
            upper = min(upper, -value / derivative)
        elif derivative < -1e-10:
            lower = max(lower, -value / derivative)
        elif value > 1e-9:
            return None
    return (lower, upper) if lower <= upper else None


def _mpc(samples, deviation, q, r, terminal, motor_bound):
    """Condense local linear dynamics and contact constraints to a torque QP."""
    count = len(samples)
    shifted = deviation
    influence = np.zeros((len(deviation), count))
    hessian = np.eye(count) * r[0, 0]
    gradient = np.zeros(count)
    limits, rows, lower, upper = [], [], [], []
    for index, sample in enumerate(samples):
        row = sample.constraint_state @ influence
        row[:, index] += sample.constraint_effort
        rows.append(row)
        limits.append(-sample.constraint_value - sample.constraint_state @ shifted)
        lower.append(-motor_bound - sample.nominal_effort)
        upper.append(motor_bound - sample.nominal_effort)
        hessian += influence.T @ q @ influence
        gradient += influence.T @ q @ shifted
        shifted = sample.a @ shifted
        influence = sample.a @ influence
        influence[:, index] += sample.b[:, 0]
    hessian += influence.T @ terminal @ influence
    gradient += influence.T @ terminal @ shifted

    result = minimize(
        lambda u: 0.5 * u @ hessian @ u + gradient @ u,
        np.clip(np.zeros(count), lower, upper),
        jac=lambda u: hessian @ u + gradient,
        method="SLSQP",
        bounds=Bounds(lower, upper),
        constraints=LinearConstraint(np.vstack(rows), -np.inf, np.concatenate(limits)),
        options={"ftol": 1e-9, "maxiter": 50},
    )
    if (
        not result.success
        or np.max(np.vstack(rows) @ result.x - np.concatenate(limits)) > 1e-6
    ):
        return None
    return float(result.x[0])


def compare_height_policies(
    model,
    trajectory,
    motor,
    *,
    period,
    duration,
    friction,
    motor_bound,
    q_weight,
    r_weight,
    initial_lean,
    horizon,
):
    """Evaluate fixed, scheduled, TV-LQR and constrained MPC in a common plant.

    The plant follows prescribed q(t). Required hip effort and traction are
    checked even if the pose trajectory is kinematically reachable.
    """
    if (
        not np.isfinite([period, duration, friction, motor_bound]).all()
        or min(period, duration, friction, motor_bound) <= 0
        or horizon < 1
    ):
        raise ValueError("Invalid height study interval, limits or MPC horizon")
    times = np.arange(round(duration / period)) * period
    offset = model.equivalent_upper_body(trajectory.initial).angle_from_vertical_rad
    plant = PrescribedSagittal(model, offset)

    def posture(q):
        return offset - model.equivalent_upper_body(q).angle_from_vertical_rad

    def frozen(q):
        a, b = WheeledPendulum.from_biped(model, q).linearization()
        return discrete_lqr(a, b, q_weight, r_weight, period)

    def bounds(state, motion):
        hip_bound = motor.joint_limit(motion[1])
        response0 = plant.evaluate(state, 0.0, motion)
        response1 = plant.evaluate(state, 1.0, motion)
        base = _force_constraints(response0, hip_bound, friction)
        slope = _force_constraints(response1, hip_bound, friction) - base
        return _interval(base, slope, motor_bound), response0, response1

    def advance(state, effort, time):
        first = plant.evaluate(state, effort, trajectory.evaluate(time)).derivative
        middle = plant.evaluate(
            state + first * period / 2, effort, trajectory.evaluate(time + period / 2)
        ).derivative
        return state + period * middle

    samples = []
    nominal = np.zeros(4)
    for time in times:
        motion = trajectory.evaluate(time)
        q, dq, ddq = motion
        step = 1e-4
        lean_ref = posture(q)
        first = (posture(q + step) - posture(q - step)) / (2 * step)
        second = (posture(q + step) + posture(q - step) - 2 * lean_ref) / step**2
        desired = (
            second * dq**2
            + first * ddq
            + 12 * (lean_ref - nominal[2])
            + 6 * (first * dq - nominal[3])
        )
        interval, zero, unit = bounds(nominal, motion)
        if interval is None:
            raise ValueError(f"Nominal lift infeasible at {time:.3f} s")
        inverse = (desired - zero.derivative[3]) / (
            unit.derivative[3] - zero.derivative[3]
        )
        effort = float(np.clip(inverse, *interval))
        local_a, local_b = linearize(
            lambda state, u: plant.evaluate(state, u, motion).derivative,
            nominal,
            effort,
        )
        a, b = zoh(local_a, local_b, period)
        hip_bound = motor.joint_limit(dq)

        def constraints(state, u):
            return _force_constraints(
                plant.evaluate(state, u, motion), hip_bound, friction
            )

        ca, cb = linearize(constraints, nominal, effort)
        samples.append(
            HeightSample(
                motion,
                nominal,
                effort,
                a,
                b,
                constraints(nominal, effort),
                ca,
                cb[:, 0],
            )
        )
        nominal = advance(nominal, effort, time)

    fixed = frozen(trajectory.initial).gain[0]
    grid = np.linspace(
        min(trajectory.initial, trajectory.final),
        max(trajectory.initial, trajectory.final),
        7,
    )
    scheduled = np.array([frozen(q).gain[0] for q in grid])
    terminal = frozen(trajectory.final).cost
    tv = finite_horizon_lqr(
        [(sample.a, sample.b) for sample in samples], q_weight, r_weight, terminal
    )
    traces = {}

    for policy in ("fixed", "scheduled", "tv_lqr", "mpc"):
        state = np.array([0.0, 0.0, initial_lean, 0.0])
        rows, statuses = [], []
        for index, (time, sample) in enumerate(zip(times, samples)):
            deviation = state - sample.nominal
            start = perf_counter()
            status = "within_limits"
            if policy == "fixed":
                correction = -fixed @ deviation
            elif policy == "scheduled":
                gain = np.array(
                    [
                        np.interp(sample.motion[0], grid, column)
                        for column in scheduled.T
                    ]
                )
                correction = -gain @ deviation
            elif policy == "tv_lqr":
                correction = -(tv[index] @ deviation)[0]
            else:
                correction = _mpc(
                    samples[index : index + horizon],
                    deviation,
                    q_weight,
                    r_weight,
                    terminal,
                    motor_bound,
                )
                if correction is None:
                    correction = -fixed @ deviation
                    status = "mpc_fallback"
            request = sample.nominal_effort + correction
            compute = perf_counter() - start
            interval, _, _ = bounds(state, sample.motion)
            effort = np.nan if interval is None else float(np.clip(request, *interval))
            if interval is None:
                status = "infeasible"
            elif status == "within_limits" and effort != request:
                status = "limited"
            response = plant.evaluate(
                state, 0.0 if interval is None else effort, sample.motion
            )
            lean = state[2] - posture(sample.motion[0])
            lift = lift_at(plant, state, sample.motion)
            usage = (
                2
                * np.max(abs(response.tangent_force))
                / max(1e-12, friction * response.normal_force)
            )
            rows.append(
                (
                    time,
                    state,
                    request,
                    effort,
                    response.hip_effort,
                    usage,
                    lean,
                    lift,
                    compute,
                )
            )
            statuses.append(status)
            if interval is None:
                break
            state = advance(state, effort, time)
        traces[policy] = HeightTrace(*map(np.asarray, zip(*rows)), tuple(statuses))
    return traces


def lift_at(plant, state, motion):
    """Height of the body reference origin in the symmetric rolling model."""
    from ..modeling.rolling_reduction import lift_symmetric_rolling

    q, dq, ddq = motion
    return lift_symmetric_rolling(
        plant.model.model.geometry,
        state[0],
        state[2],
        state[1],
        state[3],
        q,
        mount_xz_m=plant.mount,
        pitch_offset=plant.pitch_offset,
        q_rate=dq,
        q_acceleration=ddq,
    ).configuration[1]
