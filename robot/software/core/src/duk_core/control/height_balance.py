"""Compare sagittal feedback policies along the same prescribed hip trajectory."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize

from ._sampled import sample_intervals
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
    terminal_time: float
    terminal_lean: float


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
    if not np.isfinite(base).all() or not np.isfinite(slope).all():
        return None
    lower, upper = -motor_bound, motor_bound
    for value, derivative in zip(base, slope):
        if derivative > 1e-10:
            upper = min(upper, -value / derivative)
        elif derivative < -1e-10:
            lower = max(lower, -value / derivative)
        elif value > 1e-9:
            return None
    return (lower, upper) if lower <= upper else None


def _rolling_step(plant, trajectory, motor, friction, state, effort, start, end, first):
    """Reject a midpoint step if holding its effort violates a sampled constraint."""
    dt = end - start
    midpoint = state + first.derivative * dt / 2
    motion = trajectory.evaluate(start + dt / 2)
    middle = plant.evaluate(midpoint, effort, motion)
    if not np.all(
        _force_constraints(middle, motor.joint_limit(motion[1]), friction) <= 1e-8
    ):
        return None, "midpoint_infeasible"
    candidate = state + dt * middle.derivative
    motion = trajectory.evaluate(end)
    final = plant.evaluate(candidate, effort, motion)
    if not np.all(
        _force_constraints(final, motor.joint_limit(motion[1]), friction) <= 1e-8
    ):
        return None, "endpoint_infeasible"
    return candidate, None


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

    rows, limits = np.vstack(rows), np.concatenate(limits)
    result = minimize(
        lambda u: 0.5 * u @ hessian @ u + gradient @ u,
        np.clip(np.zeros(count), lower, upper),
        jac=lambda u: hessian @ u + gradient,
        method="SLSQP",
        bounds=Bounds(lower, upper),
        constraints=LinearConstraint(rows, -np.inf, limits),
        options={"ftol": 1e-9, "maxiter": 50},
    )
    if (
        not result.success
        or not np.isfinite(result.x).all()
        or np.max(rows @ result.x - limits) > 1e-6
        or np.any(result.x < np.asarray(lower) - 1e-8)
        or np.any(result.x > np.asarray(upper) + 1e-8)
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
        or not isinstance(horizon, (int, np.integer))
        or horizon < 1
    ):
        raise ValueError("Invalid height study interval, limits or MPC horizon")
    times, ends = sample_intervals(duration, period)
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

    samples = []
    nominal = np.zeros(4)
    for time, end in zip(times, ends):
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
        a, b = zoh(local_a, local_b, end - time)
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
        start_response = plant.evaluate(nominal, effort, motion)
        nominal, rejected = _rolling_step(
            plant,
            trajectory,
            motor,
            friction,
            nominal,
            effort,
            time,
            end,
            start_response,
        )
        if rejected is not None:
            raise ValueError(f"Nominal lift at {time:.3f} s: {rejected}")

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
        terminal_time = 0.0
        for index, (time, end, sample) in enumerate(zip(times, ends, samples)):
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
            next_state, rejected = None, None
            if interval is not None:
                response = plant.evaluate(state, effort, sample.motion)
                next_state, rejected = _rolling_step(
                    plant,
                    trajectory,
                    motor,
                    friction,
                    state,
                    effort,
                    time,
                    end,
                    response,
                )
                if rejected is not None:
                    status, effort = rejected, np.nan
            lean = state[2] - posture(sample.motion[0])
            lift = lift_at(plant, state, sample.motion)
            usage, hip_effort = np.nan, np.full(2, np.nan)
            if next_state is not None:
                usage = (
                    2
                    * np.max(abs(response.tangent_force))
                    / max(1e-12, friction * response.normal_force)
                )
                hip_effort = response.hip_effort
            rows.append(
                (
                    time,
                    state,
                    request,
                    effort,
                    hip_effort,
                    usage,
                    lean,
                    lift,
                    compute,
                )
            )
            statuses.append(status)
            if next_state is None:
                break
            state, terminal_time = next_state, end
        terminal_lean = state[2] - posture(trajectory.evaluate(terminal_time)[0])
        traces[policy] = HeightTrace(
            *map(np.asarray, zip(*rows)), tuple(statuses), terminal_time, terminal_lean
        )
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
