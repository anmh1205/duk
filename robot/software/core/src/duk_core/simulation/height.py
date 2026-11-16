"""Compare feedback policies on the same prescribed sagittal height trajectory."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np

from ..control.height_balance import (
    HeightSample,
    height_force_constraints,
    height_mpc_command,
    height_torque_interval,
)
from ..control.lqr import discrete_lqr, finite_horizon_lqr, linearize, zoh
from ..modeling.hip_load import LinearTorqueSpeed
from ..modeling.planar_numeric import NumericPlanarBiped
from ..modeling.rolling_reduction import PrescribedSagittal
from ..modeling.trajectories import HipTrajectory
from ..modeling.wheeled_pendulum import WheeledPendulum
from ._sampled import sample_intervals


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


def _rolling_step(plant, trajectory, motor, friction, state, effort, start, end, first):
    """Reject a midpoint step if holding its effort violates a sampled constraint."""
    dt = end - start
    midpoint = state + first.derivative * dt / 2
    motion = trajectory.evaluate(start + dt / 2)
    middle = plant.evaluate(midpoint, effort, motion)
    if not np.all(
        height_force_constraints(middle, motor.joint_limit(motion[1]), friction) <= 1e-8
    ):
        return None, "midpoint_infeasible"
    candidate = state + dt * middle.derivative
    motion = trajectory.evaluate(end)
    final = plant.evaluate(candidate, effort, motion)
    if not np.all(
        height_force_constraints(final, motor.joint_limit(motion[1]), friction) <= 1e-8
    ):
        return None, "endpoint_infeasible"
    return candidate, None


def compare_height_policies(
    model: NumericPlanarBiped,
    trajectory: HipTrajectory,
    motor: LinearTorqueSpeed,
    *,
    period: float,
    duration: float,
    friction: float,
    motor_bound: float,
    q_weight: np.ndarray,
    r_weight: np.ndarray,
    initial_lean: float,
    horizon: int,
) -> dict[str, HeightTrace]:
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
        base = height_force_constraints(response0, hip_bound, friction)
        slope = height_force_constraints(response1, hip_bound, friction) - base
        return height_torque_interval(base, slope, motor_bound), response0, response1

    samples = []
    nominal = np.zeros(4)
    for time, end in zip(times, ends):
        motion = trajectory.evaluate(time)
        q, dq, ddq = motion
        lean_ref = posture(q)
        angle_first, angle_second = model.upper_body_angle_derivatives(q)
        first, second = -angle_first, -angle_second
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
            return height_force_constraints(
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
                correction = height_mpc_command(
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
