"""Simulation harness for sagittal contact and repeatable sensor comparisons."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from ..modeling.contact_dynamics import (
    CompliantContact,
    compliant_planar,
    rigid_rolling,
)
from ..modeling.planar_numeric import NumericPlanarBiped


@dataclass(frozen=True)
class PlanarTrace:
    time: np.ndarray
    state: np.ndarray
    acceleration: np.ndarray
    contact_forces: np.ndarray
    applied: np.ndarray
    rolling_feasible: np.ndarray
    slip: np.ndarray
    axle_speed: np.ndarray
    axle_acceleration: np.ndarray
    energy: np.ndarray
    com_vertical_acceleration: np.ndarray
    relative_spin: np.ndarray
    carrier_rate: np.ndarray
    termination: str
    friction: np.ndarray

    @property
    def force_admissible(self) -> np.ndarray:
        """Force feasibility for the selected law; this does not establish sticking.

        rolling_feasible is retained as the original field name for callers.
        A compliant force can be admissible while the wheel is slipping.
        """
        return self.rolling_feasible


def simulate_planar(
    model: NumericPlanarBiped,
    initial: np.ndarray,
    times: np.ndarray,
    input_at: Callable[[float, np.ndarray], np.ndarray],
    friction: np.ndarray,
    *,
    mode: str,
    contact_law: CompliantContact | None = None,
    max_step: float,
    rtol: float,
    atol: float,
    force_atol: float = 1e-9,
    friction_change_s: float | None = None,
    friction_after: np.ndarray | None = None,
) -> PlanarTrace:
    """Retain guard samples and apply an optional friction step at its exact time.

    The continuous state is shared at the switch. Its recorded force uses the
    new left/right coefficients, including when the switch invalidates rolling.
    """
    times, friction = np.asarray(times, dtype=float), np.asarray(friction, dtype=float)
    initial = np.asarray(initial, dtype=float)
    if initial.shape != (14,) or not np.isfinite(initial).all():
        raise ValueError("Expected seven finite coordinates and seven velocities")
    after = (
        friction if friction_after is None else np.asarray(friction_after, dtype=float)
    )
    if any(
        pair.shape != (2,) or not np.isfinite(pair).all() or np.any(pair < 0)
        for pair in (friction, after)
    ):
        raise ValueError("Friction must be two finite nonnegative coefficients")
    if (friction_change_s is None) != (friction_after is None):
        raise ValueError("Specify both the friction switch time and its coefficients")
    if friction_change_s is not None and (
        not np.isfinite(friction_change_s) or friction_change_s < 0
    ):
        raise ValueError("Friction switch time must be finite and nonnegative")
    if (
        times.ndim != 1
        or times.size < 2
        or not np.isfinite(times).all()
        or np.any(np.diff(times) <= 0)
    ):
        raise ValueError("Simulation times must increase")
    if not np.isfinite(force_atol) or force_atol <= 0:
        raise ValueError("Force tolerance must be finite and positive")
    if mode not in {"rolling", "compliant", "flight"}:
        raise ValueError("Unknown sagittal contact model")
    if mode == "compliant" and contact_law is None:
        raise ValueError("Compliant contact requires an explicit force law")

    def grip_at(time):
        return (
            after
            if friction_change_s is not None and time >= friction_change_s
            else friction
        )

    def calculate(time, state, grip):
        q, v = state[:7], state[7:]
        dynamics = model.evaluate(q, v)
        u = input_at(time, state)
        if mode == "compliant":
            result = compliant_planar(dynamics, v, u, grip, contact_law)
        else:
            result = rigid_rolling(
                dynamics,
                u,
                grip,
                active=(mode == "rolling",) * 2,
                tolerance=force_atol,
            )
        return dynamics, result, u

    if mode == "rolling":
        d0, _, _ = calculate(times[0], initial, grip_at(times[0]))
        if np.max(abs(d0.gaps)) > atol or np.max(abs(d0.contact @ initial[7:])) > atol:
            raise ValueError(
                "Rolling simulation requires consistent initial gaps and velocities"
            )

    def rhs(time, state, grip):
        _, result, _ = calculate(time, state, grip)
        return np.r_[state[7:], result.acceleration]

    def guard(time, state, grip):
        dynamics, result, _ = calculate(time, state, grip)
        if mode == "flight":
            gap = np.min(dynamics.gaps)
            # A consistent initial pose may straddle zero at floating-point
            # precision. Keep the exact gap for every later root evaluation.
            return 0.0 if time == times[0] and abs(gap) <= atol else gap
        normal, tangent = np.split(result.forces, 2)
        # A force on the admissible boundary, including zero tangential force
        # at zero friction, can persist without a mode transition.
        return np.min(np.r_[normal, grip * normal - abs(tangent)]) + force_atol

    guard.terminal = True
    guard.direction = -1
    sampled_times, sampled_states = [times[0]], [initial]
    termination = "completed"
    boundaries = [times[0], times[-1]]
    if friction_change_s is not None and times[0] < friction_change_s < times[-1]:
        boundaries.insert(1, friction_change_s)
    for i, start in enumerate(boundaries):
        grip = grip_at(start)
        if mode != "compliant" and guard(start, sampled_states[-1], grip) < 0:
            if mode == "rolling" and start == friction_change_s:
                termination = "rolling_limit"
                break
            if i == 0:
                raise ValueError(
                    "The selected rigid mode is invalid at the initial state"
                )
        if i + 1 == len(boundaries):
            break
        end = boundaries[i + 1]
        grid = np.r_[start, times[(times > start) & (times < end)], end]
        solution = solve_ivp(
            rhs,
            (start, end),
            sampled_states[-1],
            args=(grip,),
            t_eval=grid,
            max_step=max_step,
            rtol=rtol,
            atol=atol,
            method="Radau",
            events=None if mode == "compliant" else guard,
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        sampled_times.extend(solution.t[1:])
        sampled_states.extend(solution.y.T[1:])
        if solution.status == 1:
            termination = "touchdown" if mode == "flight" else "rolling_limit"
            event_time = solution.t_events[0][0]
            if event_time > sampled_times[-1]:
                sampled_times.append(event_time)
                sampled_states.append(solution.y_events[0][0])
            break

    def sample(time, state):
        dynamics, result, u = calculate(time, state, grip_at(time))
        v = state[7:]
        a = result.acceleration
        slip = dynamics.contact[2:] @ v
        axle_speed = slip - model.radius * v[5:]
        axle_accel = (
            dynamics.contact[2:] @ a + dynamics.contact_bias[2:] - model.radius * a[5:]
        )
        energy = 0.5 * v @ dynamics.mass @ v + dynamics.potential
        com_accel = model.center_of_mass_acceleration(state[:7], v, a)[1]
        spin = dynamics.input[:, 2:].T @ v
        return (
            a,
            result.forces,
            u,
            result.admissible,
            slip,
            axle_speed,
            axle_accel,
            energy,
            com_accel,
            spin,
            v[5:] - v[2] - spin,
        )

    values = [sample(time, state) for time, state in zip(sampled_times, sampled_states)]
    return PlanarTrace(
        np.asarray(sampled_times),
        np.asarray(sampled_states),
        *map(np.asarray, zip(*values)),
        termination,
        np.array([grip_at(time) for time in sampled_times]),
    )
