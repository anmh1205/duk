"""Simulation harness for sagittal contact and repeatable sensor comparisons."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from .contact_dynamics import CompliantContact, compliant_planar, rigid_rolling


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


def simulate_planar(
    model,
    initial,
    times,
    input_at,
    friction,
    *,
    mode,
    contact_law: CompliantContact | None = None,
    max_step,
    rtol,
    atol,
):
    """Run an explicit contact model. Rigid invalidity terminates the selected run."""
    times, friction = np.asarray(times, dtype=float), np.asarray(friction, dtype=float)
    if (
        friction.shape != (2,)
        or not np.isfinite(friction).all()
        or np.any(friction < 0)
    ):
        raise ValueError("Friction must be two finite nonnegative coefficients")
    if times.ndim != 1 or times.size < 2 or np.any(np.diff(times) <= 0):
        raise ValueError("Simulation times must increase")
    if mode not in {"rolling", "compliant", "flight"}:
        raise ValueError("Unknown sagittal contact model")
    if mode == "compliant" and contact_law is None:
        raise ValueError("Compliant contact requires an explicit force law")

    def calculate(time, state):
        q, v = state[:7], state[7:]
        dynamics = model.evaluate(q, v)
        u = input_at(time, state)
        if mode == "compliant":
            result = compliant_planar(dynamics, v, u, friction, contact_law)
        else:
            result = rigid_rolling(
                dynamics, u, friction, active=(mode == "rolling",) * 2
            )
        return dynamics, result, u

    if mode == "rolling":
        d0, _, _ = calculate(times[0], np.asarray(initial))
        if np.max(abs(d0.gaps)) > atol or np.max(abs(d0.contact @ initial[7:])) > atol:
            raise ValueError(
                "Rolling simulation requires consistent initial gaps and velocities"
            )

    def rhs(time, state):
        _, result, _ = calculate(time, state)
        return np.r_[state[7:], result.acceleration]

    def guard(time, state):
        dynamics, result, _ = calculate(time, state)
        if mode == "flight":
            return np.min(dynamics.gaps)
        normal, tangent = np.split(result.forces, 2)
        return np.min(np.r_[normal, friction * normal - abs(tangent)])

    guard.terminal = True
    guard.direction = -1
    if mode != "compliant" and guard(times[0], np.asarray(initial)) < 0:
        raise ValueError("The selected rigid mode is invalid at the initial state")
    solution = solve_ivp(
        rhs,
        (times[0], times[-1]),
        initial,
        t_eval=times,
        max_step=max_step,
        rtol=rtol,
        atol=atol,
        method="Radau",
        events=None if mode == "compliant" else guard,
    )
    if not solution.success:
        raise RuntimeError(solution.message)

    def sample(time, state):
        dynamics, result, u = calculate(time, state)
        v = state[7:]
        a = result.acceleration
        slip = dynamics.contact[2:] @ v
        axle_speed = slip - model.radius * v[5:]
        axle_accel = (
            dynamics.contact[2:] @ a + dynamics.contact_bias[2:] - model.radius * a[5:]
        )
        energy = 0.5 * v @ dynamics.mass @ v + dynamics.potential
        # G = m*g*dz_com/dq, so finite differences of G give the CoM convective term.
        g = model.parameters["g_m_s2"]
        dt = 1e-5
        gravity_forward = model.evaluate(state[:7] + dt * v, v).gravity
        gravity_backward = model.evaluate(state[:7] - dt * v, v).gravity
        com_accel = dynamics.gravity @ a + (gravity_forward - gravity_backward) @ v / (
            2 * dt
        )
        com_accel /= g * model.masses.sum()
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

    values = [sample(time, state) for time, state in zip(solution.t, solution.y.T)]
    return PlanarTrace(
        solution.t, solution.y.T, *[np.asarray(column) for column in zip(*values)]
    )
