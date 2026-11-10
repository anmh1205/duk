"""Force feasibility and constrained local MPC for a prescribed height path."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize

from ..modeling.rolling_reduction import ReducedResponse


@dataclass(frozen=True)
class HeightSample:
    """Nominal state, discrete linearization and affine constraints for one stage."""

    motion: tuple[float, float, float]
    nominal: np.ndarray
    nominal_effort: float
    a: np.ndarray
    b: np.ndarray
    constraint_value: np.ndarray
    constraint_state: np.ndarray
    constraint_effort: np.ndarray


def height_force_constraints(
    response: ReducedResponse, hip_bound: float, friction: float
) -> np.ndarray:
    """Return residuals <= 0 for hip torque, per-wheel traction and support."""
    traction_budget = friction * response.normal_force / 2
    return np.r_[
        response.hip_effort - hip_bound,
        -response.hip_effort - hip_bound,
        response.tangent_force - traction_budget,
        -response.tangent_force - traction_budget,
        -response.normal_force,
    ]


def height_torque_interval(
    base: np.ndarray, slope: np.ndarray, motor_bound: float
) -> tuple[float, float] | None:
    """Intersect affine constraint residuals with symmetric wheel-moment limits."""
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


def height_mpc_command(
    samples: Sequence[HeightSample],
    deviation: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    terminal: np.ndarray,
    motor_bound: float,
) -> float | None:
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
