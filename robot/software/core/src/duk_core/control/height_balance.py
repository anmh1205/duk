"""Force feasibility and constrained local MPC for a prescribed height path."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize

from ..modeling.rolling_reduction import ReducedResponse
from ._validation import state_space, weight_matrix


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
    base, slope = np.asarray(base, dtype=float), np.asarray(slope, dtype=float)
    if base.ndim != 1 or slope.shape != base.shape:
        raise ValueError("Constraint base and slope must be equally sized vectors")
    if not np.isfinite(motor_bound) or motor_bound < 0:
        raise ValueError("Motor bound must be finite and nonnegative")
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
    """Return the first correction, or None when no feasible solution is found.

    Require a nonempty horizon, PSD state/terminal costs and PD effort cost.
    A stage may have no force constraints, while motor bounds still apply.
    Malformed or nonfinite problem data raises ValueError.
    """
    count = len(samples)
    deviation = np.asarray(deviation, dtype=float)
    if (
        not count
        or deviation.ndim != 1
        or not deviation.size
        or not np.isfinite(deviation).all()
        or not np.isfinite(motor_bound)
        or motor_bound < 0
    ):
        raise ValueError(
            "MPC requires a nonempty horizon, finite state and motor bound"
        )
    q = weight_matrix(q, len(deviation), "Q")
    r = weight_matrix(r, 1, "R", positive=True)
    terminal = weight_matrix(terminal, len(deviation), "Terminal cost")
    shifted = deviation
    influence = np.zeros((len(deviation), count))
    hessian = np.eye(count) * r[0, 0]
    gradient = np.zeros(count)
    limits, rows, lower, upper = [], [], [], []
    for index, sample in enumerate(samples):
        a, b = state_space(sample.a, sample.b)
        value, ca, cb = (
            np.asarray(field, dtype=float)
            for field in (
                sample.constraint_value,
                sample.constraint_state,
                sample.constraint_effort,
            )
        )
        if (
            a.shape != (len(deviation), len(deviation))
            or b.shape != (len(deviation), 1)
            or value.ndim != 1
            or ca.shape != (value.size, len(deviation))
            or cb.shape != value.shape
            or not all(np.isfinite(field).all() for field in (value, ca, cb))
            or not np.isfinite(sample.nominal_effort)
        ):
            raise ValueError(
                f"MPC stage {index} has invalid dynamics or constraint data"
            )
        row = ca @ influence
        row[:, index] += cb
        rows.append(row)
        limits.append(-value - ca @ shifted)
        lower.append(-motor_bound - sample.nominal_effort)
        upper.append(motor_bound - sample.nominal_effort)
        hessian += influence.T @ q @ influence
        gradient += influence.T @ q @ shifted
        shifted = a @ shifted
        influence = a @ influence
        influence[:, index] += b[:, 0]
    hessian += influence.T @ terminal @ influence
    gradient += influence.T @ terminal @ shifted

    rows, limits = np.vstack(rows), np.concatenate(limits)
    variable = np.any(rows != 0, axis=1)
    if np.any(limits[~variable] < -1e-9):
        return None
    constraints = (
        [LinearConstraint(rows[variable], -np.inf, limits[variable])]
        if variable.any()
        else []
    )
    result = minimize(
        lambda u: 0.5 * u @ hessian @ u + gradient @ u,
        np.clip(np.zeros(count), lower, upper),
        jac=lambda u: hessian @ u + gradient,
        method="SLSQP",
        bounds=Bounds(lower, upper),
        constraints=constraints,
        options={"ftol": 1e-9, "maxiter": 50},
    )
    if (
        not result.success
        or not np.isfinite(result.x).all()
        or np.max(rows @ result.x - limits, initial=0.0) > 1e-6
        or np.any(result.x < np.asarray(lower) - 1e-8)
        or np.any(result.x > np.asarray(upper) + 1e-8)
    ):
        return None
    return float(result.x[0])
