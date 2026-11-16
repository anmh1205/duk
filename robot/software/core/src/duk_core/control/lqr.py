"""Discrete LQR design and local state-space linearization."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

from ._validation import state_space, weight_matrix


@dataclass(frozen=True)
class LQRDesign:
    a: np.ndarray
    b: np.ndarray
    cost: np.ndarray
    gain: np.ndarray
    riccati_residual: float
    spectral_radius: float


def zoh(a: ArrayLike, b: ArrayLike, period: float) -> tuple[np.ndarray, np.ndarray]:
    """Discretize finite state-space matrices with a positive zero-order hold."""
    a, b = state_space(a, b)
    if not np.isfinite(period) or period <= 0:
        raise ValueError("Sample period must be finite and positive")
    ad, bd, *_ = cont2discrete(
        (a, b, np.eye(len(a)), np.zeros_like(b)), period, method="zoh"
    )
    return ad, bd


def discrete_lqr(
    a: ArrayLike, b: ArrayLike, q: ArrayLike, r: ArrayLike, period: float
) -> LQRDesign:
    """Design LQR with symmetric PSD state and PD effort costs."""
    ad, bd = zoh(a, b, period)
    q = weight_matrix(q, len(ad), "Q")
    r = weight_matrix(r, bd.shape[1], "R", positive=True)
    p = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    residual = ad.T @ p @ (ad - bd @ k) - p + q
    rho = np.max(abs(np.linalg.eigvals(ad - bd @ k)))
    return LQRDesign(
        ad,
        bd,
        p,
        k,
        float(np.linalg.norm(residual) / (np.linalg.norm(p) or 1.0)),
        float(rho),
    )


def bryson_weights(
    state_scales: ArrayLike, effort_scale: float
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a nonempty state-scale vector and scalar effort scale to costs."""
    scales = np.asarray(state_scales, dtype=float)
    if (
        scales.ndim != 1
        or not scales.size
        or not np.isfinite(scales).all()
        or np.any(scales <= 0)
        or not np.isfinite(effort_scale)
        or effort_scale <= 0
    ):
        raise ValueError("State and effort scales must be finite and positive")
    with np.errstate(over="ignore", under="ignore", divide="ignore"):
        state_cost = np.square(np.reciprocal(scales))
        effort_cost = np.square(np.reciprocal(float(effort_scale)))
    if (
        not np.isfinite(state_cost).all()
        or np.any(state_cost <= 0)
        or not np.isfinite(effort_cost)
        or effort_cost <= 0
    ):
        raise ValueError("Scales produce unrepresentable quadratic weights")
    return np.diag(state_cost), np.array([[effort_cost]])


def linearize(
    function: Callable[[np.ndarray, float], ArrayLike],
    state: ArrayLike,
    effort: float,
    step: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Central differences of an independent state derivative with scalar input."""
    state = np.asarray(state, dtype=float)
    if (
        state.ndim != 1
        or not state.size
        or not np.isfinite(state).all()
        or not np.isfinite([effort, step]).all()
        or step <= 0
    ):
        raise ValueError(
            "Linearization requires a finite state/input and positive step"
        )
    output_shape = None

    def evaluate(x, u):
        nonlocal output_shape
        # A callback may reuse an output buffer. Preserve each evaluation before
        # the next callback can overwrite it, otherwise central differences vanish.
        result = np.array(function(x, u), dtype=float, copy=True)
        if output_shape is None:
            output_shape = result.shape
        if (
            result.ndim != 1
            or not result.size
            or result.shape != output_shape
            or not np.isfinite(result).all()
        ):
            raise ValueError(
                "Linearized function must return consistent finite vectors"
            )
        return result

    perturbations = np.eye(len(state)) * step
    a = np.column_stack(
        [
            (evaluate(state + dx, effort) - evaluate(state - dx, effort)) / (2 * step)
            for dx in perturbations
        ]
    )
    b = (evaluate(state, effort + step) - evaluate(state, effort - step))[:, None] / (
        2 * step
    )
    return a, b


def finite_horizon_lqr(
    stages: Sequence[tuple[ArrayLike, ArrayLike]],
    q: ArrayLike,
    r: ArrayLike,
    terminal: ArrayLike,
) -> list[np.ndarray]:
    """Backward Riccati pass on a finite sequence of discrete (A, B) pairs."""
    if not stages:
        return []
    stages = [state_space(a, b) for a, b in stages]
    n, m = stages[0][1].shape
    if any(a.shape != (n, n) or b.shape != (n, m) for a, b in stages):
        raise ValueError("All LQR stages must have the same state and input dimensions")
    q = weight_matrix(q, n, "Q")
    r = weight_matrix(r, m, "R", positive=True)
    p = weight_matrix(terminal, n, "Terminal cost")
    gains = []
    for a, b in reversed(stages):
        gain = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        p = q + gain.T @ r @ gain + (a - b @ gain).T @ p @ (a - b @ gain)
        gains.append(gain)
    return list(reversed(gains))
