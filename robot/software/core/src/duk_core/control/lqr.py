"""Discrete LQR design and local state-space linearization."""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


@dataclass(frozen=True)
class LQRDesign:
    a: np.ndarray
    b: np.ndarray
    cost: np.ndarray
    gain: np.ndarray
    riccati_residual: float
    spectral_radius: float


def zoh(a, b, period):
    if not np.isfinite(period) or period <= 0:
        raise ValueError("Sample period must be finite and positive")
    ad, bd, *_ = cont2discrete(
        (a, b, np.eye(len(a)), np.zeros_like(b)), period, method="zoh"
    )
    return ad, bd


def discrete_lqr(a, b, q, r, period):
    ad, bd = zoh(a, b, period)
    p = solve_discrete_are(ad, bd, q, r)
    k = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    residual = ad.T @ p @ (ad - bd @ k) - p + q
    rho = np.max(abs(np.linalg.eigvals(ad - bd @ k)))
    return LQRDesign(
        ad, bd, p, k, float(np.linalg.norm(residual) / np.linalg.norm(p)), float(rho)
    )


def bryson_weights(state_scales, effort_scale):
    scales = np.asarray(state_scales, dtype=float)
    if (
        not np.isfinite(scales).all()
        or np.any(scales <= 0)
        or not np.isfinite(effort_scale)
        or effort_scale <= 0
    ):
        raise ValueError("State and effort scales must be finite and positive")
    return np.diag(scales**-2), np.array([[effort_scale**-2]])


def linearize(function, state, effort, step=1e-6):
    """Central differences of an independent state derivative with scalar input."""
    perturbations = np.eye(len(state)) * step
    a = np.column_stack(
        [
            (function(state + dx, effort) - function(state - dx, effort)) / (2 * step)
            for dx in perturbations
        ]
    )
    b = (function(state, effort + step) - function(state, effort - step))[:, None] / (
        2 * step
    )
    return a, b


def finite_horizon_lqr(stages, q, r, terminal):
    """Backward Riccati pass on a finite sequence of discrete (A, B) pairs."""
    p = terminal
    gains = []
    for a, b in reversed(stages):
        gain = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        p = q + gain.T @ r @ gain + (a - b @ gain).T @ p @ (a - b @ gain)
        gains.append(gain)
    return list(reversed(gains))
