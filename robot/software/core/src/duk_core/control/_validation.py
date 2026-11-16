"""Numerical contracts shared inside the control package."""

import numpy as np
from numpy.typing import ArrayLike


def state_space(a: ArrayLike, b: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
    """Require finite matrices for a nonempty state and input space."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if (
        a.ndim != 2
        or not a.shape[0]
        or a.shape[0] != a.shape[1]
        or b.ndim != 2
        or b.shape[0] != a.shape[0]
        or not b.shape[1]
        or not np.isfinite(a).all()
        or not np.isfinite(b).all()
    ):
        raise ValueError("Expected finite A (n, n) and B (n, m), with n and m positive")
    return a, b


def weight_matrix(
    value: ArrayLike, size: int, name: str, *, positive: bool = False
) -> np.ndarray:
    """Validate a symmetric quadratic weight, allowing roundoff in PSD weights."""
    matrix = np.asarray(value, dtype=float)
    if (
        matrix.shape != (size, size)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix, matrix.T, rtol=1e-10, atol=1e-12)
    ):
        raise ValueError(f"{name} must be a finite symmetric ({size}, {size}) matrix")
    matrix = matrix / 2 + matrix.T / 2
    eigenvalues = np.linalg.eigvalsh(matrix)
    tolerance = 1e-12 * max(1.0, np.max(abs(eigenvalues)))
    if (positive and eigenvalues[0] <= 0) or eigenvalues[0] < -tolerance:
        requirement = "positive definite" if positive else "positive semidefinite"
        raise ValueError(f"{name} must be {requirement}")
    return matrix
