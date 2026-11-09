"""Rigid rolling checks and unilateral compliant contact for research models."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .planar_numeric import PlanarDynamics


@dataclass(frozen=True)
class CompliantContact:
    stiffness: float
    damping: float
    slip_scale: float

    def __post_init__(self):
        values = (self.stiffness, self.damping, self.slip_scale)
        if (
            not np.isfinite(values).all()
            or self.stiffness <= 0
            or self.damping < 0
            or self.slip_scale <= 0
        ):
            raise ValueError(
                "Contact requires positive stiffness/slip scale and nonnegative damping"
            )

    def forces(self, gaps, normal_speed, tangential_speed, friction):
        """Return normal and tangential forces. The final tangent axis may be 1D or 2D."""
        gaps = np.asarray(gaps)
        tangent = np.asarray(tangential_speed)
        speed = np.linalg.norm(tangent, axis=-1)
        normal = np.where(
            gaps <= 0,
            np.maximum(0, -self.stiffness * gaps - self.damping * normal_speed),
            0,
        )
        regularized = np.divide(
            np.tanh(speed / self.slip_scale),
            speed,
            out=np.zeros_like(speed),
            where=speed > 0,
        )
        return normal, -(friction * normal * regularized)[..., None] * tangent


@dataclass(frozen=True)
class PlanarContactResult:
    acceleration: np.ndarray
    forces: np.ndarray
    admissible: bool


def constrained_acceleration(mass, rhs, jacobian, target):
    """Solve the acceleration/force system for explicitly independent rows."""
    count = len(jacobian)
    if count == 0:
        return np.linalg.solve(mass, rhs), np.empty(0)
    if np.linalg.matrix_rank(jacobian) != count:
        raise ValueError(
            "Dependent contact or joint constraints require explicit reduction"
        )
    system = np.block([[mass, -jacobian.T], [jacobian, np.zeros((count, count))]])
    solution = np.linalg.solve(system, np.r_[rhs, target])
    return solution[: len(mass)], solution[len(mass) :]


def rigid_rolling(
    dynamics: PlanarDynamics,
    effort,
    friction,
    *,
    active=(True, True),
    tolerance=1e-9,
) -> PlanarContactResult:
    """Solve a selected rolling mode and report unilateral/friction feasibility."""
    active = np.asarray(active, dtype=bool)
    rows = np.r_[active, active]
    acc, multipliers = constrained_acceleration(
        dynamics.mass,
        dynamics.input @ effort - dynamics.bias,
        dynamics.contact[rows],
        -dynamics.contact_bias[rows],
    )
    forces = np.zeros(4)
    forces[rows] = multipliers
    normal, tangent = forces[:2], forces[2:]
    feasible = np.all(normal >= -tolerance) and np.all(
        abs(tangent) <= friction * normal + tolerance
    )
    return PlanarContactResult(acc, forces, bool(feasible))


def compliant_planar(
    dynamics: PlanarDynamics, velocity, effort, friction, law: CompliantContact
):
    contact_speed = dynamics.contact @ velocity
    normal, tangent = law.forces(
        dynamics.gaps, contact_speed[:2], contact_speed[2:, None], friction
    )
    forces = np.r_[normal, tangent[:, 0]]
    acc = np.linalg.solve(
        dynamics.mass,
        dynamics.input @ effort - dynamics.bias + dynamics.contact.T @ forces,
    )
    return PlanarContactResult(acc, forces, True)
