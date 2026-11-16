"""Rigid rolling checks and unilateral compliant contact for research models."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

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

    def forces(
        self,
        gaps: ArrayLike,
        normal_speed: ArrayLike,
        tangential_speed: ArrayLike,
        friction: ArrayLike,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return unilateral, dissipative forces for finite contact measurements.

        Contact dimensions broadcast; the last tangent axis has one or two
        components. Friction is finite and nonnegative. Inputs are in SI.
        """
        gaps, normal_speed, tangent, friction = (
            np.asarray(value, dtype=float)
            for value in (gaps, normal_speed, tangential_speed, friction)
        )
        if (
            tangent.ndim < 1
            or tangent.shape[-1] not in (1, 2)
            or not all(
                np.isfinite(value).all()
                for value in (gaps, normal_speed, tangent, friction)
            )
            or np.any(friction < 0)
        ):
            raise ValueError(
                "Contact requires finite measurements, nonnegative friction and 1D/2D tangents"
            )
        try:
            shape = np.broadcast_shapes(
                gaps.shape, normal_speed.shape, tangent.shape[:-1], friction.shape
            )
            gaps, normal_speed, friction = (
                np.broadcast_to(value, shape)
                for value in (gaps, normal_speed, friction)
            )
            tangent = np.broadcast_to(tangent, (*shape, tangent.shape[-1]))
        except ValueError as exc:
            raise ValueError("Contact measurement dimensions must broadcast") from exc
        speed = (
            np.abs(tangent[..., 0])
            if tangent.shape[-1] == 1
            else np.hypot(tangent[..., 0], tangent[..., 1])
        )
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
    effort: ArrayLike,
    friction: ArrayLike,
    *,
    active=(True, True),
    tolerance: float = 1e-9,
) -> PlanarContactResult:
    """Solve a selected mode with scalar/per-wheel nonnegative finite friction.

    Invalid force inputs raise ValueError; valid inputs can yield an inadmissible
    rigid mode. A zero feasibility tolerance is supported.
    """
    effort = np.asarray(effort, dtype=float)
    friction = np.asarray(friction, dtype=float)
    if effort.shape != (dynamics.input.shape[1],) or not np.isfinite(effort).all():
        raise ValueError("Effort must contain one finite value per input")
    if (
        friction.shape not in ((), (2,))
        or not np.isfinite(friction).all()
        or np.any(friction < 0)
    ):
        raise ValueError("Friction must be finite and nonnegative, scalar or per wheel")
    if not np.isfinite(tolerance) or tolerance < 0:
        raise ValueError("Force tolerance must be finite and nonnegative")
    active = np.asarray(active, dtype=bool)
    if active.shape != (2,):
        raise ValueError("Specify active contact for each wheel")
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
