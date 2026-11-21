"""Constrained inverse dynamics for the spatial research model."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from time import perf_counter
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike
from scipy.optimize import LinearConstraint, minimize

from ..modeling.spatial_biped import SpatialDynamics, SpatialState


@dataclass(frozen=True)
class AccelerationTask:
    name: str
    row: np.ndarray
    target: float
    scale: float


@dataclass(frozen=True)
class Allocation:
    status: str
    solution: np.ndarray | None
    residual: float
    compute_seconds: float
    active_constraints: tuple[str, ...] = ()

    @property
    def effort(self):
        return None if self.solution is None else self.solution[10:14]


def _physical_constraints(
    terms,
    state,
    modes,
    friction,
    effort_bounds,
    hip_limits,
    hip_speed_limit,
    lookahead,
    velocity_stabilization,
):
    contact = terms.kinematics.contact
    contact_speed = contact @ state.velocity
    equations = [
        np.column_stack((terms.mass, -terms.actuation, -contact.reshape(6, 10).T))
    ]
    targets = [-terms.bias]
    inequalities = []
    rolling = [side for side, mode in enumerate(modes) if mode == "rolling"]
    forward = terms.kinematics.forward
    lateral = np.array([-forward[1], forward[0], 0.0])
    vertical = np.array([0.0, 0.0, 1.0])
    correction = terms.contact_bias + velocity_stabilization * contact_speed
    if rolling:
        # The common lateral row avoids enforcing two nearly coincident
        # lateral rows as independent constraints on compliant wheels.
        lateral_row = np.zeros((1, 20))
        lateral_row[0, :10] = lateral @ contact[rolling].mean(axis=0)
        equations.append(lateral_row)
        targets.append(np.array([-lateral @ correction[rolling].mean(axis=0)]))
    lower, upper = np.full(20, -np.inf), np.full(20, np.inf)
    lower[10:14], upper[10:14] = -effort_bounds, effort_bounds
    hips, hip_rates = state.joints[:2], state.velocity[6:8]
    hip_base = hips + lookahead * hip_rates
    lower[6:8] = 2 * (hip_limits[0] - hip_base) / lookahead**2
    upper[6:8] = 2 * (hip_limits[1] - hip_base) / lookahead**2
    # q(t) = q + v*t + a*t²/2 can exceed travel and return before lookahead.
    # The tight bound occurs at t = 2*distance/abs(v) if that time is inside
    # the interval. Its acceleration is the braking-distance bound v²/(2*d).
    for side, (angle, rate) in enumerate(zip(hips, hip_rates)):
        index = 6 + side
        if not hip_limits[0] <= angle <= hip_limits[1] or abs(rate) > hip_speed_limit:
            lower[index], upper[index] = np.inf, -np.inf
            continue
        distance = hip_limits[1] - angle if rate > 0 else angle - hip_limits[0]
        if 2 * distance < abs(rate) * lookahead:
            if distance == 0:
                lower[index], upper[index] = np.inf, -np.inf
            elif rate > 0:
                upper[index] = min(upper[index], -(rate**2) / (2 * distance))
            else:
                lower[index] = max(lower[index], rate**2 / (2 * distance))
    lower[6:8] = np.maximum(lower[6:8], (-hip_speed_limit - hip_rates) / lookahead)
    upper[6:8] = np.minimum(upper[6:8], (hip_speed_limit - hip_rates) / lookahead)

    for side, mode in enumerate(modes):
        index = 14 + 3 * side
        if mode == "flight":
            lower[index : index + 3] = upper[index : index + 3] = 0.0
            continue
        lower[index + 2] = 0.0
        axes = np.stack((forward, vertical)) if mode == "rolling" else vertical[None, :]
        matrix = np.zeros((len(axes), 20))
        matrix[:, :10] = axes @ contact[side]
        equations.append(matrix)
        targets.append(-axes @ correction[side])
        if mode == "sliding":
            # Linear, dissipative force policy using the estimated direction of slip.
            tangent = contact_speed[side, :2]
            norm = np.sum(abs(tangent))
            ratio = friction[side] * tangent / norm if norm > 1e-9 else np.zeros(2)
            rows = np.zeros((2, 20))
            rows[:, index : index + 2] = np.eye(2)
            rows[:, index + 2] = ratio
            equations.append(rows)
            targets.append(np.zeros(2))
        for fx, fy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            row = np.zeros(20)
            row[index : index + 3] = [fx, fy, -friction[side]]
            inequalities.append(row)
    return (
        np.vstack(equations),
        np.concatenate(targets),
        np.array(inequalities).reshape(-1, 20),
        lower,
        upper,
    )


def _equality_coordinates(matrix, target, lower, upper):
    """Parameterize all equalities, including fixed variables, as x = origin + Z y."""
    fixed = lower == upper
    matrix = np.vstack((matrix, np.eye(len(lower))[fixed]))
    target = np.r_[target, lower[fixed]]
    # Row normalization keeps force/moment units and task scales from deciding rank.
    norms = np.linalg.norm(matrix, axis=1)
    norms = np.where(norms > 0, norms, 1.0)
    left, singular, right = np.linalg.svd(matrix / norms[:, None], full_matrices=True)
    rank = np.count_nonzero(singular > 1e-10 * singular[0])
    origin = right[:rank].T @ ((left[:, :rank].T @ (target / norms)) / singular[:rank])
    return origin, right[rank:].T


def _constraint_residual(solution, eq, rhs, inequalities, lower, upper):
    return float(
        max(
            np.max(abs(eq @ solution - rhs)),
            np.max(inequalities @ solution, initial=0.0),
            np.max(lower - solution),
            np.max(solution - upper),
        )
    )


def _minimize_task(eq, rhs, inequalities, lower, upper, task, desired, penalty, guess):
    """Optimize only free coordinates; return a candidate for full verification."""
    origin, basis = _equality_coordinates(eq, rhs, lower, upper)
    fixed = lower == upper
    equality_error = max(
        np.max(abs(eq @ origin - rhs)),
        np.max(abs(origin[fixed] - lower[fixed]), initial=0.0),
    )
    if not np.isfinite(equality_error) or equality_error > 2e-6:
        return False, origin
    if not basis.shape[1]:
        return True, origin

    # Fixed bounds already belong to the equality system. Remaining bounds and
    # friction faces become linear inequalities in the reduced coordinates.
    bounded = (~fixed) & (np.isfinite(lower) | np.isfinite(upper))
    rows = np.vstack((inequalities @ basis, basis[bounded]))
    lo = np.r_[np.full(len(inequalities), -np.inf), lower[bounded] - origin[bounded]]
    hi = np.r_[-inequalities @ origin, upper[bounded] - origin[bounded]]
    variable = np.linalg.norm(rows, axis=1) > 1e-12
    if np.any(lo[~variable] > 2e-6) or np.any(hi[~variable] < -2e-6):
        return False, origin
    reduced = task @ basis
    hessian = reduced.T @ reduced + penalty * np.eye(basis.shape[1])
    gradient = reduced.T @ (task @ origin - desired) + penalty * basis.T @ origin
    constraints = (
        [LinearConstraint(rows[variable], lo[variable], hi[variable])]
        if np.any(variable)
        else []
    )
    result = minimize(
        lambda y: 0.5 * y @ hessian @ y + gradient @ y,
        basis.T @ (guess - origin),
        jac=lambda y: hessian @ y + gradient,
        method="SLSQP",
        constraints=constraints,
        options={"ftol": 1e-9, "maxiter": 60},
    )
    return result.success, origin + basis @ result.x


def allocate(
    terms: SpatialDynamics,
    state: SpatialState,
    tasks: Sequence[AccelerationTask],
    modes: Sequence[str],
    *,
    friction: ArrayLike,
    effort_bounds: ArrayLike,
    hip_limits: ArrayLike,
    hip_speed_limit: float,
    lookahead: float,
    velocity_stabilization: float,
    deadline_s: float,
    method: Literal["weighted", "hierarchical"],
    initial: ArrayLike | None = None,
) -> Allocation:
    """Solve weighted or lexicographic least squares with hard physics.

    Eliminate hard equalities and fixed variables before optimizing. The 20-entry
    solution is [world/joint acceleration (10), effort (4), world force L/R (6)].
    All original constraints and higher-priority outputs remain checked. A timeout
    or an inaccurate/infeasible result returns no command.
    """
    if method not in {"weighted", "hierarchical"} or not tasks:
        raise ValueError(
            "Specify weighted/hierarchical allocation with acceleration tasks"
        )
    if len(modes) != 2 or any(
        mode not in {"rolling", "sliding", "flight"} for mode in modes
    ):
        raise ValueError("Specify the active contact mode of each wheel")
    if (
        not np.isfinite(
            [deadline_s, lookahead, velocity_stabilization, hip_speed_limit]
        ).all()
        or min(deadline_s, lookahead, hip_speed_limit) <= 0
        or velocity_stabilization < 0
    ):
        raise ValueError(
            "Invalid allocation deadline, lookahead, hip speed or stabilization"
        )
    friction = np.asarray(friction, dtype=float)
    effort_bounds = np.asarray(effort_bounds, dtype=float)
    hip_limits = np.asarray(hip_limits, dtype=float)
    if (
        friction.shape != (2,)
        or effort_bounds.shape != (4,)
        or hip_limits.shape != (2,)
        or not np.isfinite(np.r_[friction, effort_bounds, hip_limits]).all()
        or np.any(friction < 0)
        or np.any(effort_bounds < 0)
        or hip_limits[0] >= hip_limits[1]
    ):
        raise ValueError("Invalid friction, effort or hip bounds")
    if any(
        np.shape(task.row) != (10,)
        or not np.isfinite(task.row).all()
        or not np.isfinite([task.target, task.scale]).all()
        or task.scale <= 0
        for task in tasks
    ):
        raise ValueError(
            "Tasks require ten finite coefficients, a finite target and positive scale"
        )
    guess = np.zeros(20) if initial is None else np.asarray(initial, dtype=float)
    if guess.shape != (20,) or not np.isfinite(guess).all():
        raise ValueError("Initial allocation must contain twenty finite values")

    start = perf_counter()
    eq, rhs, inequalities, lower, upper = _physical_constraints(
        terms,
        state,
        modes,
        friction,
        effort_bounds,
        hip_limits,
        hip_speed_limit,
        lookahead,
        velocity_stabilization,
    )
    if np.any(lower > upper):
        return Allocation("infeasible", None, np.inf, perf_counter() - start)
    # Scale heterogeneous accelerations, moments and forces before solving.
    scales = np.r_[np.full(10, 10.0), np.maximum(effort_bounds, 1.0), np.full(6, 20.0)]
    eq *= scales
    inequalities *= scales
    lower, upper = lower / scales, upper / scales
    guess = np.clip(guess / scales, lower, upper)
    groups = [tasks] if method == "weighted" else [(task,) for task in tasks]
    max_residual = np.inf
    for index, group in enumerate(groups):
        if perf_counter() - start > deadline_s:
            return Allocation("deadline", None, max_residual, perf_counter() - start)
        task_matrix = np.array(
            [np.r_[task.row, np.zeros(10)] * scales / task.scale for task in group]
        )
        desired = np.array([task.target / task.scale for task in group])
        regularization = 1e-6 if index + 1 == len(groups) else 0.0
        success, guess = _minimize_task(
            eq,
            rhs,
            inequalities,
            lower,
            upper,
            task_matrix,
            desired,
            regularization,
            guess,
        )
        elapsed = perf_counter() - start
        max_residual = _constraint_residual(guess, eq, rhs, inequalities, lower, upper)
        if elapsed > deadline_s:
            return Allocation("deadline", None, float(max_residual), elapsed)
        if not success or not np.isfinite(max_residual) or max_residual > 2e-6:
            return Allocation("infeasible", None, float(max_residual), elapsed)
        if index + 1 < len(groups):
            eq = np.vstack((eq, task_matrix))
            rhs = np.r_[rhs, task_matrix @ guess]
    labels = (
        "ax",
        "ay",
        "az",
        "angular_ax",
        "angular_ay",
        "angular_az",
        "hip_L_accel",
        "hip_R_accel",
        "spin_L_accel",
        "spin_R_accel",
        "hip_L_effort",
        "hip_R_effort",
        "wheel_L_effort",
        "wheel_R_effort",
        "L_force_x",
        "L_force_y",
        "L_force_z",
        "R_force_x",
        "R_force_y",
        "R_force_z",
    )
    active = [
        f"{name}:{side}"
        for side, bounds in (("lower", lower), ("upper", upper))
        for name, value, bound in zip(labels, guess, bounds)
        if abs(value - bound) <= 2e-6
    ]
    edges = [
        f"{side}_friction_edge_{i + 1}"
        for side, mode in zip(("L", "R"), modes)
        if mode != "flight"
        for i in range(4)
    ]
    active.extend(
        name for name, margin in zip(edges, inequalities @ guess) if abs(margin) <= 2e-6
    )
    return Allocation(
        "solved",
        guess * scales,
        float(max_residual),
        perf_counter() - start,
        tuple(active),
    )
