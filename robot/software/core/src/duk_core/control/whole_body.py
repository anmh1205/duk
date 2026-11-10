"""Constrained inverse dynamics for the spatial research model."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.linalg import qr
from scipy.optimize import Bounds, LinearConstraint, minimize


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


def _independent_rows(matrix):
    _, triangular, pivots = qr(matrix.T, mode="economic", pivoting=True)
    diag = abs(np.diag(triangular))
    rank = np.count_nonzero(diag > 1e-10 * max(1.0, diag[0]))
    return pivots[:rank]


def allocate(
    terms,
    state,
    tasks,
    modes,
    *,
    friction,
    effort_bounds,
    hip_limits,
    hip_speed_limit,
    lookahead,
    velocity_stabilization,
    deadline_s,
    method,
    initial=None,
):
    """Solve weighted or lexicographic least squares with hard physics.

    Keep all rows for verification, even if the optimizer needs a smaller
    independent equality basis. A timeout or an inaccurate/infeasible result
    returns no command.
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
    if any(not np.isfinite(task.scale) or task.scale <= 0 for task in tasks):
        raise ValueError("Task scales must be finite and positive")

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
    guess = np.zeros(20) if initial is None else np.asarray(initial) / scales
    guess = np.clip(guess, lower, upper)
    groups = [tasks] if method == "weighted" else [(task,) for task in tasks]
    max_residual = np.inf
    for index, group in enumerate(groups):
        if perf_counter() - start > deadline_s:
            return Allocation("deadline", None, max_residual, perf_counter() - start)
        independent = _independent_rows(eq)
        constraints = [
            LinearConstraint(eq[independent], rhs[independent], rhs[independent])
        ]
        if inequalities.size:
            constraints.append(LinearConstraint(inequalities, -np.inf, 0.0))
        task_matrix = np.array(
            [np.r_[task.row, np.zeros(10)] * scales / task.scale for task in group]
        )
        desired = np.array([task.target / task.scale for task in group])
        regularization = 1e-6 if index + 1 == len(groups) else 0.0
        hessian = task_matrix.T @ task_matrix + regularization * np.eye(20)
        gradient = -task_matrix.T @ desired
        result = minimize(
            lambda x: 0.5 * x @ hessian @ x + gradient @ x,
            guess,
            jac=lambda x: hessian @ x + gradient,
            method="SLSQP",
            bounds=Bounds(lower, upper),
            constraints=constraints,
            options={"ftol": 1e-9, "maxiter": 60},
        )
        elapsed = perf_counter() - start
        guess = result.x
        max_residual = max(
            np.max(abs(eq @ guess - rhs)),
            np.max(inequalities @ guess, initial=0.0),
            np.max(lower - guess),
            np.max(guess - upper),
        )
        if elapsed > deadline_s:
            return Allocation("deadline", None, float(max_residual), elapsed)
        if not result.success or not np.isfinite(max_residual) or max_residual > 2e-6:
            return Allocation("infeasible", None, float(max_residual), elapsed)
        if index + 1 < len(groups):
            eq = np.vstack((eq, task_matrix))
            rhs = np.r_[rhs, task_matrix @ guess]
    return Allocation(
        "solved", guess * scales, float(max_residual), perf_counter() - start
    )
