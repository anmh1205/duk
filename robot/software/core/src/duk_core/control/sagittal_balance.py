"""Fixed-hip feedback and admissible wheel-moment projection."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..modeling.wheeled_pendulum import WheeledPendulum


@dataclass(frozen=True)
class TorqueDecision:
    requested: float
    applied: float | None
    reason: str


def traction_interval(
    model: WheeledPendulum, lean, lean_rate, motor_bound, friction_budget
):
    """Intersect affine rolling force constraints with symmetric motor bounds."""
    if (
        not np.isfinite([motor_bound, friction_budget]).all()
        or min(motor_bound, friction_budget) < 0
    ):
        raise ValueError("Limits must be finite and nonnegative")
    lower, upper = -motor_bound, motor_bound
    a0 = model.rolling(lean, lean_rate, 0.0)
    a1 = model.rolling(lean, lean_rate, 1.0)
    dn, dt = a1.normal - a0.normal, a1.tangent - a0.tangent
    for coefficient, intercept in (
        (-dn, -a0.normal),
        (dt - friction_budget * dn, a0.tangent - friction_budget * a0.normal),
        (-dt - friction_budget * dn, -a0.tangent - friction_budget * a0.normal),
    ):
        if coefficient > 1e-12:
            upper = min(upper, -intercept / coefficient)
        elif coefficient < -1e-12:
            lower = max(lower, -intercept / coefficient)
        elif intercept > 0:
            return None
    return (lower, upper) if lower <= upper else None


def project_torque(requested, motor_bound, interval):
    """Apply the motor bound even when a caller supplies a wider traction interval."""
    if not np.isfinite([requested, motor_bound]).all() or motor_bound < 0:
        raise ValueError("Torque request and motor bound must be finite")
    if interval is None:
        return TorqueDecision(float(requested), None, "infeasible")
    lower, upper = interval
    if np.isnan([lower, upper]).any():
        raise ValueError("Torque interval must not contain NaN")
    lower, upper = max(lower, -motor_bound), min(upper, motor_bound)
    if lower > upper:
        return TorqueDecision(float(requested), None, "infeasible")
    applied = float(np.clip(requested, lower, upper))
    if applied == requested:
        reason = "within_limits"
    elif applied == np.clip(requested, -motor_bound, motor_bound):
        reason = "motor"
    else:
        reason = "traction"
    return TorqueDecision(float(requested), applied, reason)


def balance_command(gain, state, *, anchor=None, target_speed=0.0):
    """Keep an anchor out of the balance-only state and require it for position gain."""
    x, speed, lean, lean_rate = state
    error = [speed - target_speed, lean, lean_rate]
    if anchor is not None:
        error.insert(0, x - anchor)
    gain = np.asarray(gain)
    if gain.size != len(error):
        raise ValueError("Gain must match balance-only or explicitly anchored mode")
    return float(gain.reshape(-1) @ -np.asarray(error))
