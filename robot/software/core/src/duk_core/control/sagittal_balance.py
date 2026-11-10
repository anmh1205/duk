"""Sampled fixed-hip balance experiments with actuator and traction limits."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from ..estimation.sagittal import SlipGatedVelocity
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


@dataclass(frozen=True)
class BalanceTrace:
    time: np.ndarray
    state: np.ndarray
    requested: np.ndarray
    limited: np.ndarray
    applied: np.ndarray
    slip: np.ndarray
    normal: np.ndarray
    tangent: np.ndarray
    speed_estimate: np.ndarray
    limit_reason: tuple[str, ...]
    outcome: str


def simulate_balance(
    model,
    gain,
    initial_lean,
    *,
    period,
    duration,
    friction,
    motor_bound,
    slip_scale,
    fall_angle,
    feedback,
    traction_budget=None,
    anchor=None,
    delay_samples=0,
    noise_seed=0,
    accel_noise=0.0,
    pitch_noise=0.0,
    slip_threshold=0.03,
    slip_confirmation_samples=2,
    wheel_correction=0.08,
):
    """Use the same finite-slip plant for truth, wheel odometry and gated replay."""
    if feedback not in {"truth", "odometry", "gated"}:
        raise ValueError("Unknown feedback source")
    if (
        not isinstance(delay_samples, (int, np.integer))
        or not np.isfinite(
            [
                period,
                duration,
                friction,
                motor_bound,
                slip_scale,
                fall_angle,
                initial_lean,
                accel_noise,
                pitch_noise,
            ]
        ).all()
        or min(period, duration, slip_scale, fall_angle) <= 0
        or min(friction, motor_bound, delay_samples, accel_noise, pitch_noise) < 0
    ):
        raise ValueError("Invalid balance scenario")
    rng = np.random.default_rng(noise_seed)
    observer = SlipGatedVelocity(
        0.0, slip_threshold, slip_confirmation_samples, wheel_correction
    )
    state = np.array([0.0, 0.0, initial_lean, 0.0, 0.0, 0.0])
    estimated_x, time = 0.0, 0.0
    commands, rows, reasons = [], [], []
    previous_applied = 0.0
    outcome = "completed"

    def contact(s, applied):
        return model.finite_slip(
            s[2], s[3], applied, s[1] - model.radius_m * s[5], friction, slip_scale
        )

    for tick in range(int(np.ceil(duration / period))):
        x, speed, lean, rate, spin, spin_rate = state
        wheel_speed = model.radius_m * spin_rate
        if feedback == "truth":
            x_hat, speed_hat = x, speed
        elif feedback == "odometry":
            x_hat, speed_hat = model.radius_m * spin, wheel_speed
        else:
            if tick:
                response = contact(state, previous_applied)
                estimate = observer.step(
                    period,
                    response.acceleration[0] + rng.normal(0, accel_noise),
                    wheel_speed,
                    in_contact=response.normal > 0,
                )
                estimated_x += period * estimate.speed
            x_hat, speed_hat = estimated_x, observer.speed
        lean_hat = lean + rng.normal(0, pitch_noise)
        request = balance_command(
            gain, [x_hat, speed_hat, lean_hat, rate], anchor=anchor
        )
        interval = (-motor_bound, motor_bound)
        if traction_budget is not None:
            interval = traction_interval(
                model, lean_hat, rate, motor_bound, traction_budget
            )
        decision = project_torque(request, motor_bound, interval)
        # The stop policy clears queued commands when no torque is admissible.
        limited = 0.0 if decision.applied is None else decision.applied
        commands.append(limited)
        applied = (
            commands[tick - delay_samples]
            if decision.applied is not None and tick >= delay_samples
            else 0.0
        )
        previous_applied = applied
        reasons.append(decision.reason)
        sample = contact(state, applied)
        rows.append(
            (
                time,
                state,
                request,
                limited,
                applied,
                speed - wheel_speed,
                sample.normal,
                sample.tangent,
                speed_hat,
            )
        )

        def derivative(_, s):
            ax, angular_acc, wheel_acc = contact(s, applied).acceleration
            return np.array([s[1], ax, s[3], angular_acc, s[5], wheel_acc])

        def fall(_, s):
            return fall_angle - abs(s[2])

        def lift(_, s):
            return contact(s, applied).normal

        fall.terminal = lift.terminal = True
        fall.direction = lift.direction = -1
        if sample.normal <= 0 or abs(lean) >= fall_angle:
            outcome = "lost_contact" if sample.normal <= 0 else "pitch_limit"
            break
        if decision.applied is None:
            outcome = "traction_infeasible"
            break
        step = solve_ivp(
            derivative,
            (time, min(duration, (tick + 1) * period)),
            state,
            method="LSODA",
            rtol=1e-6,
            atol=1e-8,
            events=(fall, lift),
        )
        if not step.success:
            raise RuntimeError(step.message)
        time, state = step.t[-1], step.y[:, -1]
        if step.status == 1 or tick + 1 == int(np.ceil(duration / period)):
            outcome = (
                "pitch_limit"
                if step.t_events[0].size
                else "lost_contact"
                if step.t_events[1].size
                else "completed"
            )
            end = contact(state, applied)
            rows.append(
                (
                    time,
                    state,
                    request,
                    limited,
                    applied,
                    state[1] - model.radius_m * state[5],
                    end.normal,
                    end.tangent,
                    speed_hat,
                )
            )
            reasons.append(decision.reason)
            break

    return BalanceTrace(
        *[np.asarray(column) for column in zip(*rows)], tuple(reasons), outcome
    )
