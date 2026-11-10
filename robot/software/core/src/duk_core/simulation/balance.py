"""Sampled fixed-hip experiments with a finite-slip plant and sensor replay."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from ..control.sagittal_balance import (
    balance_command,
    project_torque,
    traction_interval,
)
from ..estimation.sagittal import SlipGatedVelocity
from ..modeling.wheeled_pendulum import WheeledPendulum
from ._sampled import sample_intervals


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
    model: WheeledPendulum,
    gain: np.ndarray,
    initial_lean: float,
    *,
    period: float,
    duration: float,
    friction: float,
    motor_bound: float,
    slip_scale: float,
    fall_angle: float,
    feedback: str,
    traction_budget: float | None = None,
    anchor: float | None = None,
    delay_samples: int = 0,
    noise_seed: int = 0,
    accel_noise: float = 0.0,
    pitch_noise: float = 0.0,
    slip_threshold: float = 0.03,
    slip_confirmation_samples: int = 2,
    wheel_correction: float = 0.08,
) -> BalanceTrace:
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

    _, ends = sample_intervals(duration, period)
    for tick, end_time in enumerate(ends):
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
            (time, end_time),
            state,
            method="LSODA",
            rtol=1e-6,
            atol=1e-8,
            events=(fall, lift),
        )
        if not step.success:
            raise RuntimeError(step.message)
        time, state = step.t[-1], step.y[:, -1]
        if step.status == 1 or tick + 1 == len(ends):
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
