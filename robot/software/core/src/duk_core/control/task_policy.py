"""Simulation task transitions and supervisory policy independent of a solver."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FeatureTransition:
    amount: float = 0.0
    status: str = "Idle"

    def step(self, enabled, available, capacity, dt, ramp_time):
        if (
            not np.isfinite([capacity, dt, ramp_time]).all()
            or not 0 <= capacity <= 1
            or min(dt, ramp_time) <= 0
        ):
            raise ValueError("Invalid feature capacity or transition interval")
        if not available:
            self.amount = 0.0
            self.status = "Paused" if enabled else "Idle"
            return self.amount
        target = capacity if enabled else 0.0
        self.amount += float(
            np.clip(target - self.amount, -dt / ramp_time, dt / ramp_time)
        )
        if not enabled:
            self.status = "Exit" if self.amount > 1e-10 else "Idle"
        elif capacity < 1:
            self.status = "Limited"
        elif self.amount < 1 - 1e-10:
            self.status = "Enter"
        else:
            self.status = "Active"
        return self.amount


@dataclass
class Supervisor:
    fallback_after: int
    recover_after: int
    failures: int = 0
    successes: int = 0
    mode: str = "normal"

    def step(self, solver_status, *, sensor_valid, supported, fallback_valid, fall):
        if self.mode == "stop" or fall or not sensor_valid:
            self.mode = "stop"
        elif solver_status != "solved":
            self.failures += 1
            self.successes = 0
            if self.failures >= self.fallback_after:
                self.mode = "fallback" if fallback_valid and supported else "stop"
            else:
                self.mode = "reduced"
        else:
            self.failures = 0
            self.successes = self.successes + 1 if supported else 0
            self.mode = "normal" if self.successes >= self.recover_after else "reduced"
        return self.mode


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def return_to_anchor(
    position,
    anchor,
    heading,
    *,
    speed_gain,
    yaw_gain,
    speed_limit,
    yaw_limit,
    position_tolerance,
):
    """Rotate/drive toward an explicitly recorded planar anchor."""
    displacement = np.asarray(anchor) - position
    distance = np.linalg.norm(displacement)
    if distance <= position_tolerance:
        return 0.0, 0.0
    bearing = wrap_angle(np.arctan2(displacement[1], displacement[0]) - heading)
    forward = max(0.0, np.cos(bearing))
    return (
        float(min(speed_limit, speed_gain * distance * forward)),
        float(np.clip(yaw_gain * bearing, -yaw_limit, yaw_limit)),
    )
