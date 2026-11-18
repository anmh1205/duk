"""Simulated observations and integration guards for the whole-body experiment."""

from dataclasses import dataclass

import numpy as np

from ..estimation.sagittal import SlipGatedVelocity
from ..modeling.leg_linkage import LinkageError


@dataclass(frozen=True)
class Observation:
    """Pose/rates in lean, body height, roll, yaw order with wheel contact modes."""

    pose: np.ndarray
    rate: np.ndarray
    contact_speed: np.ndarray
    modes: tuple[str, str]
    speed: float
    estimated_speed: float
    odometry: float
    acceleration: float


class ContactObserver:
    """Own velocity replay, noise generation and the friction belief."""

    def __init__(self, settings, scenario):
        self.settings = settings
        self.observer = SlipGatedVelocity(0.0, settings["slip_threshold_m_s"], 2, 0.08)
        self.rng = np.random.default_rng(settings["sensor_seed"])
        self.friction_belief = np.full(2, scenario["friction"], dtype=float)
        self.friction_belief *= settings["friction_reserve"]
        self.last_speed = self.forward_estimate = 0.0

    def observe(self, model, state, terms, previous_force, tick) -> Observation:
        p, k = self.settings, terms.kinematics
        period = p["period_s"]
        rate = k.task_jacobian @ state.velocity
        c_speed = k.contact @ state.velocity
        supported = (k.gaps <= p["contact_gap_m"]) & (previous_force[:, 2] > 0)
        current_speed = float(
            k.forward @ (k.linear[model.wheels] @ state.velocity).mean(axis=0)
        )
        odometry = model.rolling_odometry(k, state.velocity).mean()
        independent = np.nan
        if tick:
            independent = (current_speed - self.last_speed) / period + self.rng.normal(
                0.0, p["accel_noise_m_s2"]
            )
            estimate = self.observer.step(
                period,
                independent,
                odometry,
                in_contact=bool(supported.all()),
            )
            self.forward_estimate = estimate.speed
            longitudinal_slip = estimate.slip_detected
        else:
            longitudinal_slip = False
        self.last_speed = current_speed
        sliding = (
            np.linalg.norm(c_speed[:, :2], axis=1) > p["spatial_slip_threshold_m_s"]
        )
        sliding |= longitudinal_slip & (
            abs(c_speed @ k.forward) > p["slip_threshold_m_s"]
        )
        modes = tuple(
            "flight" if not on_ground else "sliding" if slip else "rolling"
            for on_ground, slip in zip(supported, sliding)
        )
        valid_force = sliding & (previous_force[:, 2] > p["minimum_force_N"])
        ratios = (
            np.linalg.norm(previous_force[valid_force, :2], axis=1)
            / previous_force[valid_force, 2]
        )
        self.friction_belief[valid_force] = np.minimum(
            self.friction_belief[valid_force], p["friction_reserve"] * ratios
        )
        return Observation(
            pose=k.tasks,
            rate=rate,
            contact_speed=c_speed,
            modes=modes,
            speed=current_speed,
            estimated_speed=self.forward_estimate,
            odometry=odometry,
            acceleration=independent,
        )


class PlantGuard:
    """Track terminal evidence and stop the numerical plant at substep guards."""

    def __init__(self, settings):
        self.settings = settings
        self.flight_started = None
        self.terminal_time = 0.0
        self.terminal_pose = np.full(4, np.nan)
        self.peak_abs_lean_roll = np.zeros(2)
        self.stop_reason = None

    def observe(self, kinematics, force, time) -> bool:
        p = self.settings
        self.terminal_time = time
        self.terminal_pose = kinematics.tasks
        self.peak_abs_lean_roll = np.maximum(
            self.peak_abs_lean_roll, abs(kinematics.tasks[[0, 2]])
        )
        supported = (kinematics.gaps <= p["contact_gap_m"]) & (force[:, 2] > 0)
        if supported.any():
            self.flight_started = None
        elif self.flight_started is None:
            self.flight_started = time
        return np.max(abs(kinematics.tasks[[0, 2]])) > np.deg2rad(p["fall_deg"]) or (
            self.flight_started is not None
            and time - self.flight_started > p["max_flight_s"]
        )

    def advance(self, model, state, effort, friction, setup, start, end, duration):
        substeps = self.settings["substeps"]
        period = self.settings["period_s"]
        step = (period if end < duration else end - start) / substeps
        for index in range(substeps):
            state, force = model.compliant_step(
                state, effort, friction, setup.contact_law, step
            )
            self.terminal_time = start + (index + 1) * step
            hip_travel = np.any(state.joints[:2] < setup.hip_limits[0]) or np.any(
                state.joints[:2] > setup.hip_limits[1]
            )
            try:
                current = model.kinematics(state)
            except LinkageError:
                if not hip_travel:
                    raise
                self.terminal_pose = np.full(4, np.nan)
                self.stop_reason = "hip_travel"
                break
            fallen = self.observe(current, force, self.terminal_time)
            if hip_travel or fallen:
                self.stop_reason = "hip_travel" if hip_travel else "flight_or_fall"
                break
        if self.stop_reason is None:
            self.terminal_time = end
        return state, force
