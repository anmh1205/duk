"""Sagittal estimation baselines for timestamped simulated sensor replay."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def absolute_wheel_rate(relative_rate, body_rate, carrier_rate):
    """Wheel absolute CCW rate from already calibrated relative and carrier rates."""
    return np.asarray(relative_rate) + body_rate + carrier_rate


def complementary_pitch(previous, gyro_rate, acceleration_pitch, dt, time_constant):
    if (
        not np.isfinite(time_constant)
        or time_constant <= 0
        or not np.isfinite(dt)
        or dt <= 0
    ):
        raise ValueError(
            "Filter interval and time constant must be finite and positive"
        )
    weight = time_constant / (time_constant + dt)
    predicted = previous + gyro_rate * dt
    innovation = (acceleration_pitch - predicted + np.pi) % (2 * np.pi) - np.pi
    return predicted + (1 - weight) * innovation


@dataclass(frozen=True)
class VelocityEstimate:
    speed: float
    slip: float
    odometry_accepted: bool
    slip_detected: bool


class SlipGatedVelocity:
    """Predict from independent axle acceleration and gate wheel-speed corrections."""

    def __init__(
        self,
        speed: float,
        threshold: float,
        confirmation_samples: int,
        correction: float,
    ):
        if not np.isfinite([speed, threshold, correction]).all() or threshold <= 0:
            raise ValueError("Invalid initial speed or slip threshold")
        if confirmation_samples < 1 or not 0 <= correction <= 1:
            raise ValueError("Invalid confirmation count or wheel correction")
        self.speed = speed
        self.threshold = threshold
        self.confirmation_samples = confirmation_samples
        self.correction = correction
        self._bad = 0
        self._good = 0
        self._slipping = False

    def step(self, dt, axle_acceleration, wheel_surface_speed, *, in_contact=True):
        if (
            not np.isfinite([dt, axle_acceleration, wheel_surface_speed]).all()
            or dt <= 0
        ):
            raise ValueError(
                "Velocity replay requires finite measurements and a positive interval"
            )
        predicted = self.speed + axle_acceleration * dt
        slip = predicted - wheel_surface_speed
        suspicious = abs(slip) > self.threshold or not in_contact
        self._bad = self._bad + 1 if suspicious else 0
        self._good = self._good + 1 if not suspicious else 0
        if self._bad >= self.confirmation_samples or not in_contact:
            self._slipping = True
        if self._good >= self.confirmation_samples:
            self._slipping = False
        accepted = not suspicious and not self._slipping
        self.speed = predicted - self.correction * slip if accepted else predicted
        return VelocityEstimate(self.speed, slip, accepted, self._slipping)


def replay_velocity(
    times, independent_acceleration, wheel_surface_speed, in_contact, observer
):
    """Replay synchronized signals. Index zero records the initial estimate."""
    times, accel, wheel, contact = map(
        np.asarray, (times, independent_acceleration, wheel_surface_speed, in_contact)
    )
    if (
        times.ndim != 1
        or not times.size
        or any(x.shape != times.shape for x in (accel, wheel, contact))
    ):
        raise ValueError(
            "Replay requires equally sized, nonempty time and sensor channels"
        )
    records = [
        VelocityEstimate(
            observer.speed, observer.speed - wheel[0], bool(contact[0]), False
        )
    ]
    for i, dt in enumerate(np.diff(times), start=1):
        records.append(
            observer.step(dt, accel[i], wheel[i], in_contact=bool(contact[i]))
        )
    return records


def ideal_planar_contact_estimate(
    wheel_torque_ccw,
    spin_inertia,
    spin_acceleration,
    radius,
    total_mass,
    gravity,
    com_vertical_acceleration,
):
    """Use wheel spin balance and total vertical force, assuming negligible axle losses."""
    tangential = (spin_inertia * spin_acceleration - wheel_torque_ccw) / radius
    normal_total = total_mass * (gravity + com_vertical_acceleration)
    return tangential, normal_total
