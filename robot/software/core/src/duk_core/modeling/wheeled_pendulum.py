"""Fixed-hip wheel-moment pendulum in forward-positive coordinates."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contact_dynamics import constrained_acceleration


@dataclass(frozen=True)
class WheelResponse:
    acceleration: np.ndarray
    tangent: float
    normal: float


@dataclass(frozen=True)
class WheeledPendulum:
    """Aggregate the two wheels and the locked upper assembly separately."""

    wheel_mass_kg: float
    wheel_inertia_kg_m2: float
    upper_mass_kg: float
    upper_inertia_kg_m2: float
    length_m: float
    radius_m: float
    gravity_m_s2: float

    def __post_init__(self):
        values = (
            self.wheel_mass_kg,
            self.wheel_inertia_kg_m2,
            self.upper_mass_kg,
            self.upper_inertia_kg_m2,
            self.length_m,
            self.radius_m,
            self.gravity_m_s2,
        )
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError(
                "All pendulum masses, inertias, lengths and gravity must be finite and positive"
            )

    @classmethod
    def from_biped(cls, model, q):
        upper = model.equivalent_upper_body(q)
        wheels = list(model.wheels)
        return cls(
            model.masses[wheels].sum(),
            model.inertias[wheels].sum(),
            upper.mass_kg,
            upper.inertia_about_center_kg_m2,
            upper.length_m,
            model.radius,
            model.parameters["g_m_s2"],
        )

    @property
    def moment(self):
        return self.upper_inertia_kg_m2 + self.upper_mass_kg * self.length_m**2

    def linearization(self):
        """A, B at upright in [axle x, axle velocity, lean, lean rate]."""
        c = self.upper_mass_kg * self.length_m
        radius = self.radius_m
        mass = (
            self.upper_mass_kg
            + self.wheel_mass_kg
            + self.wheel_inertia_kg_m2 / radius**2
        )
        mass_matrix = np.array([[mass, c], [c, self.moment]])
        a = np.zeros((4, 4))
        a[0, 1] = a[2, 3] = 1
        a[[1, 3], 2] = np.linalg.solve(mass_matrix, [0, c * self.gravity_m_s2])
        b = np.zeros((4, 1))
        b[[1, 3], 0] = np.linalg.solve(mass_matrix, [1 / radius, -1])
        return a, b

    def _axle_dynamics(self, lean, lean_rate, torque):
        c = self.upper_mass_kg * self.length_m
        total_mass = self.upper_mass_kg + self.wheel_mass_kg
        cosine, sine = np.cos(lean), np.sin(lean)
        mass = np.array(
            [
                [total_mass, c * cosine, 0.0],
                [c * cosine, self.moment, 0.0],
                [0.0, 0.0, self.wheel_inertia_kg_m2],
            ]
        )
        rhs = np.array(
            [c * sine * lean_rate**2, c * self.gravity_m_s2 * sine - torque, torque]
        )
        contact = np.array([1.0, 0.0, -self.radius_m])
        normal_at_zero_acceleration = (
            total_mass * self.gravity_m_s2 - c * cosine * lean_rate**2
        )
        return mass, rhs, contact, normal_at_zero_acceleration, c * sine

    def rolling(self, lean, lean_rate, torque):
        mass, rhs, row, n0, coefficient = self._axle_dynamics(lean, lean_rate, torque)
        acceleration, tangent = constrained_acceleration(
            mass, rhs, row[None, :], np.zeros(1)
        )
        return WheelResponse(
            acceleration, float(tangent[0]), float(n0 - coefficient * acceleration[1])
        )

    def finite_slip(self, lean, lean_rate, torque, slip, friction, slip_scale):
        """Smooth tangential friction with height held while N is positive."""
        mass, rhs, row, n0, coefficient = self._axle_dynamics(lean, lean_rate, torque)
        signed_ratio = friction * np.tanh(slip / slip_scale)
        coupled_mass = mass.copy()
        coupled_mass[:, 1] -= signed_ratio * coefficient * row
        acceleration = np.linalg.solve(coupled_mass, rhs - signed_ratio * n0 * row)
        normal = float(n0 - coefficient * acceleration[1])
        if normal <= 0:
            # The caller terminates the held-height experiment at lift-off.
            acceleration = np.linalg.solve(mass, rhs)
        return WheelResponse(acceleration, -signed_ratio * max(normal, 0.0), normal)
