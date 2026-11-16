"""Numerical access to the canonical sagittal rigid bodies and equations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from functools import cached_property
from types import MappingProxyType

import numpy as np
from numpy.typing import ArrayLike

from .leg_differential import angle_derivatives
from .mass_properties import (
    PlanarRigidBody,
    combine_planar_mass_properties,
    equivalent_pendulum_2d,
)
from .planar_biped import PlanarBipedModel


@dataclass(frozen=True)
class PlanarKinematics:
    centers: np.ndarray
    jacobians: np.ndarray
    angular_rows: np.ndarray


@dataclass(frozen=True)
class PlanarKinematicBias:
    """Per-body convective acceleration at zero generalized acceleration, in SI."""

    linear: np.ndarray
    angular: np.ndarray


@dataclass(frozen=True)
class PlanarDynamics:
    mass: np.ndarray
    bias: np.ndarray
    input: np.ndarray
    gaps: np.ndarray
    contact: np.ndarray
    contact_bias: np.ndarray
    potential: float
    gravity: np.ndarray


class NumericPlanarBiped:
    """Compile a fully bound symbolic model once per experiment.

    The numerical model requires all parameter values explicitly. Kinematics
    stay on the physical assembly branch checked by the canonical solver.
    """

    def __init__(self, model: PlanarBipedModel, parameters: Mapping[str, float]):
        missing = sorted(
            name for name in model.parameters if parameters.get(name) is None
        )
        if missing:
            raise ValueError("Missing numeric parameters: " + ", ".join(missing))
        self.model = model
        self._bindings = model.bind_parameters(parameters)
        self.parameters = MappingProxyType(
            {name: self._bindings[s] for name, s in model.parameters.items()}
        )
        self.names = tuple(body.name for body in model.bodies)
        self.masses = np.array([self._bindings[body.mass] for body in model.bodies])
        self.inertias = np.array(
            [self._bindings[body.inertia] for body in model.bodies]
        )
        self.wheels = tuple(self.names.index("wheel_" + side) for side in ("L", "R"))
        self.radius = model.geometry.wheel_radius_mm / 1000.0

    def _compile(self, expressions, *, velocity=False):
        from sympy import lambdify

        args = tuple(self.model.coordinates)
        if velocity:
            args += tuple(self.model.velocities)
        entries = [
            value.xreplace(self._bindings)
            for expression in expressions
            for value in expression
        ]
        return lambdify(args, entries, "numpy", cse=True, docstring_limit=0)

    @cached_property
    def _kinematics(self):
        bodies = self.model.bodies
        return self._compile(
            [body.center for body in bodies]
            + [body.jacobian for body in bodies]
            + [body.angular_jacobian for body in bodies]
        )

    def kinematics(self, configuration) -> PlanarKinematics:
        self.model.bind_configuration(configuration)
        count = len(self.names)
        values = np.asarray(self._kinematics(*configuration), dtype=float)
        centers, jacobians, angular = np.split(values, [2 * count, 16 * count])
        return PlanarKinematics(
            centers.reshape(count, 2),
            jacobians.reshape(count, 2, 7),
            angular.reshape(count, 7),
        )

    @cached_property
    def _kinematic_bias(self):
        terms = self.model.body_acceleration_bias
        return self._compile(
            [linear for linear, _ in terms] + [angular for _, angular in terms],
            velocity=True,
        )

    def kinematic_bias(self, configuration, velocity) -> PlanarKinematicBias:
        """Evaluate analytic Jdot v on the physical branch without perturbed poses."""
        self.model.bind_configuration(configuration)
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (7,) or not np.isfinite(velocity).all():
            raise ValueError("Expected seven finite generalized velocities")
        values = np.asarray(
            self._kinematic_bias(*configuration, *velocity), dtype=float
        )
        linear, angular = np.split(values, [2 * len(self.names)])
        return PlanarKinematicBias(linear.reshape(-1, 2), angular)

    def center_of_mass_acceleration(
        self, configuration: ArrayLike, velocity: ArrayLike, acceleration: ArrayLike
    ) -> np.ndarray:
        """Return world (x, z) CoM acceleration in m/s² at this configuration.

        Use the mass-weighted body accelerations J a + Jdot v. No neighbouring
        configurations are evaluated, including near a linkage branch boundary.
        """
        acceleration = np.asarray(acceleration, dtype=float)
        if acceleration.shape != (7,) or not np.isfinite(acceleration).all():
            raise ValueError("Expected seven finite generalized accelerations")
        kinematics = self.kinematics(configuration)
        bias = self.kinematic_bias(configuration, velocity)
        body_acceleration = kinematics.jacobians @ acceleration + bias.linear
        return self.masses @ body_acceleration / self.masses.sum()

    @cached_property
    def _dynamics(self):
        model = self.model
        bias = (
            model.velocity_bias
            + model.gravity
            + model.damping_matrix @ model.velocities
        )
        return self._compile(
            [
                model.mass_matrix,
                bias,
                model.input_matrix,
                model.normal_gaps,
                model.contact_jacobian,
                model.contact_acceleration_bias,
                [model.potential_energy],
                model.gravity,
            ],
            velocity=True,
        )

    def evaluate(self, configuration, velocity) -> PlanarDynamics:
        self.model.bind_configuration(configuration)
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (7,) or not np.isfinite(velocity).all():
            raise ValueError("Expected seven finite generalized velocities")
        values = np.asarray(self._dynamics(*configuration, *velocity), dtype=float)
        mass, bias, inputs, gaps, contact, drift, potential, gravity = np.split(
            values, [49, 56, 84, 86, 114, 118, 119]
        )
        return PlanarDynamics(
            mass.reshape(7, 7),
            bias,
            inputs.reshape(7, 4),
            gaps,
            contact.reshape(4, 7),
            drift,
            float(potential[0]),
            gravity,
        )

    def equivalent_upper_body(self, q: float):
        """Compose the upper assembly about a common axle at equal left/right q."""
        state = self.kinematics([0.0, 0.0, 0.0, q, q, 0.0, 0.0])
        axle = self._symmetric_support(state)
        bodies = [
            PlanarRigidBody(mass, tuple(center * 1000.0), inertia)
            for i, (mass, center, inertia) in enumerate(
                zip(self.masses, state.centers, self.inertias)
            )
            if i not in self.wheels
        ]
        return equivalent_pendulum_2d(
            combine_planar_mass_properties(bodies), tuple(axle * 1000.0)
        )

    def _symmetric_support(self, kinematics):
        left, right = kinematics.centers[list(self.wheels)]
        if not np.allclose(left, right, rtol=0, atol=1e-10):
            raise ValueError(
                "The symmetric reduction requires coincident projected wheel centers"
            )
        return left

    def upper_body_angle_derivatives(self, q: float) -> tuple[float, float]:
        """Return d(angle)/dq and d²(angle)/dq² at equal hip coordinate q.

        The angle is from vertical, as in equivalent_upper_body. Derivatives
        use q in radians and evaluate only this configuration on its branch.
        """
        configuration = np.array([0.0, 0.0, 0.0, q, q, 0.0, 0.0])
        kinematics = self.kinematics(configuration)
        axle = self._symmetric_support(kinematics)
        upper = [index for index in range(len(self.masses)) if index not in self.wheels]
        weights = self.masses[upper] / self.masses[upper].sum()
        direction = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0])
        first = kinematics.jacobians @ direction
        second = self.kinematic_bias(configuration, direction).linear
        vectors = (
            weights @ kinematics.centers[upper] - axle,
            weights @ first[upper] - first[self.wheels[0]],
            weights @ second[upper] - second[self.wheels[0]],
        )
        # Angle from vertical uses atan2(forward, up).
        return angle_derivatives(*(tuple(vector[::-1]) for vector in vectors))

    def static_hip_effort(self, configuration, axle_forces_xz) -> np.ndarray:
        """Joint effort from gravity and specified external forces applied at O."""
        state = self.kinematics(configuration)
        forces = np.asarray(axle_forces_xz, dtype=float)
        if forces.shape != (2, 2) or not np.isfinite(forces).all():
            raise ValueError("Supply a finite sagittal axle force for each wheel")
        gravity = self.parameters["g_m_s2"] * self.masses @ state.jacobians[:, 1, 3:5]
        load = np.einsum(
            "ij,ijk->k", forces, state.jacobians[list(self.wheels), :, 3:5]
        )
        return gravity - load
