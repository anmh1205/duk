"""Parameterized sagittal-plane biped equations, independent of ROS/Gazebo.

This is a mathematical model with explicit assumptions, not identified robot
physics. Geometry comes from LegGeometry. Every other physical quantity stays
symbolic until the caller binds it. SymPy is imported only when building a model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cached_property
from types import MappingProxyType
from typing import Any, Mapping, Sequence

from .leg_linkage import LegGeometry, leg_jacobian, solve_leg_pose


@dataclass(frozen=True)
class PlanarBodyKinematics:
    """World CoM, translational Jacobian and absolute angular-velocity row."""

    name: str
    mass: Any
    inertia: Any
    center: Any
    jacobian: Any
    angular_jacobian: Any


@dataclass(frozen=True)
class PlanarBipedModel:
    """Seven coordinates: x, z, theta, q_L, q_R, phi_L, phi_R.

    phi is absolute wheel angle (CCW), not an encoder reading. q retains the
    established AB->DA convention. Positive theta is CCW in the (x,z) plane.
    Translation is in metres; unknown physical parameters have no defaults.
    """

    geometry: LegGeometry
    coordinates: Any
    velocities: Any
    accelerations: Any
    parameters: Mapping[str, Any]
    bodies: tuple[PlanarBodyKinematics, ...]
    wheel_centers: tuple
    local_points: Mapping[str, Any]
    mass_matrix: Any
    potential_energy: Any
    gravity: Any
    damping_matrix: Any
    input_matrix: Any
    inputs: Any
    normal_gaps: Any
    contact_jacobian: Any
    contact_forces: Any

    @cached_property
    def velocity_bias(self):
        """Convective inertial force Σ m Jᵀ Jdot v + I jωᵀ jωdot v."""
        import sympy as sp

        result = sp.zeros(7, 1)
        for body in self.bodies:
            translation = self._convective(body.jacobian)
            rotation = self._convective(body.angular_jacobian)
            result += body.mass * body.jacobian.T * translation
            result += body.inertia * body.angular_jacobian.T * rotation
        return sp.ImmutableMatrix(result)

    def _convective(self, jacobian):
        import sympy as sp

        derivative = sp.zeros(*jacobian.shape)
        for q, rate in zip(self.coordinates, self.velocities):
            if q in jacobian.free_symbols:
                derivative += jacobian.diff(q) * rate
        return derivative * self.velocities

    @cached_property
    def contact_acceleration_bias(self):
        """Jdot_contact v for an explicitly selected sticking-contact mode."""
        return self._convective(self.contact_jacobian).as_immutable()

    @property
    def kinetic_energy(self):
        return (self.velocities.T * self.mass_matrix * self.velocities)[0] / 2

    @property
    def dynamics_residual(self):
        """M a + h + G + D v - B u - J_contactᵀ lambda; set equal to zero."""
        return (
            self.mass_matrix * self.accelerations
            + self.velocity_bias
            + self.gravity
            + self.damping_matrix * self.velocities
            - self.input_matrix * self.inputs
            - self.contact_jacobian.T * self.contact_forces
        )

    def bind_parameters(self, values: Mapping[str, float]) -> dict:
        """Build a validated substitution map; omitted parameters remain symbols."""
        substitutions = {}
        for name, value in values.items():
            if name not in self.parameters:
                raise KeyError(f"Unknown physical parameter: {name}")
            symbol = self.parameters[name]
            if isinstance(value, bool):
                raise ValueError(f"{name} must be a physical number, not a boolean")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if symbol.is_positive and value <= 0:
                raise ValueError(f"{name} must be positive")
            if symbol.is_nonnegative and value < 0:
                raise ValueError(f"{name} must be non-negative")
            substitutions[symbol] = value
        return substitutions

    def bind_configuration(self, values: Sequence[float]) -> dict:
        """Validate both leg assembly branches before numerical state substitution."""
        if len(values) != 7 or not all(math.isfinite(v) for v in values):
            raise ValueError("Configuration must contain seven finite SI coordinates")
        for index in (3, 4):
            pose = solve_leg_pose(self.geometry, values[index])
            leg_jacobian(self.geometry, pose)
        return dict(zip(self.coordinates, values))

    @property
    def contact_conditions(self):
        """Unilateral/Coulomb conditions; no active set or impact law is chosen."""
        import sympy as sp

        n_l, n_r, t_l, t_r = self.contact_forces
        return tuple(
            (
                gap >= 0,
                force >= 0,
                sp.Eq(gap * force, 0),
                sp.Abs(tangent) <= self.parameters[f"mu_{side}"] * force,
            )
            for side, gap, force, tangent in zip(
                ("L", "R"), self.normal_gaps, (n_l, n_r), (t_l, t_r)
            )
        )


def build_planar_biped_model(geometry: LegGeometry) -> PlanarBipedModel:
    """Build ideal planar rigid-body equations on the configured assembly branch.

    Assumptions: rigid four-bars, parallel leg planes, concentric wheel CoMs,
    ideal wheel bearings, uniform gravity, optional viscous relative damping.
    No motor ratio, actuator limits, CAD inertias or controller is selected.
    Contact constraints are exposed, not automatically enforced or solved.
    """
    try:
        import sympy as sp
    except ImportError as exc:
        raise ImportError(
            "Install duk-core[symbolic] to build symbolic dynamics"
        ) from exc

    q = sp.ImmutableMatrix(sp.symbols("x z theta q_L q_R phi_L phi_R", real=True))
    v = sp.ImmutableMatrix(
        sp.symbols("v_x v_z omega dq_L dq_R dphi_L dphi_R", real=True)
    )
    a = sp.ImmutableMatrix(
        sp.symbols("a_x a_z alpha ddq_L ddq_R ddphi_L ddphi_R", real=True)
    )
    parameters = {}

    def parameter(name, **assumptions):
        symbol = sp.Symbol(name, real=True, **assumptions)
        parameters[name] = symbol
        return symbol

    def mm(value):
        return sp.Rational(str(value)) / 1000

    def normal(vec):
        return sp.Matrix([-vec[1], vec[0]])

    def angular_rate(unit, coordinate):
        derivative = unit.diff(coordinate)
        return unit[0] * derivative[1] - unit[1] * derivative[0]

    theta = q[2]
    rotation = sp.Matrix(
        [[sp.cos(theta), -sp.sin(theta)], [sp.sin(theta), sp.cos(theta)]]
    )
    origin = sp.Matrix(q[:2])
    bodies = []
    mass_matrix = sp.zeros(7)
    damping = sp.zeros(7)
    gravity = sp.zeros(7, 1)
    potential = sp.S.Zero
    g = parameter("g_m_s2", positive=True)

    def body(name, center, angular):
        nonlocal mass_matrix, gravity, potential
        mass = parameter(f"m_{name}_kg", positive=True)
        inertia = parameter(f"I_{name}_kg_m2", positive=True)
        jacobian = center.jacobian(q)
        mass_matrix += mass * jacobian.T * jacobian + inertia * angular.T * angular
        gravity += mass * g * jacobian[1, :].T
        potential += mass * g * center[1]
        bodies.append(
            PlanarBodyKinematics(
                name,
                mass,
                inertia,
                center.as_immutable(),
                jacobian.as_immutable(),
                angular.as_immutable(),
            )
        )

    body_com = sp.Matrix([parameter("c_body_x_m"), parameter("c_body_z_m")])
    pitch_row = sp.zeros(1, 7)
    pitch_row[2] = 1
    body("body", origin + rotation * body_com, pitch_row)
    ab, cd, ad, bc, bh, oh, radius = map(
        mm,
        (
            geometry.ab_mm,
            geometry.cd_mm,
            geometry.ad_mm,
            geometry.bc_mm,
            geometry.bh_mm,
            geometry.oh_mm,
            geometry.wheel_radius_mm,
        ),
    )
    da = sp.Float(geometry.da_angle_rad, 17)
    d = -ad * sp.Matrix([sp.cos(da), sp.sin(da)])
    wheels, gaps, normals, rolling = [], [], [], []
    local_points = {}
    input_matrix = sp.zeros(7, 4)

    for side, index, spin_index, input_index in (("L", 3, 5, 0), ("R", 4, 6, 1)):
        coordinate = q[index]
        b = ab * sp.Matrix([sp.cos(da - coordinate), sp.sin(da - coordinate)])
        bd = d - b
        distance_sq = bd.dot(bd)
        # Written with |BD|² to keep the exact circle construction compact.
        fraction = (bc**2 - cd**2 + distance_sq) / (2 * distance_sq)
        offset = sp.sqrt(bc**2 / distance_sq - fraction**2)
        c_sign = 1 if geometry.c_side_of_bd == "left" else -1
        c = b + fraction * bd + c_sign * offset * normal(bd)
        e_bc = (c - b) / bc
        o_sign = 1 if geometry.bco_body.o_side_of_bc == "left" else -1
        h = b - bh * e_bc
        o = h + o_sign * oh * normal(e_bc)
        local_points[side] = {
            "A": sp.zeros(2, 1),
            "B": b,
            "C": c,
            "D": d,
            "H": h,
            "O": o,
        }
        mount = sp.Matrix([parameter(f"a_{side}_x_m"), parameter(f"a_{side}_z_m")])
        carrier_rate = None
        for label, anchor, unit in (
            ("AB", sp.zeros(2, 1), b / ab),
            ("CD", d, (c - d) / cd),
            ("BCO", b, e_bc),
        ):
            u = parameter(f"c_{label}_{side}_u_m")
            n = parameter(f"c_{label}_{side}_n_m")
            center = origin + rotation * (mount + anchor + u * unit + n * normal(unit))
            angular = pitch_row.copy()
            rate = -sp.S.One if label == "AB" else angular_rate(unit, coordinate)
            angular[index] = rate
            body(f"{label}_{side}", center, angular)
            if label == "BCO":
                carrier_rate = rate
        wheel = origin + rotation * (mount + o)
        spin_row = sp.zeros(1, 7)
        spin_row[spin_index] = 1
        body(f"wheel_{side}", wheel, spin_row)
        wheels.append(wheel.as_immutable())
        gaps.append(wheel[1] - radius)
        wheel_jacobian = wheel.jacobian(q)
        normals.append(wheel_jacobian[1, :])
        rolling.append(wheel_jacobian[0, :] + radius * spin_row)
        relative_spin = spin_row - pitch_row
        relative_spin[index] = -carrier_rate
        leg_row = sp.zeros(1, 7)
        leg_row[index] = 1
        damping += (
            parameter(f"b_q_{side}_N_m_s", nonnegative=True) * leg_row.T * leg_row
        )
        damping += (
            parameter(f"b_w_{side}_N_m_s", nonnegative=True)
            * relative_spin.T
            * relative_spin
        )
        parameter(f"mu_{side}", nonnegative=True)
        input_matrix[index, input_index] = 1
        input_matrix[:, input_index + 2] = relative_spin.T

    contact_jacobian = sp.Matrix.vstack(*normals, *rolling)
    return PlanarBipedModel(
        geometry=geometry,
        coordinates=q,
        velocities=v,
        accelerations=a,
        parameters=MappingProxyType(parameters),
        bodies=tuple(bodies),
        wheel_centers=tuple(wheels),
        local_points=MappingProxyType(
            {
                side: MappingProxyType(
                    {name: point.as_immutable() for name, point in points.items()}
                )
                for side, points in local_points.items()
            }
        ),
        mass_matrix=mass_matrix.as_immutable(),
        potential_energy=potential,
        gravity=gravity.as_immutable(),
        damping_matrix=damping.as_immutable(),
        input_matrix=input_matrix.as_immutable(),
        inputs=sp.ImmutableMatrix(sp.symbols("Q_q_L Q_q_R tau_w_L tau_w_R", real=True)),
        normal_gaps=sp.ImmutableMatrix(gaps),
        contact_jacobian=contact_jacobian.as_immutable(),
        contact_forces=sp.ImmutableMatrix(sp.symbols("N_L N_R T_L T_R", real=True)),
    )
