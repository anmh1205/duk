"""Research 3D lift of the canonical four-bar, with thin-disk floor contact.

Generalized velocity is [world linear, world angular, hip L/R, relative spin L/R].
The candidate rigid bodies share sagittal properties with the planar model.
All transverse offsets and inertia components are explicit experiment inputs.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .leg_differential import angle_derivatives
from .leg_linkage import solve_leg_pose
from .planar_numeric import NumericPlanarBiped


def skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _disk_support(axle, radius, axle_rate=None):
    """Lowest disk offset and its geometric rate on a horizontal support."""
    radial = np.array([0.0, 0.0, 1.0]) - axle * axle[2]
    norm = np.linalg.norm(radial)
    if norm < 1e-8:
        raise ValueError(
            "Flat-floor thin-disk contact is undefined for a vertical wheel axle"
        )
    unit = radial / norm
    bottom = -radius * unit
    if axle_rate is None:
        return bottom, np.zeros(3)
    radial_rate = -axle_rate * axle[2] - axle * axle_rate[2]
    bottom_rate = -radius / norm * (radial_rate - unit * (unit @ radial_rate))
    return bottom, bottom_rate


def _angle_bias(x, y, dx, dy, ddx, ddy):
    """Second derivative of atan2(y, x) along a constant-generalized-velocity path."""
    return angle_derivatives((x, y), (dx, dy), (ddx, ddy))[1]


@dataclass(frozen=True)
class SpatialState:
    position: np.ndarray
    rotation: np.ndarray
    joints: np.ndarray
    velocity: np.ndarray

    def advance(self, velocity, step):
        """Advance configuration using the supplied world angular velocity."""
        rotation = (
            Rotation.from_rotvec(step * velocity[3:6]).as_matrix() @ self.rotation
        )
        return SpatialState(
            self.position + step * velocity[:3],
            rotation,
            self.joints + step * velocity[6:],
            velocity,
        )


@dataclass(frozen=True)
class SpatialKinematics:
    centers: np.ndarray
    linear: np.ndarray
    angular: np.ndarray
    inertias: np.ndarray
    contact: np.ndarray
    gaps: np.ndarray
    tasks: np.ndarray
    task_jacobian: np.ndarray
    forward: np.ndarray


@dataclass(frozen=True)
class SpatialDynamics:
    mass: np.ndarray
    bias: np.ndarray
    actuation: np.ndarray
    kinematics: SpatialKinematics
    contact_bias: np.ndarray
    task_bias: np.ndarray


class SpatialBiped:
    """3D candidate with spin-symmetric wheels on horizontal support surfaces."""

    def __init__(
        self,
        planar: NumericPlanarBiped,
        lateral_mounts_m,
        transverse_inertias,
        *,
        support_levels_m=(0.0, 0.0),
    ):
        self.planar = planar
        self.geometry = planar.model.geometry
        self.radius = planar.radius
        self.masses = planar.masses
        self.wheels = list(planar.wheels)
        self.carriers = [planar.names.index("BCO_" + side) for side in ("L", "R")]
        self.upper = [i for i in range(len(planar.names)) if i not in self.wheels]
        self._upper_weights = self.masses[self.upper] / self.masses[self.upper].sum()
        lateral = np.asarray(lateral_mounts_m, dtype=float)
        if (
            lateral.shape != (2,)
            or not np.isfinite(lateral).all()
            or lateral[0] <= lateral[1]
        ):
            raise ValueError("Specify distinct lateral mounts in left/right order")
        self.lateral = np.array([0.0] + [lateral[0]] * 4 + [lateral[1]] * 4)
        self.support_levels = np.asarray(support_levels_m, dtype=float)
        if (
            self.support_levels.shape != (2,)
            or not np.isfinite(self.support_levels).all()
        ):
            raise ValueError("Specify one finite horizontal support level per wheel")
        if set(transverse_inertias) != set(planar.names):
            raise ValueError(
                "Supply the two transverse principal inertias for every rigid body"
            )
        transverse = np.array(
            [transverse_inertias[name] for name in planar.names], dtype=float
        )
        if (
            transverse.shape != (len(planar.names), 2)
            or not np.isfinite(transverse).all()
            or np.any(transverse <= 0)
        ):
            raise ValueError("Transverse inertias must be finite and positive")
        if not np.allclose(
            transverse[self.wheels, 0], transverse[self.wheels, 1], rtol=1e-12, atol=0
        ):
            raise ValueError("This wheel model requires axisymmetric spin inertia")
        principal = np.column_stack(
            (transverse[:, 0], planar.inertias, transverse[:, 1])
        )
        if np.any(2 * principal > principal.sum(axis=1, keepdims=True) + 1e-12):
            raise ValueError(
                "Principal moments must satisfy the inertia triangle inequalities"
            )
        self._inertias = np.array([np.diag(components) for components in principal])
        p = planar.parameters
        self._damping = np.array(
            [
                p[key]
                for key in (
                    "b_q_L_N_m_s",
                    "b_q_R_N_m_s",
                    "b_w_L_N_m_s",
                    "b_w_R_N_m_s",
                )
            ]
        )
        self.actuation = np.eye(10)[:, 6:]
        self.gravity = p["g_m_s2"]

    def sagittal_state(self, configuration, velocity):
        """Map the sagittal pose/rates to spatial coordinates and return the velocity lift."""
        x, z, theta, ql, qr, pl, pr = configuration
        leg_poses = [solve_leg_pose(self.geometry, q) for q in (ql, qr)]
        beta = np.array([np.arctan2(p.e_bc[1], p.e_bc[0]) for p in leg_poses])
        rel_spin = np.array([pl, pr]) - theta - beta
        rows = self.planar.kinematics(configuration).angular_rows[self.carriers]
        tangent = np.zeros((10, 7))
        tangent[0, 0] = tangent[2, 1] = 1
        tangent[4, 2] = -1
        tangent[6:8, 3:5] = np.eye(2)
        tangent[8:, :] = np.eye(7)[5:] - rows
        state = SpatialState(
            np.array([x, 0.0, z]),
            Rotation.from_rotvec([0.0, -theta, 0.0]).as_matrix(),
            np.r_[ql, qr, rel_spin],
            tangent @ velocity,
        )
        return state, tangent

    def kinematics(self, state):
        ql, qr = state.joints[:2]
        local = self.planar.kinematics([0.0, 0.0, 0.0, ql, qr, 0.0, 0.0])
        rotation = state.rotation
        axle = -rotation[:, 1]  # Positive CCW in the sagittal x-z plane.
        relative = (
            np.column_stack((local.centers[:, 0], self.lateral, local.centers[:, 1]))
            @ rotation.T
        )
        centers = relative + state.position
        count = len(self.masses)
        linear = np.zeros((count, 3, 10))
        angular = np.zeros_like(linear)
        linear[:, :, :3] = np.eye(3)
        linear[:, :, 3:6] = -np.array([skew(p) for p in relative])
        angular[:, :, 3:6] = np.eye(3)
        local_derivative = np.zeros((count, 3, 2))
        local_derivative[:, [0, 2], :] = local.jacobians[:, :, 3:5]
        linear[:, :, 6:8] = rotation @ local_derivative
        angular[:, :, 6:8] = axle[None, :, None] * local.angular_rows[:, None, 3:5]
        for side, wheel in enumerate(self.wheels):
            angular[wheel, :, 6:8] = angular[self.carriers[side], :, 6:8]
            angular[wheel, :, 8 + side] = axle

        link_frames = [rotation]
        for q in (ql, qr):
            pose = solve_leg_pose(self.geometry, q)
            for x, z in (pose.b, np.subtract(pose.c, pose.d), pose.e_bc):
                link_frames.append(
                    rotation
                    @ Rotation.from_rotvec([0.0, -np.arctan2(z, x), 0.0]).as_matrix()
                )
            link_frames.append(
                rotation
            )  # A spin-symmetric inertia is independent of spin angle.
        frames = np.asarray(link_frames)
        inertias = frames @ self._inertias @ frames.transpose(0, 2, 1)

        bottom, _ = _disk_support(axle, self.radius)
        contact = linear[self.wheels] - skew(bottom) @ angular[self.wheels]
        gaps = centers[self.wheels, 2] + bottom[2] - self.support_levels

        heading = rotation[:, 0].copy()
        heading[2] = 0.0
        horizontal = np.linalg.norm(heading)
        if horizontal < 1e-8:
            raise ValueError("Heading is undefined for a vertical body forward axis")
        forward = heading / horizontal
        heading_j = -skew(rotation[:, 0])
        heading_j[2] = 0.0
        forward_j = (np.eye(3) - np.outer(forward, forward)) @ heading_j / horizontal
        upper_weights = self._upper_weights
        com_delta = upper_weights @ centers[self.upper] - centers[self.wheels].mean(
            axis=0
        )
        com_j = np.einsum("i,ijk->jk", upper_weights, linear[self.upper]) - linear[
            self.wheels
        ].mean(axis=0)
        dx, dz = com_delta @ forward, com_delta[2]
        dx_j = forward @ com_j
        dx_j[3:6] += com_delta @ forward_j
        lean_j = (dz * dx_j - dx * com_j[2]) / (dx**2 + dz**2)

        roll_y, roll_x = rotation[2, 1], rotation[2, 2]
        roll_j = (
            roll_x * -skew(rotation[:, 1])[2] + roll_y * skew(rotation[:, 2])[2]
        ) / (roll_x**2 + roll_y**2)
        yaw_j = (heading[0] * heading_j[1] - heading[1] * heading_j[0]) / horizontal**2
        task_j = np.zeros((4, 10))
        task_j[0] = lean_j
        task_j[1, 2] = 1.0
        task_j[2, 3:6] = roll_j
        task_j[3, 3:6] = yaw_j
        tasks = np.array(
            [
                np.arctan2(dx, dz),
                state.position[2],
                np.arctan2(roll_y, roll_x),
                np.arctan2(heading[1], heading[0]),
            ]
        )
        return SpatialKinematics(
            centers, linear, angular, inertias, contact, gaps, tasks, task_j, forward
        )

    def rolling_odometry(self, kinematics, velocity):
        """Per-wheel forward axle speed inferred from zero longitudinal slip.

        The difference of the axle and contact rows cancels world translation.
        It retains body, carrier and spin rotation about the tilted disk's
        actual support point, rather than assuming a horizontal wheel axle.
        """
        rotation_rows = kinematics.linear[self.wheels] - kinematics.contact
        return (rotation_rows @ velocity) @ kinematics.forward

    def _body_bias(self, state, kinematics):
        """Rigid-base transport plus the canonical linkage's analytic curvature."""
        v, rotation = state.velocity, state.rotation
        local = self.planar.kinematic_bias(
            np.r_[0.0, 0.0, 0.0, state.joints[:2], 0.0, 0.0],
            np.r_[0.0, 0.0, 0.0, v[6:8], 0.0, 0.0],
        )
        relative = kinematics.centers - state.position
        internal_velocity = kinematics.linear[:, :, 6:8] @ v[6:8]
        local_acceleration = (
            np.column_stack(
                (local.linear[:, 0], np.zeros(len(self.masses)), local.linear[:, 1])
            )
            @ rotation.T
        )
        omega = v[3:6]
        linear = (
            np.cross(omega, np.cross(omega, relative))
            + 2 * np.cross(omega, internal_velocity)
            + local_acceleration
        )
        # Relative wheel spin adds to carrier rotation; its acceleration is zero
        # along this bias evaluation, but the axle direction still rotates.
        angular_curvature = local.angular.copy()
        angular_curvature[self.wheels] = angular_curvature[self.carriers]
        angular = np.cross(omega, kinematics.angular @ v - omega)
        angular += angular_curvature[:, None] * -rotation[:, 1]
        return linear, angular

    def _task_bias(self, state, current, linear_bias):
        """Analytic lean, height, roll and heading convective accelerations."""
        omega, rotation = state.velocity[3:6], state.rotation
        frame_rate = skew(omega) @ rotation
        frame_acceleration = skew(omega) @ frame_rate
        heading = rotation[:, 0].copy()
        heading_rate = frame_rate[:, 0].copy()
        heading_acceleration = frame_acceleration[:, 0].copy()
        heading[2] = heading_rate[2] = heading_acceleration[2] = 0.0
        norm, forward = np.linalg.norm(heading), current.forward
        norm_rate = forward @ heading_rate
        forward_rate = (heading_rate - forward * norm_rate) / norm
        norm_acceleration = (
            heading_rate @ heading_rate + heading @ heading_acceleration - norm_rate**2
        ) / norm
        forward_acceleration = (
            heading_acceleration
            - forward * norm_acceleration
            - 2 * forward_rate * norm_rate
        ) / norm

        def upper_relative(values):
            return self._upper_weights @ values[self.upper] - values[self.wheels].mean(
                axis=0
            )

        delta = upper_relative(current.centers)
        delta_rate = upper_relative(current.linear @ state.velocity)
        delta_acceleration = upper_relative(linear_bias)
        dx = delta @ forward
        dx_rate = delta_rate @ forward + delta @ forward_rate
        dx_acceleration = (
            delta_acceleration @ forward
            + 2 * delta_rate @ forward_rate
            + delta @ forward_acceleration
        )
        return np.array(
            [
                _angle_bias(
                    delta[2],
                    dx,
                    delta_rate[2],
                    dx_rate,
                    delta_acceleration[2],
                    dx_acceleration,
                ),
                0.0,
                _angle_bias(
                    rotation[2, 2],
                    rotation[2, 1],
                    frame_rate[2, 2],
                    frame_rate[2, 1],
                    frame_acceleration[2, 2],
                    frame_acceleration[2, 1],
                ),
                _angle_bias(
                    heading[0],
                    heading[1],
                    heading_rate[0],
                    heading_rate[1],
                    heading_acceleration[0],
                    heading_acceleration[1],
                ),
            ]
        )

    def evaluate(self, state):
        """Mass and analytic velocity biases, without finite-difference probe states."""
        v = state.velocity
        current = self.kinematics(state)
        linear_drift, angular_drift = self._body_bias(state, current)

        jv, jw, inertia = current.linear, current.angular, current.inertias
        mass = np.einsum("i,ijk,ijl->kl", self.masses, jv, jv)
        mass += np.sum(jw.transpose(0, 2, 1) @ inertia @ jw, axis=0)
        linear_bias = linear_drift + [0.0, 0.0, self.gravity]
        angular_velocity = jw @ v
        angular_bias = (inertia @ angular_drift[..., None])[..., 0]
        angular_bias += np.cross(
            angular_velocity, (inertia @ angular_velocity[..., None])[..., 0]
        )
        bias = np.einsum("i,ijk,ij->k", self.masses, jv, linear_bias)
        bias += np.einsum("ijk,ij->k", jw, angular_bias)
        bias[6:] += self._damping * v[6:]
        axle = -state.rotation[:, 1]
        bottom, bottom_rate = _disk_support(axle, self.radius, np.cross(v[3:6], axle))
        contact_bias = (
            linear_drift[self.wheels]
            + np.cross(angular_drift[self.wheels], bottom)
            + np.cross(angular_velocity[self.wheels], bottom_rate)
        )
        return SpatialDynamics(
            mass,
            bias,
            self.actuation,
            current,
            contact_bias,
            self._task_bias(state, current, linear_drift),
        )

    def compliant_step(self, state, effort, friction, law, dt):
        """Semi-implicit step of the independent compliant 3D plant."""
        terms = self.evaluate(state)
        kinematics = terms.kinematics
        speed = kinematics.contact @ state.velocity
        normal, tangent = law.forces(
            kinematics.gaps, speed[:, 2], speed[:, :2], friction
        )
        force = np.column_stack((tangent, normal))
        rhs = (
            terms.actuation @ effort
            - terms.bias
            + kinematics.contact.reshape(6, 10).T @ force.ravel()
        )
        acc = np.linalg.solve(terms.mass, rhs)
        return state.advance(state.velocity + dt * acc, dt), force
