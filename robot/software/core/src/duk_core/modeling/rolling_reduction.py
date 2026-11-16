"""Sagittal reduction for coincident wheel projections on a flat floor."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .leg_differential import leg_second_derivative
from .leg_linkage import LegGeometry, leg_jacobian, solve_leg_pose


@dataclass(frozen=True)
class RollingLift:
    """Full state and the affine acceleration map a = tangent @ reduced_a + bias."""

    configuration: np.ndarray
    velocity: np.ndarray
    tangent: np.ndarray
    bias: np.ndarray


def lift_symmetric_rolling(
    geometry: LegGeometry,
    axle_x: float,
    lean: float,
    axle_speed: float,
    lean_rate: float,
    q: float,
    *,
    mount_xz_m: tuple[float, float],
    pitch_offset: float,
    q_rate: float = 0.0,
    q_acceleration: float = 0.0,
) -> RollingLift:
    """Lift axle x and forward lean to the canonical CCW model.

    The projected mounts must coincide and both legs follow the same q(t).
    Lean is measured relative to a fixed pitch offset. With locked hips
    and the CoM offset as reference, it equals the equivalent balance angle.
    """
    pose = solve_leg_pose(geometry, q)
    wheel = np.asarray(pose.o) / 1000.0 + mount_xz_m
    wheel_jacobian = np.asarray(leg_jacobian(geometry, pose).d_o) / 1000.0
    wheel_rate = wheel_jacobian * q_rate
    wheel_acceleration = wheel_jacobian * q_acceleration
    if q_rate:
        wheel_acceleration += np.asarray(leg_second_derivative(geometry, pose).dd_o) * (
            q_rate**2 / 1000.0
        )

    theta = pitch_offset - lean
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
    normal = np.array([[0.0, -1.0], [1.0, 0.0]])
    radius = geometry.wheel_radius_mm / 1000.0
    configuration = np.array(
        [axle_x, radius, theta, q, q, -axle_x / radius, -axle_x / radius]
    )
    configuration[:2] -= rotation @ wheel

    tangent = np.zeros((7, 2))
    tangent[[0, 5, 6], 0] = [1.0, -1.0 / radius, -1.0 / radius]
    tangent[:2, 1] = rotation @ normal @ wheel
    tangent[2, 1] = -1.0
    velocity = tangent @ [axle_speed, lean_rate]
    velocity[:2] -= rotation @ wheel_rate
    velocity[3:5] = q_rate

    bias = np.zeros(7)
    bias[:2] = rotation @ (
        lean_rate**2 * wheel
        + 2.0 * lean_rate * normal @ wheel_rate
        - wheel_acceleration
    )
    bias[3:5] = q_acceleration
    return RollingLift(configuration, velocity, tangent, bias)


def symmetric_forward_wheel_input(total_torque: float) -> np.ndarray:
    """Split forward torque equally and convert to the core's CCW inputs."""
    return np.array([0.0, 0.0, -total_torque / 2.0, -total_torque / 2.0])


@dataclass(frozen=True)
class ReducedResponse:
    derivative: np.ndarray
    hip_effort: np.ndarray
    normal_force: float
    tangent_force: np.ndarray
    residual: float


class PrescribedSagittal:
    """Project the planar plant with synchronous, prescribed hip motion."""

    def __init__(self, model, pitch_offset: float):
        self.model = model
        self.pitch_offset = pitch_offset
        params = model.parameters
        left = np.array([params["a_L_x_m"], params["a_L_z_m"]])
        right = np.array([params["a_R_x_m"], params["a_R_z_m"]])
        if not np.array_equal(left, right):
            raise ValueError(
                "Reduced rolling requires the same projected mount on both sides"
            )
        self.mount = tuple(left)

    def evaluate(self, state, total_torque, hip_motion):
        """Return forward-lean dynamics and constraint loads at (q, dq, ddq)."""
        x, speed, lean, lean_rate = state
        q, dq, ddq = hip_motion
        lift = lift_symmetric_rolling(
            self.model.model.geometry,
            x,
            lean,
            speed,
            lean_rate,
            q,
            mount_xz_m=self.mount,
            pitch_offset=self.pitch_offset,
            q_rate=dq,
            q_acceleration=ddq,
        )
        terms = self.model.evaluate(lift.configuration, lift.velocity)
        u = symmetric_forward_wheel_input(total_torque)
        tangent = lift.tangent
        rhs = terms.input @ u - terms.bias - terms.mass @ lift.bias
        acc = np.linalg.solve(tangent.T @ terms.mass @ tangent, tangent.T @ rhs)
        residual = terms.mass @ (tangent @ acc) - rhs
        # Equal normal split for the sagittally symmetric comparison. The
        # two traction forces are recovered from their independent wheel rows.
        directions = np.column_stack(
            (terms.input[:, :2], terms.contact[:2].mean(axis=0), terms.contact[2:].T)
        )
        reactions, _, rank, _ = np.linalg.lstsq(directions, residual, rcond=None)
        if rank != 5:
            raise ValueError("Prescribed-motion reactions are not independent")
        return ReducedResponse(
            np.array([speed, acc[0], lean_rate, acc[1]]),
            reactions[:2],
            float(reactions[2]),
            reactions[3:],
            float(np.linalg.norm(directions @ reactions - residual, ord=np.inf)),
        )
