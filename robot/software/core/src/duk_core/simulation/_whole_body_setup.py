"""Initial stance and frozen-posture fallback design for the spatial study."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ..control.lqr import bryson_weights, discrete_lqr
from ..modeling.contact_dynamics import CompliantContact
from ..modeling.rolling_reduction import lift_symmetric_rolling
from ..modeling.spatial_biped import SpatialState
from ..modeling.wheeled_pendulum import WheeledPendulum


@dataclass(frozen=True)
class WholeBodySetup:
    state: SpatialState
    hip_angle: float
    hip_limits: np.ndarray
    home_height: float
    normal_at_rest: float
    contact_law: CompliantContact
    pendulum: WheeledPendulum
    a: np.ndarray
    b: np.ndarray
    gain: np.ndarray
    hip_feedforward: np.ndarray


def initialize(model, weights, settings, scenario) -> WholeBodySetup:
    """Bind the explicit study inputs without selecting new physical values."""
    p, c = settings, scenario
    q0 = np.deg2rad(p["initial_q_deg"])
    period = p["period_s"]
    hip_limits = np.deg2rad(p["hip_limits_deg"])
    law = CompliantContact(**p["contact_law"])
    nominal_body = model.planar.equivalent_upper_body(q0)
    pendulum = WheeledPendulum.from_biped(model.planar, q0)
    mount = tuple(model.planar.parameters[f"a_L_{axis}_m"] for axis in ("x", "z"))
    lift = lift_symmetric_rolling(
        model.geometry,
        0.0,
        np.deg2rad(c["initial_lean_deg"]),
        0.0,
        0.0,
        q0,
        mount_xz_m=mount,
        pitch_offset=nominal_body.angle_from_vertical_rad,
    )
    height_at_rest = lift_symmetric_rolling(
        model.geometry,
        0.0,
        0.0,
        0.0,
        0.0,
        q0,
        mount_xz_m=mount,
        pitch_offset=nominal_body.angle_from_vertical_rad,
    ).configuration[1]
    normal_at_rest = model.masses.sum() * model.gravity / 2
    settled_z = normal_at_rest / law.stiffness
    configuration = lift.configuration.copy()
    configuration[1] -= settled_z
    state, _ = model.sagittal_state(configuration, np.zeros(7))
    home = height_at_rest - settled_z + model.support_levels.mean()
    if np.any(model.support_levels):
        # Symmetric q gives coincident sagittal wheel projections. A roll
        # about world x separates their heights by the lateral track times sin(roll).
        width = -np.diff(model.lateral[model.wheels])[0]
        relative_height = -np.diff(model.support_levels)[0] / width
        if abs(relative_height) >= 1:
            raise ValueError(
                "Split support levels exceed the symmetric starting stance"
            )
        roll = Rotation.from_rotvec([np.arcsin(relative_height), 0.0, 0.0]).as_matrix()
        state = SpatialState(
            state.position, roll @ state.rotation, state.joints, state.velocity
        )
        shift = -model.kinematics(state).gaps.mean() - settled_z
        state = SpatialState(
            state.position + [0.0, 0.0, shift],
            state.rotation,
            state.joints,
            state.velocity,
        )
    hip_feedforward = model.planar.static_hip_effort(
        lift.configuration, [[0.0, normal_at_rest]] * 2
    )
    q_cost, r_cost = bryson_weights(
        weights["state_scales"][1:], weights["torque_scale_N_m"]
    )
    a, b = pendulum.linearization()
    gain = discrete_lqr(a[1:, 1:], b[1:], q_cost, r_cost, period).gain
    return WholeBodySetup(
        state=state,
        hip_angle=q0,
        hip_limits=hip_limits,
        home_height=home,
        normal_at_rest=normal_at_rest,
        contact_law=law,
        pendulum=pendulum,
        a=a,
        b=b,
        gain=gain,
        hip_feedforward=hip_feedforward,
    )
