"""Closed-loop research scenarios for constrained spatial allocation."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

from .lqr import bryson_weights, discrete_lqr
from .sagittal_balance import balance_command, project_torque, traction_interval
from .task_policy import FeatureTransition, Supervisor, return_to_anchor, wrap_angle
from .whole_body import AccelerationTask, Allocation, allocate
from ..estimation.sagittal import SlipGatedVelocity
from ..modeling.contact_dynamics import CompliantContact
from ..modeling.rolling_reduction import (
    lift_symmetric_rolling,
    symmetric_forward_wheel_input,
)
from ..modeling.spatial_biped import SpatialState
from ..modeling.wheeled_pendulum import WheeledPendulum


@dataclass(frozen=True)
class WholeBodyTrace:
    time: np.ndarray
    pose: np.ndarray
    pose_rate: np.ndarray
    position_xy: np.ndarray
    hip_angles: np.ndarray
    contact_gap: np.ndarray
    contact_speed: np.ndarray
    requested_reference: np.ndarray
    effective_reference: np.ndarray
    requested_effort: np.ndarray
    applied_effort: np.ndarray
    effort_limit: np.ndarray
    force: np.ndarray
    allocated_force: np.ndarray
    feature_amount: np.ndarray
    feature_intent: np.ndarray
    true_and_estimated_speed: np.ndarray
    solver_seconds: np.ndarray
    solver_residual: np.ndarray
    solver_status: tuple
    supervisor: tuple
    feature_status: tuple
    contact_modes: tuple
    capacity_reason: tuple
    stop_reason: str | None


def run_whole_body(model, hip_motor, weights, settings, scenario, *, method, priority):
    """Compare allocation policies on the same compliant spatial plant.

    Sensor inputs are simulated. The independent velocity channel represents
    already-compensated axle acceleration and pose is supplied by the simulator.
    The supervisor and its zero-command stop are policies for this study.
    """
    p, c = settings, scenario
    if priority not in {"balance_first", "legs_first"}:
        raise ValueError("Unknown task priority")
    q0 = np.deg2rad(p["initial_q_deg"])
    period = p["period_s"]
    substeps = p["substeps"]
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
    features = [
        FeatureTransition() for _ in range(4)
    ]  # Bounce, level, anchor, heading.
    supervisor = Supervisor(
        p["fallback_after"], p["recover_after"], successes=p["recover_after"]
    )
    observer = SlipGatedVelocity(0.0, p["slip_threshold_m_s"], 2, 0.08)
    rng = np.random.default_rng(p["sensor_seed"])
    friction_belief = np.full(2, c["friction"] * p["friction_reserve"])
    previous_force = np.array([[0.0, 0.0, normal_at_rest]] * 2)
    warm_start, anchor, heading_anchor = None, None, None
    last_speed, forward_estimate, flying_ticks = 0.0, 0.0, 0
    rows, solver_states, supervisors, feature_states, modes_log = [], [], [], [], []
    capacity_reasons = []
    previous_usage = 0.0
    stop_reason = None

    for tick in range(round(c["duration_s"] / period)):
        t = tick * period
        if tick == round(c["impulse_at_s"] / period):
            velocity = state.velocity.copy()
            velocity[:3] += c["velocity_impulse_m_s"]
            velocity[3:6] += c["angular_velocity_impulse_rad_s"]
            state = SpatialState(state.position, state.rotation, state.joints, velocity)
        fault = c["fault_start_s"] <= t < c["fault_end_s"]
        friction = np.full(
            2, c["friction_after"] if t >= c["friction_change_s"] else c["friction"]
        )
        terms = model.evaluate(state)
        k = terms.kinematics
        rate = k.task_jacobian @ state.velocity
        lean, height, roll, yaw = k.tasks
        c_speed = k.contact @ state.velocity
        supported = (k.gaps <= p["contact_gap_m"]) & (previous_force[:, 2] > 0)
        current_speed = float(
            k.forward @ (k.linear[model.wheels] @ state.velocity).mean(axis=0)
        )
        wheel_rates = (k.angular[model.wheels] @ state.velocity) @ -state.rotation[:, 1]
        if tick:
            independent = (current_speed - last_speed) / period + rng.normal(
                0.0, p["accel_noise_m_s2"]
            )
            estimate = observer.step(
                period,
                independent,
                -model.radius * wheel_rates.mean(),
                in_contact=bool(supported.all()),
            )
            forward_estimate = estimate.speed
            longitudinal_slip = estimate.slip_detected
        else:
            longitudinal_slip = False
        last_speed = current_speed
        sliding = (
            np.linalg.norm(c_speed[:, :2], axis=1) > p["spatial_slip_threshold_m_s"]
        )
        sliding |= longitudinal_slip & (abs(c_speed[:, 0]) > p["slip_threshold_m_s"])
        modes = tuple(
            "flight" if not on_ground else "sliding" if slip else "rolling"
            for on_ground, slip in zip(supported, sliding)
        )
        valid_force = sliding & (previous_force[:, 2] > p["minimum_force_N"])
        ratios = (
            np.sum(abs(previous_force[valid_force, :2]), axis=1)
            / previous_force[valid_force, 2]
        )
        friction_belief[valid_force] = np.minimum(
            friction_belief[valid_force], p["friction_reserve"] * ratios
        )

        flying_ticks = flying_ticks + 1 if not supported.any() else 0
        sensor_valid = not (fault and c["fault"] == "sensor")
        secondary_available = (
            all(mode == "rolling" for mode in modes)
            and sensor_valid
            and supervisor.mode == "normal"
        )
        bounce_on = c["feature_start_s"] <= t < c["bounce_off_s"]
        level_on = c["level_hold"] and t >= c["feature_start_s"]
        anchor_on = c["anchor"] and t >= c["feature_start_s"]
        heading_on = c["heading_hold"] and t >= c["feature_start_s"]
        if heading_on and heading_anchor is None:
            heading_anchor = yaw
        elif not heading_on:
            heading_anchor = None
        if anchor_on and anchor is None:
            anchor = state.position[:2].copy()
        elif not anchor_on:
            anchor = None
        headroom = 1 / (
            1
            + p["governor_gain"] * max(0.0, previous_usage - p["governor_utilization"])
        )
        capacity = min(c["reference_capacity"], headroom)
        if not sensor_valid or supervisor.mode != "normal":
            capacity_reason = "supervisor"
        elif not all(mode == "rolling" for mode in modes):
            capacity_reason = "contact_or_slip"
        elif headroom < c["reference_capacity"]:
            capacity_reason = "actuator_headroom"
        elif capacity < 1:
            capacity_reason = "scenario_cap"
        else:
            capacity_reason = "available"
        intents = (bounce_on, level_on, anchor_on, heading_on)
        amounts = [
            transition.step(
                enabled, secondary_available, capacity, period, p["feature_ramp_s"]
            )
            for transition, enabled in zip(features, intents)
        ]
        if not secondary_available:
            # Exit is a user transition. Safety gates drop optional contributions.
            for transition in features:
                transition.amount = 0.0
                if transition.status == "Exit":
                    transition.status = "Idle"
            amounts = [0.0] * len(features)

        phase = 2 * np.pi * c["bounce_hz"] * max(0.0, t - c["feature_start_s"])
        omega = 2 * np.pi * c["bounce_hz"]
        wave = c["bounce_amplitude_m"] * np.sin(phase)
        h_ref = home + amounts[0] * wave
        h_rate = amounts[0] * c["bounce_amplitude_m"] * omega * np.cos(phase)
        h_acc = -amounts[0] * omega**2 * wave
        requested_yaw_rate = c["turn_rate_rad_s"] if t < c["turn_end_s"] else 0.0
        requested_speed = c["drive_speed_m_s"] if t < c["drive_end_s"] else 0.0
        speed_ref, yaw_rate_ref = (
            (requested_speed, requested_yaw_rate) if secondary_available else (0.0, 0.0)
        )
        if secondary_available and abs(speed_ref) + abs(yaw_rate_ref) < 1e-12:
            if amounts[2] > 0 and anchor is not None:
                speed_ref, yaw_rate_ref = return_to_anchor(
                    state.position[:2],
                    anchor,
                    yaw,
                    speed_gain=p["anchor_speed_gain"],
                    yaw_gain=p["anchor_yaw_gain"],
                    speed_limit=p["anchor_speed_limit_m_s"],
                    yaw_limit=p["anchor_yaw_limit_rad_s"],
                    position_tolerance=p["anchor_tolerance_m"],
                )
                speed_ref *= amounts[2]
                yaw_rate_ref *= amounts[2]
            if (
                abs(speed_ref) + abs(yaw_rate_ref) < 1e-12
                and amounts[3] > 0
                and heading_anchor is not None
            ):
                yaw_rate_ref = amounts[3] * np.clip(
                    p["anchor_yaw_gain"] * wrap_angle(heading_anchor - yaw),
                    -p["anchor_yaw_limit_rad_s"],
                    p["anchor_yaw_limit_rad_s"],
                )
        balance_state = [0.0, forward_estimate, lean, rate[0]]
        direct = balance_command(gain, balance_state, target_speed=speed_ref)
        desired_lean = float(
            a[3] @ [0.0, forward_estimate - speed_ref, lean, rate[0]] + b[3, 0] * direct
        )
        desired_height = (
            h_acc
            + p["height_kp"] * (h_ref - height)
            + p["height_kd"] * (h_rate - rate[1])
        )
        desired_roll = -p["level_kp"] * roll - p["level_kd"] * rate[2]
        hip_delta = state.joints[0] - state.joints[1]
        hip_rate_delta = state.velocity[6] - state.velocity[7]
        desired_symmetry = -p["level_kp"] * hip_delta - p["level_kd"] * hip_rate_delta
        roll_row = amounts[1] * k.task_jacobian[2] + (1 - amounts[1]) * (
            np.eye(10)[6] - np.eye(10)[7]
        )
        tasks = [
            AccelerationTask(
                "lean",
                k.task_jacobian[0],
                desired_lean - terms.task_bias[0],
                p["task_scales"][0],
            ),
            AccelerationTask(
                "height",
                k.task_jacobian[1],
                desired_height - terms.task_bias[1],
                p["task_scales"][1],
            ),
            AccelerationTask(
                "level_or_symmetry",
                roll_row,
                amounts[1] * (desired_roll - terms.task_bias[2])
                + (1 - amounts[1]) * desired_symmetry,
                p["task_scales"][2],
            ),
            AccelerationTask(
                "yaw_rate",
                k.task_jacobian[3],
                p["yaw_kd"] * (yaw_rate_ref - rate[3]) - terms.task_bias[3],
                p["task_scales"][3],
            ),
        ]
        if priority == "legs_first":
            tasks = [tasks[i] for i in (1, 2, 0, 3)]

        effort_bounds = np.r_[
            [hip_motor.joint_limit(v) * c["hip_capacity"] for v in state.velocity[6:8]],
            np.full(2, p["wheel_limit_N_m"]),
        ]
        start = perf_counter()
        if fault and c["fault"] == "deadline":
            # Inject a late return so that this case is independent of CPU speed.
            result = Allocation("deadline", None, np.nan, 0.0)
        else:
            if fault and c["fault"] == "infeasible":
                # Try an actual infeasible problem: all joints disabled and
                # a contradictory forced contact acceleration on the same model.
                disabled = np.zeros(4)
                result = allocate(
                    terms,
                    state,
                    tasks,
                    modes,
                    friction=np.zeros(2),
                    effort_bounds=disabled,
                    hip_limits=(hip_limits[1] + 0.1, hip_limits[1] + 0.2),
                    hip_speed_limit=hip_motor.no_load_speed,
                    lookahead=p["lookahead_s"],
                    velocity_stabilization=p["contact_stabilization"],
                    deadline_s=p["deadline_s"],
                    method=method,
                )
            else:
                result = allocate(
                    terms,
                    state,
                    tasks,
                    modes,
                    friction=friction_belief,
                    effort_bounds=effort_bounds,
                    hip_limits=hip_limits,
                    hip_speed_limit=hip_motor.no_load_speed,
                    lookahead=p["lookahead_s"],
                    velocity_stabilization=p["contact_stabilization"],
                    deadline_s=p["deadline_s"],
                    method=method,
                    initial=warm_start,
                )
        solve_seconds = perf_counter() - start
        solver_status = result.status
        fallback_valid = (
            all(mode == "rolling" for mode in modes)
            and abs(roll) < np.deg2rad(p["fallback_roll_deg"])
            and np.max(abs(state.joints[:2] - q0))
            < np.deg2rad(p["fallback_hip_error_deg"])
            and np.all(state.joints[:2] > hip_limits[0])
            and np.all(state.joints[:2] < hip_limits[1])
        )
        fallen = (
            max(abs(lean), abs(roll)) > np.deg2rad(p["fall_deg"])
            or flying_ticks * period > p["max_flight_s"]
        )
        mode = supervisor.step(
            solver_status,
            sensor_valid=sensor_valid,
            supported=all(m == "rolling" for m in modes),
            fallback_valid=fallback_valid,
            fall=fallen,
        )
        if result.solution is not None:
            requested = result.effort
            allocated_force = result.solution[14:].reshape(2, 3)
            warm_start = result.solution
        else:
            requested = np.full(4, np.nan)
            allocated_force = np.full((2, 3), np.nan)
            warm_start = None
        if mode == "stop":
            effort = np.zeros(4)
        elif result.solution is not None:
            effort = result.effort
        elif fallback_valid:
            bound = 2 * min(effort_bounds[2:])
            interval = traction_interval(
                pendulum, lean, rate[0], bound, min(friction_belief)
            )
            torque = project_torque(direct, bound, interval)
            if torque.applied is None:
                effort = np.zeros(4)
                mode = supervisor.mode = "stop"
            else:
                effort = symmetric_forward_wheel_input(torque.applied)
                effort[:2] = np.clip(
                    hip_feedforward
                    + p["fallback_hip_kp"] * (q0 - state.joints[:2])
                    - p["fallback_hip_kd"] * state.velocity[6:8],
                    -effort_bounds[:2],
                    effort_bounds[:2],
                )
                requested = symmetric_forward_wheel_input(direct)
        else:
            effort = np.zeros(4)
            mode = supervisor.mode = "stop"

        requested_h = home + wave if bounce_on else home
        previous_usage = np.max(
            np.divide(
                abs(effort), effort_bounds, out=np.zeros(4), where=effort_bounds > 0
            )
        )
        rows.append(
            (
                t,
                k.tasks,
                rate,
                state.position[:2].copy(),
                state.joints[:2].copy(),
                k.gaps,
                c_speed,
                [requested_h, requested_yaw_rate, requested_speed],
                [h_ref, yaw_rate_ref, speed_ref],
                requested,
                effort,
                effort_bounds,
                previous_force.copy(),
                allocated_force,
                amounts,
                intents,
                (current_speed, forward_estimate),
                solve_seconds,
                result.residual,
            )
        )
        solver_states.append(solver_status)
        supervisors.append(mode)
        feature_states.append(tuple(feature.status for feature in features))
        modes_log.append(modes)
        capacity_reasons.append(capacity_reason)
        if mode == "stop":
            stop_reason = (
                "sensor"
                if not sensor_valid
                else "flight_or_fall"
                if fallen
                else "no_valid_command"
            )
            break
        for _ in range(substeps):
            state, previous_force = model.compliant_step(
                state, effort, friction, law, period / substeps
            )
            if np.any(state.joints[:2] < hip_limits[0]) or np.any(
                state.joints[:2] > hip_limits[1]
            ):
                stop_reason = "hip_travel"
                break
        if stop_reason is not None:
            break
    return WholeBodyTrace(
        *map(np.asarray, zip(*rows)),
        tuple(solver_states),
        tuple(supervisors),
        tuple(feature_states),
        tuple(modes_log),
        tuple(capacity_reasons),
        stop_reason,
    )
