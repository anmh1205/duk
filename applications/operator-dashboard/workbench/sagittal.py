"""Workbench adapters for the existing NB3, NB4 and NB5 studies."""

from __future__ import annotations

import math
from functools import cached_property

import numpy as np
from scipy.spatial.transform import Rotation
from simulation_adapter import DashboardSimulation

from duk_core.control.lqr import bryson_weights
from duk_core.estimation.sagittal import (
    SlipGatedVelocity,
    absolute_wheel_rate,
    replay_velocity,
)
from duk_core.modeling.contact_dynamics import CompliantContact
from duk_core.modeling.rolling_reduction import (
    lift_symmetric_rolling,
    symmetric_forward_wheel_input,
)
from duk_core.modeling.trajectories import HipTrajectory
from duk_core.simulation.height import compare_height_policies
from duk_core.simulation.planar import simulate_planar

from .inputs import ROOT, Inputs, integer, merged, number, vector
from .kinematics import linkage_signals
from .records import RunRecord, Signal, event, plain, transitions
from .spatial import SIDES, spatial_model

PLANAR_ORDER = [
    "x [m]",
    "z [m]",
    "θ CCW [rad]",
    "qL [rad]",
    "qR [rad]",
    "φL CCW [rad]",
    "φR CCW [rad]",
]
PLANAR_PRESETS = ("rolling", "compliant", "slip", "flight", "rolling_limit")
HEIGHT_POLICIES = ("fixed", "scheduled", "tv_lqr", "mpc")


def sagittal_replay(inputs: Inputs, state: np.ndarray, rates: np.ndarray) -> dict:
    model = spatial_model(inputs, {"support_levels_m": [0.0, 0.0]})
    spatial = [model.sagittal_state(q, v)[0] for q, v in zip(state, rates)]
    kinematics = [model.kinematics(s) for s in spatial]
    return {
        "position": Signal("m", "body", plain([s.position for s in spatial]), "world"),
        "orientation_xyzw": Signal(
            "1",
            "body",
            plain(Rotation.from_matrix([s.rotation for s in spatial]).as_quat()),
            "body → world",
        ),
        "state_joints": Signal(
            "rad",
            "robot",
            plain([s.joints for s in spatial]),
            "hip L/R, carrier-relative spin L/R",
        ),
        "height": Signal("m", "body", plain(state[:, 1]), "world"),
        "pitch": Signal("rad", "body", plain(-state[:, 2]), "world"),
        "center_of_mass": Signal(
            "m",
            "body",
            plain(
                [
                    np.average(k.centers, axis=0, weights=model.masses)
                    for k in kinematics
                ]
            ),
            "world",
        ),
        **{
            side + "_wheel_center": Signal(
                "m",
                side + ".wheel",
                plain([k.centers[model.wheels[i]] for k in kinematics]),
                "world",
            )
            for i, side in enumerate(SIDES)
        },
    }


class SagittalStudies:
    def __init__(self, inputs: Inputs) -> None:
        self.inputs = inputs

    @cached_property
    def balance(self) -> DashboardSimulation:
        return DashboardSimulation(ROOT, self.inputs.profile_path)

    def defaults(self, study: str, case: str | None = None) -> dict:
        data = self.inputs.fixture
        if study == "NB4":
            choices = {"dry": {}, **self.balance.presets}
            case = case or "dry"
            if case not in choices:
                raise ValueError("Unknown NB4 preset")
            return {
                "study": study,
                "case": case,
                **self.balance.defaults,
                **choices[case],
            }
        if study == "NB5":
            p = data["height_experiment"]
            case = case or "slow"
            if case not in ("slow", "fast", "stationary"):
                raise ValueError("Unknown NB5 preset")
            return {
                "study": study,
                "case": case,
                "controller": "fixed",
                "duration_s": p["duration_s"],
                "period_s": p["period_s"],
                "lift_s": p["fast_duration_s"]
                if case == "fast"
                else p["slow_duration_s"],
                "initial_q_deg": p["initial_q_deg"],
                "target_q_deg": p["initial_q_deg"]
                if case == "stationary"
                else p["final_q_deg"],
                "initial_lean_deg": p["initial_lean_deg"],
                "friction": p["friction"],
                "total_wheel_limit_N_m": p["total_wheel_limit_N_m"],
                "mpc_horizon": p["mpc_horizon"],
            }
        if study != "NB3":
            raise ValueError("Unknown sagittal study")
        case = case or "rolling"
        if case not in PLANAR_PRESETS:
            raise ValueError("Unknown NB3 preset")
        p = data["planar_contact"]
        friction = (
            p["high_friction"]
            if case in ("rolling", "compliant")
            else p["low_friction"]
        )
        return {
            "study": study,
            "case": case,
            "mode": {"slip": "compliant", "rolling_limit": "rolling"}.get(case, case),
            "duration_s": 0.20,
            "period_s": 0.0025,
            "max_step_s": 0.003,
            "initial_q_deg": data["whole_body"]["initial_q_deg"],
            "initial_axle_m_s": 0.0,
            "initial_lean_deg": 0.0,
            "drop_height_m": 0.1 if case == "flight" else 0.0,
            "friction": [friction, friction],
            "total_wheel_torque_N_m": 1.8 if case == "slip" else 0.10,
            "wheel_torque_ramp_N_m_s": 8.5 if case == "rolling_limit" else 0.0,
            "feedback": "gated",
            "sensor_seed": p["sensor_seed"],
            "accel_noise_m_s2": p["acceleration_noise_m_s2"],
            "encoder_noise_rad_s": p["encoder_noise_rad_s"],
        }

    def run(self, study: str, options: dict) -> RunRecord:
        case = options.get("case")
        if case is not None and not isinstance(case, str):
            raise ValueError("Preset name must be text")
        config = merged(self.defaults(study, case), options)
        if study == "NB4":
            return self.fixed_hip(config)
        duration = number(config["duration_s"], "duration_s", 0.001, 60)
        period = number(config["period_s"], "period_s", 0.001, 1)
        if math.ceil(duration / period) > 5_000:
            raise ValueError("At most 5,000 sample intervals")
        return (
            self.height(config, duration, period)
            if study == "NB5"
            else self.planar(config, duration, period)
        )

    def fixed_hip(self, config: dict) -> RunRecord:
        response = self.balance.balance(
            {k: v for k, v in config.items() if k in self.balance.defaults}
        )
        config = config | response["configuration"]
        trace = response["trace"]
        time = trace["time"]
        state = np.array(trace["state"])
        signals = {
            key: Signal(unit, scope, plain(values), frame, held)
            for key, unit, scope, values, frame, held in (
                ("axle_position", "m", "body", state[:, 0], "world x", False),
                ("model_speed", "m/s", "body", state[:, 1], "world x", False),
                ("lean", "rad", "body", state[:, 2], "+x", False),
                ("lean_rate", "rad/s", "body", state[:, 3], "+x", False),
                ("wheel_spin", "rad", "wheel_pair", state[:, 4], "+x", False),
                ("wheel_spin_rate", "rad/s", "wheel_pair", state[:, 5], "+x", False),
                ("slip_speed", "m/s", "wheel_pair", trace["slip"], "world x", False),
                ("normal_force", "N", "wheel_pair", trace["normal"], "total", False),
                (
                    "tangential_force",
                    "N",
                    "wheel_pair",
                    trace["tangent"],
                    "total, world x",
                    False,
                ),
                (
                    "requested",
                    "N·m",
                    "wheel_pair",
                    trace["requested"],
                    "total +x",
                    True,
                ),
                ("limited", "N·m", "wheel_pair", trace["limited"], "total +x", True),
                ("applied", "N·m", "wheel_pair", trace["applied"], "total +x", True),
                (
                    "odometry_speed",
                    "m/s",
                    "body",
                    trace["wheel_odometry"],
                    "world x",
                    False,
                ),
                (
                    "estimated_speed",
                    "m/s",
                    "body",
                    trace["speed_estimate"],
                    "world x",
                    False,
                ),
                ("sensed_lean", "rad", "body", trace["sensed_lean"], "+x", False),
                (
                    "sensed_acceleration",
                    "m/s²",
                    "body",
                    trace["sensed_acceleration"],
                    "+x",
                    False,
                ),
                ("slip_detected", "1", "control", trace["slip_detected"], "", False),
                ("limit_reason", "1", "control", trace["limit_reason"], "", False),
            )
        }
        outcome = trace["outcome"]
        return RunRecord(
            "NB4",
            config,
            self.inputs.provenance(),
            time,
            signals,
            outcome,
            transitions(time, trace["limit_reason"], "Giới hạn")
            + [event(outcome, None, time[-1])],
            {
                "parameters": {
                    k: response[k] for k in ("plant", "simulation", "design_inputs")
                },
                "linear_model": response["linear_model"],
                "assumptions": "Aggregate fixed-hip pendulum. Torques refer to both wheels together.",
            },
        )

    def planar(self, config: dict, duration: float, period: float) -> RunRecord:
        inputs, mode = self.inputs, config["mode"]
        model = inputs.planar
        if mode not in ("rolling", "compliant", "flight"):
            raise ValueError("NB3 contact law must be rolling, compliant or flight")
        feedback = config["feedback"]
        if feedback not in ("truth", "odometry", "gated"):
            raise ValueError("NB3 sensor replay must be truth, odometry or gated")
        p = inputs.fixture["planar_contact"]
        law = CompliantContact(p["stiffness"], p["damping"], p["slip_scale"])
        limits = tuple(
            map(math.degrees, inputs.geometry.geometric_linkage_angle_limits_rad())
        )
        q = math.radians(number(config["initial_q_deg"], "initial_q_deg", *limits))
        lift = lift_symmetric_rolling(
            inputs.geometry,
            0,
            math.radians(number(config["initial_lean_deg"], "initial_lean_deg")),
            number(config["initial_axle_m_s"], "initial_axle_m_s"),
            0,
            q,
            mount_xz_m=tuple(model.parameters[f"a_L_{axis}_m"] for axis in ("x", "z")),
            pitch_offset=model.equivalent_upper_body(q).angle_from_vertical_rad,
        )
        initial = np.r_[lift.configuration, lift.velocity]
        preload = model.masses.sum() * model.parameters["g_m_s2"] / 2
        drop = number(config["drop_height_m"], "drop_height_m", 0)
        if drop and mode != "flight":
            raise ValueError("drop_height_m applies to the flight mode")
        if mode == "flight":
            initial[1] += drop
        elif mode == "compliant":
            initial[1] -= preload / law.stiffness
        static = model.static_hip_effort(lift.configuration, [[0.0, preload]] * 2)
        friction = vector(config["friction"], "friction", 2, 0)
        torque = number(config["total_wheel_torque_N_m"], "total_wheel_torque_N_m")
        ramp = number(config["wheel_torque_ramp_N_m_s"], "wheel_torque_ramp_N_m_s")
        max_step = number(config["max_step_s"], "max_step_s", 0.00001, 1)
        seed = integer(config["sensor_seed"], "sensor_seed", 0, 2**32 - 1)
        accel_noise = number(config["accel_noise_m_s2"], "accel_noise_m_s2", 0)
        encoder_noise = number(config["encoder_noise_rad_s"], "encoder_noise_rad_s", 0)

        def effort(time, state):
            if mode == "flight":
                return np.zeros(4)
            command = symmetric_forward_wheel_input(torque + time * ramp)
            command[:2] = (
                static
                + p["hip_hold_kp"] * (q - state[3:5])
                - p["hip_hold_kd"] * state[10:12]
            )
            return command

        times = np.minimum(
            np.arange(math.ceil(duration / period) + 1) * period, duration
        )
        trace = simulate_planar(
            model,
            initial,
            np.unique(times),
            effort,
            friction,
            mode=mode,
            contact_law=law,
            max_step=max_step,
            rtol=1e-7,
            atol=1e-9,
        )
        times = trace.time.tolist()
        states, rates = np.hsplit(trace.state, 2)
        signals = sagittal_replay(inputs, states, rates)
        signals.update(linkage_signals(inputs, signals))

        def add(name, unit, scope, data, frame=""):
            signals[name] = Signal(unit, scope, plain(data), frame)

        rng = np.random.default_rng(seed)
        encoder = trace.relative_spin + rng.normal(
            0, encoder_noise, trace.relative_spin.shape
        )
        wheel_speed = -model.radius * absolute_wheel_rate(
            encoder, rates[:, 2, None], trace.carrier_rate
        )
        accel = trace.axle_acceleration.mean(axis=1) + rng.normal(
            0, accel_noise, len(times)
        )
        truth = trace.axle_speed.mean(axis=1)
        on_ground = trace.contact_forces[:, :2] > 0
        gated = replay_velocity(
            trace.time,
            accel,
            wheel_speed.mean(axis=1),
            on_ground.all(axis=1),
            SlipGatedVelocity(float(truth[0]), p["slip_threshold_m_s"], 2, 0.08),
        )
        estimate = {
            "truth": truth,
            "odometry": wheel_speed.mean(axis=1),
            "gated": [r.speed for r in gated],
        }
        add("model_speed", "m/s", "body", truth)
        add("estimated_speed", "m/s", "body", estimate[feedback])
        add("sensed_acceleration", "m/s²", "body", accel)
        add("odometry_speed", "m/s", "body", wheel_speed.mean(axis=1))
        add("slip_detected", "1", "control", [r.slip_detected for r in gated])
        add("state_7dof", "mixed", "robot", states, "sagittal")
        add("rate_7dof", "mixed/s", "robot", rates, "sagittal")
        add("energy", "J", "body", trace.energy)
        add("force_admissible", "1", "control", trace.force_admissible)
        wheel_centers = [
            [signals[side + "_wheel_center"].values[i] for side in SIDES]
            for i in range(len(times))
        ]
        events = []
        for i, side in enumerate(SIDES):
            wheel = side + ".wheel"
            modes = [
                "flight"
                if not contact
                else "sliding"
                if abs(slip) > p["slip_threshold_m_s"]
                else "rolling"
                for contact, slip in zip(on_ground[:, i], trace.slip[:, i])
            ]
            events += transitions(times, modes, side)
            for name, unit, values in (
                ("contact_mode", "1", modes),
                ("normal_force", "N", trace.contact_forces[:, i]),
                ("tangential_force", "N", trace.contact_forces[:, i + 2]),
                ("slip_speed", "m/s", trace.slip[:, i]),
                ("wheel_velocity", "m/s", trace.axle_speed[:, i]),
                ("wheel_acceleration", "m/s²", trace.axle_acceleration[:, i]),
                (
                    "gap",
                    "m",
                    [centers[i][2] - model.radius for centers in wheel_centers],
                ),
                ("spin_absolute", "rad", states[:, i + 5]),
                ("spin_relative_rate", "rad/s", trace.relative_spin[:, i]),
                ("wheel_applied", "N·m", trace.applied[:, i + 2]),
                ("friction", "1", [friction[i]] * len(times)),
            ):
                add(side + "_" + name, unit, wheel, values)
            add(
                side + "_contact_point",
                "m",
                wheel,
                [
                    [centers[i][0], centers[i][1], centers[i][2] - model.radius]
                    for centers in wheel_centers
                ],
                "world",
            )
            add(side + "_hip_angle", "rad", side + ".hip", states[:, i + 3])
            add(side + "_hip_applied", "N·m", side + ".hip", trace.applied[:, i])
        events.append(event(trace.termination, None, times[-1]))
        return RunRecord(
            "NB3",
            config,
            inputs.provenance(),
            times,
            signals,
            trace.termination,
            events,
            {
                "parameters": {
                    "planar": dict(model.parameters),
                    "contact": p,
                    "rtol": 1e-7,
                    "atol": 1e-9,
                },
                "coordinate_order": PLANAR_ORDER,
            },
        )

    def height(self, config: dict, duration: float, period: float) -> RunRecord:
        inputs = self.inputs
        method = config["controller"]
        if method not in HEIGHT_POLICIES:
            raise ValueError(
                "NB5 accepts fixed, scheduled, tv_lqr or mpc total-wheel effort"
            )
        limits = tuple(
            map(math.degrees, inputs.geometry.geometric_linkage_angle_limits_rad())
        )
        q0, q1 = [
            math.radians(number(config[k], k, *limits))
            for k in ("initial_q_deg", "target_q_deg")
        ]
        trajectory = HipTrajectory(
            q0, q1, number(config["lift_s"], "lift_s", 0.001, 60)
        )
        v2 = inputs.fixture["v2_balance"]
        q_weight, r_weight = bryson_weights(v2["state_scales"], v2["torque_scale_N_m"])
        trace = compare_height_policies(
            inputs.planar,
            trajectory,
            inputs.motor,
            period=period,
            duration=duration,
            friction=number(config["friction"], "friction", 0.000001),
            motor_bound=number(
                config["total_wheel_limit_N_m"], "total_wheel_limit_N_m", 0.000001
            ),
            q_weight=q_weight,
            r_weight=r_weight,
            initial_lean=math.radians(
                number(config["initial_lean_deg"], "initial_lean_deg")
            ),
            horizon=integer(config["mpc_horizon"], "mpc_horizon", 1, 50),
        )[method]
        time = trace.time.tolist()
        end = float(trace.terminal_time)
        append_end = end > time[-1]
        if append_end:
            time.append(end)
        states = [*trace.state, trace.terminal_state] if append_end else trace.state
        offsets = inputs.planar.equivalent_upper_body(q0).angle_from_vertical_rad
        mount = tuple(inputs.planar.parameters[f"a_L_{axis}_m"] for axis in ("x", "z"))
        lifts = [
            lift_symmetric_rolling(
                inputs.geometry,
                x,
                lean,
                speed,
                rate,
                motion[0],
                mount_xz_m=mount,
                pitch_offset=offsets,
                q_rate=motion[1],
                q_acceleration=motion[2],
            )
            for t, (x, speed, lean, rate) in zip(time, states)
            for motion in [trajectory.evaluate(t)]
        ]
        q7, v7 = [
            np.array([getattr(lift, name) for lift in lifts])
            for name in ("configuration", "velocity")
        ]
        signals = sagittal_replay(inputs, q7, v7)
        signals.update(linkage_signals(inputs, signals))

        def add(name, unit, scope, data, held=False):
            signals[name] = Signal(unit, scope, plain(data), held=held)

        def sampled(values):
            return plain(values) + ([None] if append_end else [])

        add("state_7dof", "mixed", "robot", q7)
        add("rate_7dof", "mixed/s", "robot", v7)
        add(
            "lean",
            "rad",
            "body",
            np.r_[trace.lean, trace.terminal_lean] if append_end else trace.lean,
        )
        add("model_speed", "m/s", "body", np.array(states)[:, 1])
        add("requested", "N·m", "wheel_pair", sampled(trace.requested), True)
        add("applied", "N·m", "wheel_pair", sampled(trace.applied), True)
        add("grip_usage", "1", "control", sampled(trace.grip_usage))
        add("solver_status", "1", "control", sampled(trace.status))
        add("solver_seconds", "s", "control", sampled(trace.compute_time))
        add("linear_a", "mixed", "control", sampled(trace.linear_a))
        add("linear_b", "mixed", "control", sampled(trace.linear_b))
        add("gain", "mixed", "control", sampled(trace.gain))
        add(
            "nominal_force_constraints",
            "mixed",
            "control",
            sampled(trace.nominal_force_constraints),
        )
        for i, side in enumerate(SIDES):
            wheel = side + ".wheel"
            hip = side + ".hip"
            add(side + "_hip_angle", "rad", hip, q7[:, i + 3])
            add(side + "_hip_rate", "rad/s", hip, v7[:, i + 3])
            add(
                side + "_hip_applied", "N·m", hip, sampled(trace.hip_effort[:, i]), True
            )
            add(
                side + "_hip_limit",
                "N·m",
                hip,
                [inputs.motor.joint_limit(rate) for rate in v7[:, i + 3]],
            )
            add(side + "_normal_force", "N", wheel, sampled(trace.normal_force / 2))
            add(
                side + "_wheel_applied",
                "N·m",
                wheel,
                sampled(
                    [symmetric_forward_wheel_input(t)[i + 2] for t in trace.applied]
                ),
                True,
            )
            add(
                side + "_tangential_force",
                "N",
                wheel,
                sampled(trace.tangent_force[:, i]),
            )
            add(
                side + "_contact_point",
                "m",
                wheel,
                [
                    [center[0], center[1], 0.0]
                    for center in signals[side + "_wheel_center"].values
                ],
            )
        outcome = "completed" if end >= duration else trace.status[-1]
        return RunRecord(
            "NB5",
            config,
            inputs.provenance(),
            time,
            signals,
            outcome,
            transitions(trace.time.tolist(), list(trace.status), "Điều khiển")
            + [event(outcome, None, time[-1])],
            plain(
                {
                    "parameters": {
                        "planar": dict(inputs.planar.parameters),
                        "hip_envelope": inputs.fixture["hip_envelope"],
                    },
                    "coordinate_order": PLANAR_ORDER,
                    "Q": q_weight,
                    "R": r_weight,
                    "assumptions": "Prescribed symmetric hip trajectory and rigid rolling. Wheel command is total +x torque.",
                }
            ),
        )


def planar_equations(inputs: Inputs, run: RunRecord, index: int) -> dict:
    state = run.signals["state_7dof"].values[index]
    rate = run.signals["rate_7dof"].values[index]
    terms = inputs.planar.evaluate(state, rate)
    return plain(
        {
            "Phương trình": "M q̈ + b = S τ + Jᵀ F",
            "Thứ tự q": PLANAR_ORDER,
            "M": terms.mass,
            "b": terms.bias,
            "S": terms.input,
            "J [Nz L/R; Fx L/R]": terms.contact,
            "J̇ q̇": terms.contact_bias,
            **(
                {
                    "Aᵈ": run.signals["linear_a"].values[index],
                    "Bᵈ": run.signals["linear_b"].values[index],
                    "Gain": run.signals["gain"].values[index],
                    "Q": run.metadata["Q"],
                    "R": run.metadata["R"],
                    "Ràng buộc lực danh định ≤ 0": run.signals[
                        "nominal_force_constraints"
                    ].values[index],
                }
                if run.study == "NB5"
                else {}
            ),
        }
    )
