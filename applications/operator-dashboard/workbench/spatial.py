"""Bind NB6, its spatial inspection and its playback to the workbench."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from duk_core.modeling.leg_linkage import LinkageError
from duk_core.modeling.spatial_biped import SpatialBiped, SpatialState
from duk_core.simulation.whole_body import run_whole_body

from .inputs import Inputs
from .kinematics import linkage_signals
from .records import RunRecord, Signal, event, plain, transitions

VELOCITY_ORDER = [
    "vx [m/s]",
    "vy [m/s]",
    "vz [m/s]",
    "ωx [rad/s]",
    "ωy [rad/s]",
    "ωz [rad/s]",
    "q̇L [rad/s]",
    "q̇R [rad/s]",
    "spiṅL [rad/s]",
    "spiṅR [rad/s]",
]
SIDES = ("left", "right")


def spatial_model(inputs: Inputs, config: dict) -> SpatialBiped:
    parameters = inputs.fixture["spatial"]
    return SpatialBiped(
        inputs.planar,
        parameters["lateral_mounts_m"],
        parameters["transverse_inertias_kg_m2"],
        support_levels_m=config["support_levels_m"],
    )


def spatial_record(inputs: Inputs, options: dict) -> RunRecord:
    config = inputs.whole_body_configuration(options)
    model = spatial_model(inputs, config)
    settings = inputs.fixture["whole_body"] | {
        key: config[key]
        for key in (
            "period_s",
            "substeps",
            "initial_q_deg",
            "sensor_seed",
            "accel_noise_m_s2",
        )
    }
    trace = run_whole_body(
        model,
        inputs.motor,
        inputs.fixture["v2_balance"],
        settings,
        config,
        method=config["controller"],
        priority=config["priority"],
    )
    times = trace.time.tolist()
    terminal = trace.terminal_state
    append_end = trace.terminal_time > times[-1]
    try:
        kin_end = model.kinematics(terminal)
    except LinkageError:
        kin_end = None
    signals = {}

    def add(name, unit, scope, data, end=None, *, frame="", held=False):
        values = plain(data)
        if append_end:
            values.append(plain(end))
        signals[name] = Signal(unit, scope, values, frame, held)

    add("position", "m", "body", trace.position, terminal.position, frame="world")
    add(
        "orientation_xyzw",
        "1",
        "body",
        Rotation.from_matrix(trace.rotation).as_quat(),
        Rotation.from_matrix(terminal.rotation).as_quat(),
        frame="body → world",
    )
    add(
        "state_joints",
        "rad",
        "robot",
        trace.joints,
        terminal.joints,
        frame="hip L/R, carrier-relative spin L/R",
    )
    add(
        "state_velocity",
        "m/s, rad/s",
        "robot",
        trace.velocity,
        terminal.velocity,
        frame="world linear/angular, joints L/R",
    )
    com_end = (
        np.average(kin_end.centers, axis=0, weights=model.masses) if kin_end else None
    )
    add("center_of_mass", "m", "body", trace.center_of_mass, com_end, frame="world")
    angles = Rotation.from_matrix(trace.rotation).as_euler("xyz")
    angles_end = Rotation.from_matrix(terminal.rotation).as_euler("xyz")
    for index, name in enumerate(("roll", "pitch", "yaw")):
        add(
            name, "rad", "body", angles[:, index], angles_end[index], frame="world, XYZ"
        )
    add(
        "height", "m", "body", trace.position[:, 2], terminal.position[2], frame="world"
    )
    add(
        "lean",
        "rad",
        "body",
        trace.pose[:, 0],
        trace.terminal_pose[0],
        frame="upper CoM about axle",
    )
    add("model_speed", "m/s", "body", trace.true_and_estimated_speed[:, 0])
    add(
        "estimated_speed",
        "m/s",
        "body",
        np.where(trace.sensor_valid, trace.true_and_estimated_speed[:, 1], np.nan),
    )
    add("odometry_speed", "m/s", "body", trace.sensor[:, 0])
    add("accelerometer", "m/s²", "body", trace.sensor[:, 1])
    add("sensor_valid", "1", "robot", trace.sensor_valid)
    for index, (reference, unit) in enumerate(
        (("height", "m"), ("yaw_rate", "rad/s"), ("speed", "m/s"))
    ):
        add(
            f"requested_{reference}",
            unit,
            "body",
            trace.requested_reference[:, index],
            held=True,
        )
        add(
            f"effective_{reference}",
            unit,
            "body",
            trace.effective_reference[:, index],
            held=True,
        )
    for index, side in enumerate(SIDES):
        hip, wheel = side + ".hip", side + ".wheel"
        add(
            side + "_hip_angle",
            "rad",
            hip,
            trace.joints[:, index],
            terminal.joints[index],
        )
        add(
            side + "_hip_rate",
            "rad/s",
            hip,
            trace.velocity[:, index + 6],
            terminal.velocity[index + 6],
        )
        add(
            side + "_hip_travel_margin",
            "rad",
            hip,
            np.min(
                np.abs(
                    trace.joints[:, index, None]
                    - np.deg2rad(settings["hip_limits_deg"])
                ),
                axis=1,
            ),
        )
        add(
            side + "_wheel_center",
            "m",
            wheel,
            trace.wheel_centers[:, index],
            kin_end.centers[model.wheels[index]] if kin_end else None,
            frame="world",
        )
        add(
            side + "_contact_point",
            "m",
            wheel,
            trace.contact_points[:, index],
            kin_end.contact_points[index] if kin_end else None,
            frame="world",
        )
        add(
            side + "_spin",
            "rad",
            wheel,
            trace.joints[:, index + 2],
            terminal.joints[index + 2],
            frame="carrier",
        )
        add(
            side + "_spin_rate",
            "rad/s",
            wheel,
            trace.velocity[:, index + 8],
            terminal.velocity[index + 8],
            frame="carrier",
        )
        add(
            side + "_gap",
            "m",
            wheel,
            trace.contact_gap[:, index],
            kin_end.gaps[index] if kin_end else None,
        )
        end_speed = kin_end.contact[index] @ terminal.velocity if kin_end else None
        add(
            side + "_slip_speed",
            "m/s",
            wheel,
            np.linalg.norm(trace.contact_speed[:, index, :2], axis=1),
            np.linalg.norm(end_speed[:2]) if end_speed is not None else None,
        )
        add(
            side + "_wheel_velocity",
            "m/s",
            wheel,
            trace.wheel_velocity[:, index],
            kin_end.linear[model.wheels[index]] @ terminal.velocity
            if kin_end
            else None,
            frame="world",
        )
        add(
            side + "_normal_force",
            "N",
            wheel,
            trace.force[:, index, 2],
            trace.terminal_force[index, 2],
        )
        add(
            side + "_tangential_force",
            "N",
            wheel,
            trace.force[:, index, :2],
            trace.terminal_force[index, :2],
            frame="world x/y",
        )
        add(
            side + "_allocated_force",
            "N",
            wheel,
            trace.allocated_force[:, index],
            frame="world",
            held=True,
        )
        add(side + "_friction", "1", wheel, trace.actual_friction[:, index], held=True)
        add(
            side + "_friction_belief",
            "1",
            wheel,
            trace.friction_belief[:, index],
            held=True,
        )
        add(
            side + "_contact_mode",
            "1",
            wheel,
            [m[index] for m in trace.contact_modes],
            trace.terminal_contact_modes[index]
            if trace.terminal_contact_modes
            else None,
        )
        for actuator, offset, scope in (("hip", 0, hip), ("wheel", 2, wheel)):
            for label, data in (
                ("requested", trace.requested_effort),
                ("applied", trace.applied_effort),
                ("limit", trace.effort_limit),
            ):
                add(
                    f"{side}_{actuator}_{label}",
                    "N·m",
                    scope,
                    data[:, index + offset],
                    held=True,
                )
            channel = index + offset
            add(
                f"{side}_{actuator}_preceding_effort",
                "N·m",
                scope,
                [None, *trace.applied_effort[:-1, channel]],
                trace.terminal_integrated_effort[channel]
                if trace.terminal_integrated_effort is not None
                else None,
            )

    for name, data, unit in (
        ("solver_status", trace.solver_status, "1"),
        ("supervisor", trace.supervisor, "1"),
        ("command_source", trace.command_source, "1"),
        ("solver_residual", trace.solver_residual, "mixed"),
        ("solver_seconds", trace.solver_seconds, "s"),
        ("capacity_reason", trace.capacity_reason, "1"),
        ("feature_status", trace.feature_status, "1"),
        ("feature_amount", trace.feature_amount, "1"),
        ("task_names", trace.task_names, "1"),
        ("task_scales", trace.task_scales, "mixed"),
        ("task_targets", trace.task_targets, "mixed"),
        ("active_constraints", trace.active_constraints, "1"),
    ):
        add(name, unit, "control", data)

    def actual(requested):
        return next((time for time in times if time >= requested), None)

    events = []
    if config["friction"] != config["friction_after"]:
        when = config["friction_change_s"]
        events.append(event("Đổi độ bám", when, actual(when)))
    if any(config["velocity_impulse_m_s"] + config["angular_velocity_impulse_rad_s"]):
        when = config["impulse_at_s"]
        events.append(event("Nhiễu động vận tốc", when, actual(when)))
    if config["fault"] != "none":
        when = config["fault_start_s"]
        active = next((t for t in times if when <= t < config["fault_end_s"]), None)
        events.append(event(f"Lỗi · {config['fault']}", when, active))
    events += transitions(times, list(trace.command_source), "Lệnh")
    if append_end:
        times.append(float(trace.terminal_time))
    for side, name in zip(SIDES, ("Bánh trái", "Bánh phải")):
        events += transitions(times, signals[side + "_contact_mode"].values, name)
    signals.update(linkage_signals(inputs, signals))
    outcome = trace.stop_reason or "completed"
    events.append(event(outcome, None, times[-1]))
    return RunRecord(
        "NB6",
        config,
        inputs.provenance(),
        times,
        signals,
        outcome,
        events,
        plain(
            {
                "parameters": {
                    "planar": dict(model.planar.parameters),
                    "spatial": inputs.fixture["spatial"],
                    "hip_envelope": inputs.fixture["hip_envelope"],
                    "settings": {
                        k: v
                        for k, v in settings.items()
                        if k not in ("cases", "scenario")
                    },
                },
                "velocity_order": VELOCITY_ORDER,
                "fallback_state_order": ["v [m/s]", "lean [rad]", "leaṅ [rad/s]"],
                "fallback_gain": trace.fallback_gain,
                "fallback_a": trace.fallback_a,
                "fallback_b": trace.fallback_b,
                "contact_assumptions": "Thin disks on independent horizontal supports. Force is from the preceding plant substep (static normal preload at t=0).",
            }
        ),
    )


def spatial_equations(inputs: Inputs, run: RunRecord, index: int) -> dict:
    data = {name: signal.values[index] for name, signal in run.signals.items()}
    state = SpatialState(
        np.array(data["position"]),
        Rotation.from_quat(data["orientation_xyzw"]).as_matrix(),
        np.array(data["state_joints"]),
        np.array(data["state_velocity"]),
    )
    try:
        terms = spatial_model(inputs, run.configuration).evaluate(state)
    except LinkageError:
        return {"Trạng thái": "Cơ cấu vượt miền giải tại mẫu dừng."}
    return plain(
        {
            "Phương trình": "M a + b = S τ + Jᵀ F",
            "Thứ tự vận tốc": VELOCITY_ORDER,
            "M": terms.mass,
            "b": terms.bias,
            "S": terms.actuation,
            "J [bánh; x, y, z]": terms.kinematics.contact,
            "J̇ v": terms.contact_bias,
            "Thứ tự nhiệm vụ": data["task_names"],
            "Mục tiêu gia tốc": data["task_targets"],
            "Thang nhiệm vụ": data["task_scales"],
            "Ràng buộc tại biên": data["active_constraints"],
            "A dự phòng": run.metadata["fallback_a"],
            "B dự phòng": run.metadata["fallback_b"],
            "Gain dự phòng": run.metadata["fallback_gain"],
        }
    )
