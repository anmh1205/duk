"""NB1–NB2 linkage, prescribed motion and static hip loading."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from duk_core.modeling.ground_contact import (
    grounded_thin_disk_motion,
    place_biped_on_ground,
)
from duk_core.modeling.kinematic_bench import sample_bench_leg
from duk_core.modeling.leg_differential import (
    inverse_wheel_position,
    leg_second_derivative,
)
from duk_core.modeling.leg_linkage import LinkageError, leg_jacobian
from duk_core.modeling.trajectories import HipTrajectory

from .inputs import Inputs, merged, number, vector
from .records import RunRecord, Signal, plain

POINTS = "ABCDHO"
SIDES = ("left", "right")


def kinematic_defaults(inputs: Inputs) -> dict:
    q = inputs.fixture["whole_body"]["initial_q_deg"]
    pose = sample_bench_leg(inputs.geometry, math.radians(q)).pose
    force = float(inputs.planar.masses.sum() * inputs.planar.parameters["g_m_s2"] / 2)
    return {
        "study": "NB1–2",
        "mode": "pose",
        "left_q_deg": q,
        "right_q_deg": q,
        "left_final_deg": q,
        "right_final_deg": q,
        "joint_rate_rad_s": [0.0, 0.0],
        "joint_accel_rad_s2": [0.0, 0.0],
        "inverse_side": "left",
        "inverse_target_mm": list(pose.o),
        "force_left_N": [0.0, force],
        "force_right_N": [0.0, force],
        "support_levels_m": [0.0, 0.0],
        "duration_s": inputs.fixture["height_experiment"]["slow_duration_s"],
        "period_s": inputs.fixture["height_experiment"]["period_s"],
    }


def kinematic_record(inputs: Inputs, options: dict) -> RunRecord:
    config = merged(kinematic_defaults(inputs), options)
    mode = config["mode"]
    if mode not in ("pose", "trajectory", "inverse"):
        raise ValueError(
            "Choose forward pose, prescribed trajectory or inverse wheel target"
        )
    geometry = inputs.geometry
    limits = tuple(map(math.degrees, geometry.geometric_linkage_angle_limits_rad()))
    angles = [
        math.radians(number(config[s + "_q_deg"], s + "_q_deg", *limits)) for s in SIDES
    ]
    solutions = []
    if mode == "inverse":
        side = config["inverse_side"]
        if side not in SIDES:
            raise ValueError("inverse_side must be left or right")
        target = vector(config["inverse_target_mm"], "inverse_target_mm", 2)
        poses = inverse_wheel_position(geometry, tuple(target))
        solutions = [math.degrees(pose.linkage_angle_rad) for pose in poses]
        idx = SIDES.index(side)
        angles[idx] = min(
            poses, key=lambda pose: abs(pose.linkage_angle_rad - angles[idx])
        ).linkage_angle_rad
    duration = number(config["duration_s"], "duration_s", 0.001, 60)
    period = number(config["period_s"], "period_s", 0.001, 1)
    if math.ceil(duration / period) > 5_000:
        raise ValueError("At most 5,000 kinematic intervals")
    rates = vector(config["joint_rate_rad_s"], "joint_rate_rad_s", 2)
    accels = vector(config["joint_accel_rad_s2"], "joint_accel_rad_s2", 2)
    forces = [
        vector(config["force_" + side + "_N"], f"{side} x/z force", 2) for side in SIDES
    ]
    support = tuple(
        1000 * x for x in vector(config["support_levels_m"], "support_levels_m", 2)
    )
    lateral = inputs.fixture["spatial"]["lateral_mounts_m"]
    spacing = 1000 * (lateral[0] - lateral[1])
    mounts = tuple(
        tuple(
            1000 * inputs.planar.parameters[f"a_{side}_{axis}_m"] for axis in ("x", "z")
        )
        for side in ("L", "R")
    )
    paths = [
        HipTrajectory(
            q,
            math.radians(number(config[s + "_final_deg"], s + "_final_deg", *limits)),
            duration,
        )
        for s, q in zip(SIDES, angles)
    ]
    times = (
        np.linspace(0, duration, math.ceil(duration / period) + 1).tolist()
        if mode == "trajectory"
        else [0.0]
    )
    signals = {}

    def add(name, unit, scope, value, frame=""):
        if name not in signals:
            signals[name] = Signal(unit, scope, [], frame)
        signals[name].values.append(plain(value))

    for time in times:
        motion = (
            [path.evaluate(time) for path in paths]
            if mode == "trajectory"
            else zip(angles, rates, accels)
        )
        left, right = legs = [sample_bench_leg(geometry, *values) for values in motion]
        placement = place_biped_on_ground(
            geometry,
            left.pose,
            right.pose,
            leg_spacing_mm=spacing,
            wheel_width_mm=0.0,
            support_levels_mm=support,
            sagittal_mounts_mm=mounts,
            lateral_center_mm=500 * sum(lateral),
        )
        velocity, acceleration = grounded_thin_disk_motion(
            placement,
            left.pose,
            right.pose,
            tuple(leg.wheel_velocity_mm_s for leg in legs),
            tuple(leg.wheel_acceleration_mm_s2 for leg in legs),
        )
        ql, qr = [leg.pose.linkage_angle_rad for leg in legs]
        # Sagittal static gravity and explicit axle load, while RViz displays the geometric support pose.
        needed = inputs.planar.static_hip_effort([0, 0, 0, ql, qr, 0, 0], forces)
        add(
            "position",
            "m",
            "body",
            [0.0, 0.0, placement.body_height_mm / 1000],
            "world",
        )
        add(
            "orientation_xyzw",
            "1",
            "body",
            Rotation.from_rotvec([placement.roll_rad, 0, 0]).as_quat(),
            "body → world",
        )
        add(
            "state_joints",
            "rad",
            "robot",
            [
                ql,
                qr,
                *(-math.atan2(leg.pose.e_bc[1], leg.pose.e_bc[0]) for leg in legs),
            ],
        )
        add("height", "m", "body", placement.body_height_mm / 1000)
        add("roll", "rad", "body", placement.roll_rad)
        for index, (side, leg) in enumerate(zip(SIDES, legs)):

            def leg_value(name, unit, value, part="linkage", frame=""):
                add(side + "_" + name, unit, side + "." + part, value, frame)

            leg_value("hip_angle", "rad", leg.pose.linkage_angle_rad, "hip")
            leg_value("hip_rate", "rad/s", leg.q_dot_rad_s, "hip")
            leg_value("hip_acceleration", "rad/s²", leg.q_ddot_rad_s2, "hip")
            leg_value("boundary_margin", "rad", leg.boundary_margin_rad, "hip")
            leg_value("closure_error", "mm", leg.closure_error_mm)
            leg_value("carrier_error", "mm", leg.carrier_error_mm)
            leg_value("double_area_error", "mm²", leg.carrier_area_error_mm2)
            leg_value("loop_sine", "1", leg.loop_sine)
            leg_value("hip_static_effort", "N·m", needed[index], "hip")
            bound = inputs.motor.joint_limit(leg.q_dot_rad_s)
            leg_value("hip_limit", "N·m", bound, "hip")
            leg_value(
                "hip_utilization",
                "1",
                abs(needed[index]) / bound if bound else None,
                "hip",
            )
            leg_value("hip_saturation", "1", abs(needed[index]) > bound, "hip")
            leg_value("axle_external_load", "N", forces[index], "wheel", "body x/z")
            leg_value(
                "wheel_velocity_body",
                "mm/s",
                leg.wheel_velocity_mm_s,
                "wheel",
                "body x/z",
            )
            leg_value(
                "wheel_accel_body",
                "mm/s²",
                leg.wheel_acceleration_mm_s2,
                "wheel",
                "body x/z",
            )
            leg_value("wheel_velocity_world", "mm/s", velocity[index], "wheel", "world")
            leg_value(
                "wheel_accel_world", "mm/s²", acceleration[index], "wheel", "world"
            )
            leg_value(
                "wheel_center",
                "m",
                np.array(placement.leg_point(leg.pose, side, "O")) / 1000,
                "wheel",
                "world",
            )
            leg_value("gap", "mm", placement.ground_gap_mm(leg.pose, side), "wheel")
            for point in POINTS:
                leg_value(
                    point, "mm", leg.pose.point(point), "linkage." + point, "leg x/z"
                )
                leg_value(
                    point + "_world",
                    "mm",
                    placement.leg_point(leg.pose, side, point),
                    "linkage." + point,
                    "world",
                )
    return RunRecord(
        "NB1–2",
        config,
        inputs.provenance(),
        times,
        signals,
        "completed",
        [],
        {
            "parameters": {
                "planar": dict(inputs.planar.parameters),
                "hip_envelope": inputs.fixture["hip_envelope"],
            },
            "inverse_solutions_deg": solutions,
            "geometric_bounds_deg": list(limits),
            "assumptions": "Thin-disk support geometry. Sagittal static load at O with separate speed-dependent actuator limit.",
        },
    )


def linkage_equations(inputs: Inputs, q: float, point: str | None = None) -> dict:
    sample = sample_bench_leg(inputs.geometry, q)
    jac = leg_jacobian(inputs.geometry, sample.pose)
    second = leg_second_derivative(inputs.geometry, sample.pose)
    return {
        "Quy ước": "q = ∠(AB → DA), CCW; x phải, z lên; A là gốc",
        "Phương trình": "θ_AB = θ_DA − q; ṗ = J q̇; p̈ = J q̈ + (d²p/dq²) q̇²",
        "q [rad]": q,
        "Khe hình học [rad]": sample.boundary_margin_rad,
        "Sai số khép vòng [mm]": sample.closure_error_mm,
        "Sai số BCO [mm]": sample.carrier_error_mm,
        "Sai số gấp đôi diện tích BCO [mm²]": sample.carrier_area_error_mm2,
        "|sin ∠(BC, DC)|": sample.loop_sine,
        **{
            name: {
                "p [mm]": sample.pose.point(name),
                "dp/dq [mm/rad]": jac.point(name),
                "d²p/dq² [mm/rad²]": second.point(name),
            }
            for name in point or POINTS
        },
    }


def linkage_signals(inputs: Inputs, signals: dict[str, Signal]) -> dict[str, Signal]:
    """Annotate replay geometry at the same poses as the body and wheel samples."""
    geometry = inputs.geometry
    result = {}
    for index, side in enumerate(SIDES):
        prefix = side + "_"
        suffix = ("L", "R")[index]
        mount = np.array(
            [
                inputs.planar.parameters[f"a_{suffix}_x_m"],
                inputs.fixture["spatial"]["lateral_mounts_m"][index],
                inputs.planar.parameters[f"a_{suffix}_z_m"],
            ]
        )
        scope = side + ".linkage"
        quantities = {
            "closure_error": ("mm", "closure_error_mm"),
            "carrier_error": ("mm", "carrier_error_mm"),
            "double_area_error": ("mm²", "carrier_area_error_mm2"),
            "loop_sine": ("1", "loop_sine"),
        }
        for name, (unit, _) in quantities.items():
            result[prefix + name] = Signal(unit, scope, [])
        for point in POINTS:
            result[prefix + point] = Signal("mm", scope + "." + point, [], "leg x/z")
            result[prefix + point + "_world"] = Signal(
                "mm", scope + "." + point, [], "world"
            )
        for joints, origin, orientation in zip(
            signals["state_joints"].values,
            signals["position"].values,
            signals["orientation_xyzw"].values,
        ):
            try:
                sample = sample_bench_leg(geometry, joints[index])
            except LinkageError:
                sample = None
            transform = Rotation.from_quat(orientation)
            for name, (_, attribute) in quantities.items():
                result[prefix + name].values.append(
                    getattr(sample, attribute) if sample else None
                )
            for point in POINTS:
                local = sample.pose.point(point) if sample else None
                world = None
                if local is not None:
                    world = 1000 * (
                        transform.apply(
                            mount + np.array([local[0], 0, local[1]]) / 1000
                        )
                        + origin
                    )
                result[prefix + point].values.append(plain(local))
                result[prefix + point + "_world"].values.append(plain(world))
    return result
