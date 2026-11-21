"""Bind the local dashboard to the canonical geometry and NB4 research inputs."""

from __future__ import annotations

import json
import math
from dataclasses import fields
from hashlib import sha256
from pathlib import Path
from threading import Lock
from typing import Any

from workbench.records import plain

from duk_core.control.sagittal_balance import design_fixed_hip_lqr
from duk_core.modeling.kinematic_bench import KinematicBench
from duk_core.modeling.leg_linkage import leg_jacobian, load_leg_geometry_json
from duk_core.modeling.wheeled_pendulum import WheeledPendulum
from duk_core.simulation.balance import simulate_balance

CONTROLLER_STATES = {
    "position": ("x [m]", "v [m/s]", "θ [rad]", "ω [rad/s]"),
    "balance": ("v [m/s]", "θ [rad]", "ω [rad/s]"),
}
FEEDBACK_SOURCES = ("odometry", "truth", "gated")


def _number(
    value: object, label: str, lower: float = 0, upper: float = math.inf
) -> float:
    if type(value) not in (int, float) or not lower <= value <= upper:
        raise ValueError(f"Giá trị {label} nằm ngoài miền hợp lệ.")
    if not math.isfinite(value):
        raise ValueError(f"Giá trị {label} phải hữu hạn.")
    return float(value)


def _integer(value: object, label: str, upper: int) -> int:
    if type(value) is not int or not 0 <= value <= upper:
        raise ValueError(f"Giá trị {label} phải là số nguyên từ 0 đến {upper}.")
    return value


class DashboardSimulation:
    """Translate JSON to the core's explicit SI inputs and return labeled traces."""

    def __init__(
        self,
        repository: Path,
        profile_path: Path | None = None,
        *,
        profile_bytes: bytes | None = None,
    ) -> None:
        geometry_path = repository / "robot/config/physical/duk_leg_geometry.json"
        fixture_path = (
            profile_path or repository / "research/experiments/duk-v3/synthetic.json"
        )
        scenarios_path = (
            repository / "robot/simulation/scenarios/fixed-hip-balance.json"
        )
        self.geometry = load_leg_geometry_json(geometry_path)
        bench = KinematicBench(self.geometry)
        if profile_bytes is None:
            profile_bytes = fixture_path.read_bytes()
        fixture = json.loads(profile_bytes)
        if (
            fixture["schema_version"] != 1
            or fixture["status"] != "synthetic_research_only"
        ):
            raise ValueError("Expected the versioned synthetic DUK v3 fixture")
        plant = fixture["v2_balance"]
        study = fixture["balance_experiment"]
        self.plant_inputs = {
            field.name: plant[field.name] for field in fields(WheeledPendulum)
        }
        self.model = WheeledPendulum(**self.plant_inputs)
        self.design_inputs = {
            "period_s": plant["sample_period_s"],
            "state_scales": plant["state_scales"],
            "torque_scale_N_m": plant["torque_scale_N_m"],
        }
        position, balance = design_fixed_hip_lqr(
            self.model,
            self.design_inputs["period_s"],
            self.design_inputs["state_scales"],
            self.design_inputs["torque_scale_N_m"],
        )
        self.designs = {"position": position, "balance": balance}
        self.settings = {
            "period": plant["sample_period_s"],
            "duration": study["duration_s"],
            "motor_bound": plant["total_torque_limit_N_m"],
            "slip_scale": study["slip_scale_m_s"],
            "fall_angle": math.radians(study["fall_angle_deg"]),
            "slip_threshold": study["slip_threshold_m_s"],
        }
        self.presets = json.loads(scenarios_path.read_bytes())
        self.defaults = {
            **self.presets["dry"],
            "controller": "position",
            "feedback": "odometry",
            "initial_lean_deg": study["initial_lean_deg"],
            "delay_samples": 0,
            "noise_seed": study["sensor_seed"],
            "accel_noise": study["acceleration_noise_m_s2"],
            "pitch_noise": 0,
        }
        self.max_delay = math.ceil(self.settings["duration"] / self.settings["period"])
        self.max_lean = study["fall_angle_deg"]
        self.inputs = {
            "profile": fixture["status"],
            "fixture_sha256": sha256(profile_bytes).hexdigest(),
            "geometry_sha256": sha256(geometry_path.read_bytes()).hexdigest(),
        }
        self.config = {
            "balance": self.defaults,
            "presets": self.presets,
            "duration_s": self.settings["duration"],
            "period_s": self.settings["period"],
            "max_delay_samples": self.max_delay,
            "fall_angle_deg": self.max_lean,
            "kinematics": {
                "bounds_deg": [math.degrees(value) for value in bench.bounds],
                "initial_deg": math.degrees(bench.samples[0].pose.linkage_angle_rad),
            },
        }
        # LSODA maintains shared integration state in some supported SciPy versions.
        self.balance_lock = Lock()

    def kinematics(self, request: dict[str, Any]) -> dict[str, Any]:
        if set(request) != {"left_deg", "right_deg"}:
            raise ValueError("Cần cung cấp cả góc chân trái và chân phải.")
        low, high = self.config["kinematics"]["bounds_deg"]
        angles = [
            _number(request[key], label, low, high)
            for key, label in (("left_deg", "góc trái"), ("right_deg", "góc phải"))
        ]
        bench = KinematicBench(self.geometry)
        # Preserve the exact radian bounds at endpoints after the round trip.
        bench.set_angles(
            *(
                bench.bounds[0]
                if angle == low
                else bench.bounds[1]
                if angle == high
                else math.radians(angle)
                for angle in angles
            )
        )
        legs = []
        for angle, sample in zip(angles, bench.samples):
            legs.append(
                {
                    "q_deg": angle,
                    "points_mm": {name: sample.pose.point(name) for name in "ABCDHO"},
                    "wheel_jacobian_mm_rad": leg_jacobian(
                        self.geometry, sample.pose
                    ).d_o,
                    "closure_error_mm": sample.closure_error_mm,
                    "carrier_error_mm": sample.carrier_error_mm,
                    "boundary_margin_deg": math.degrees(sample.boundary_margin_rad),
                }
            )
        return {"legs": legs, "wheel_radius_mm": self.geometry.wheel_radius_mm}

    def balance(self, request: dict[str, Any]) -> dict[str, Any]:
        if extra := request.keys() - self.defaults.keys():
            raise ValueError("Tham số không được hỗ trợ: " + ", ".join(sorted(extra)))
        options = self.defaults | request
        controller = options["controller"]
        if controller not in tuple(self.designs):
            raise ValueError("Bộ điều khiển không tương thích với mô hình NB4.")
        feedback = options["feedback"]
        if feedback not in FEEDBACK_SOURCES:
            raise ValueError("Nguồn phản hồi không tương thích với mô hình NB4.")
        for key, label in (
            ("friction", "độ bám"),
            ("accel_noise", "nhiễu gia tốc"),
            ("pitch_noise", "nhiễu góc"),
        ):
            options[key] = _number(options[key], label)
        if options["traction_budget"] is not None:
            options["traction_budget"] = _number(
                options["traction_budget"], "giới hạn theo độ bám"
            )
        options["initial_lean_deg"] = _number(
            options["initial_lean_deg"],
            "góc nghiêng ban đầu",
            -self.max_lean,
            self.max_lean,
        )
        options["delay_samples"] = _integer(
            options["delay_samples"], "độ trễ", self.max_delay
        )
        options["noise_seed"] = _integer(options["noise_seed"], "seed", 2**32 - 1)
        design = self.designs[controller]
        anchor = 0.0 if controller == "position" else None
        with self.balance_lock:
            trace = simulate_balance(
                self.model,
                design.gain,
                math.radians(options["initial_lean_deg"]),
                **self.settings,
                anchor=anchor,
                **{
                    name: options[name]
                    for name in (
                        "friction",
                        "feedback",
                        "traction_budget",
                        "delay_samples",
                        "noise_seed",
                        "accel_noise",
                        "pitch_noise",
                    )
                },
            )
        a, b = self.model.linearization()
        return {
            "schema_version": 1,
            "inputs": self.inputs,
            "configuration": options,
            "plant": self.plant_inputs,
            "simulation": self.settings | {"anchor": anchor},
            "design_inputs": self.design_inputs,
            "linear_model": {
                "continuous_a": a.tolist(),
                "continuous_b": b.tolist(),
                "state_order": CONTROLLER_STATES["position"],
                "feedback_state_order": CONTROLLER_STATES[controller],
                "gain": design.gain[0].tolist(),
                "spectral_radius": design.spectral_radius,
            },
            "trace": {
                field.name: plain(getattr(trace, field.name))
                for field in fields(trace)
                if field.name not in ("outcome", "limit_reason")
            }
            | {"outcome": trace.outcome, "limit_reason": trace.limit_reason},
        }
