"""Research input binding and scenario validation for the local workbench."""

from __future__ import annotations

import json
import math
import subprocess
from functools import cached_property
from hashlib import sha256
from importlib.metadata import version
from pathlib import Path
from typing import Any

from duk_core.modeling import build_planar_biped_model
from duk_core.modeling.hip_load import LinearTorqueSpeed
from duk_core.modeling.leg_linkage import geometry_from_mapping
from duk_core.modeling.planar_numeric import NumericPlanarBiped
from duk_core.simulation import sample_times
from duk_core.simulation.whole_body_schedule import FAULT_MODES, compile_schedule

from .records import decimal, finite_number, plain, reject_json_constant

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = ROOT / "robot/config/physical/duk_leg_geometry.json"
PROFILE = ROOT / "research/experiments/duk-v3/synthetic.json"
LEFT_GRIP = ROOT / "robot/simulation/scenarios/whole-body-left-grip.json"
INTEGER_SCENARIO_FIELDS = frozenset(
    ("sensor_seed", "substeps", "noise_seed", "delay_samples", "mpc_horizon")
)
NB6_LEGACY_TIMED_FIELDS = frozenset(
    (
        "friction_change_s",
        "friction_after",
        "impulse_at_s",
        "velocity_impulse_m_s",
        "angular_velocity_impulse_rad_s",
        "fault",
        "fault_start_s",
        "fault_end_s",
        "drive_speed_m_s",
        "drive_end_s",
        "turn_rate_rad_s",
        "turn_end_s",
    )
)


def number(
    value: Any, name: str, low: float = -math.inf, high: float = math.inf
) -> float:
    if not finite_number(value) or not low <= value <= high:
        bounds = [
            f"{operator} {decimal(bound)}"
            for operator, bound in (("≥", low), ("≤", high))
            if math.isfinite(bound)
        ]
        detail = " (" + " · ".join(bounds) + ")" if bounds else ""
        raise ValueError(f"{name} cần giá trị hữu hạn{detail}.")
    return float(value)


def integer(value: Any, name: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer from {low} to {high}")
    return value


def vector(value: Any, name: str, size: int, low: float = -math.inf) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} requires {size} values in the documented order")
    return [number(item, name, low) for item in value]


def merged(defaults: dict, options: dict) -> dict:
    extra = options.keys() - defaults.keys()
    if extra:
        raise ValueError("Unsupported scenario fields: " + ", ".join(sorted(extra)))
    return defaults | options


def read_object(path: Path) -> dict:
    data = json.loads(path.read_bytes(), parse_constant=reject_json_constant)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a JSON object in {path.name}")
    return data


def fingerprints(root: Path, folder: str) -> dict[str, str]:
    return {
        str(path.relative_to(root)): sha256(path.read_bytes()).hexdigest()
        for path in sorted((root / folder).rglob("*.py"))
    }


class Inputs:
    def __init__(self, profile_path: Path = PROFILE) -> None:
        self.profile_path = profile_path
        self.profile_bytes = profile_path.read_bytes()
        self.fixture = json.loads(
            self.profile_bytes, parse_constant=reject_json_constant
        )
        if (
            not isinstance(self.fixture, dict)
            or type(self.fixture.get("schema_version")) is not int
            or self.fixture.get("schema_version") != 1
            or self.fixture.get("status") != "synthetic_research_only"
        ):
            raise ValueError(
                "Expected a versioned synthetic research parameter profile"
            )
        geometry_bytes = GEOMETRY.read_bytes()
        self.geometry = geometry_from_mapping(json.loads(geometry_bytes))
        self.parameter_info = {
            "profile": self.fixture["status"],
            "profile_name": profile_path.name,
            "profile_sha256": sha256(self.profile_bytes).hexdigest(),
            "geometry_sha256": sha256(geometry_bytes).hexdigest(),
        }

    @cached_property
    def planar(self) -> NumericPlanarBiped:
        return NumericPlanarBiped(
            build_planar_biped_model(self.geometry), self.fixture["planar"]
        )

    @cached_property
    def motor(self) -> LinearTorqueSpeed:
        values = self.fixture["hip_envelope"]
        return LinearTorqueSpeed(
            values["stall_torque_N_m"],
            values["no_load_speed_rad_s"],
            values["efficiency"],
        )

    def provenance(self) -> dict:
        try:
            commit = subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                cwd=ROOT,
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=5,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            commit = None
        return self.parameter_info | {
            "git_commit": commit,
            "code": (
                fingerprints(ROOT, "robot/software/core/src/duk_core")
                | fingerprints(ROOT, "applications/operator-dashboard")
            ),
            "dependencies": {
                package: version(package) for package in ("numpy", "scipy", "sympy")
            },
        }

    def whole_body_defaults(self, case: str = "nominal") -> dict:
        study = self.fixture["whole_body"]
        if case == "left_grip":
            return self.whole_body_defaults("nominal") | read_object(LEFT_GRIP)
        if not isinstance(case, str) or case not in study["cases"]:
            raise ValueError("Unknown NB6 preset")
        scenario = study["scenario"] | study["cases"][case]
        for name in ("friction", "friction_after"):
            if isinstance(scenario[name], (int, float)):
                scenario[name] = [scenario[name], scenario[name]]
        return {
            "study": "NB6",
            "case": case,
            "controller": "weighted",
            "priority": "balance_first",
            "support_levels_m": [0.0, 0.0],
            "period_s": study["period_s"],
            "substeps": study["substeps"],
            "initial_q_deg": study["initial_q_deg"],
            "sensor_seed": study["sensor_seed"],
            "accel_noise_m_s2": study["accel_noise_m_s2"],
            "height_offset_m": 0.0,
            **scenario,
            "schedule": None,
        }

    def whole_body_configuration(self, options: dict) -> dict:
        defaults = self.whole_body_defaults(options.get("case", "nominal"))
        config = merged(defaults, options)
        if config["schedule"] is not None:
            # The explicit schedule owns these timed inputs. Preserve only
            # the active values in the resolved scenario, even for imported drafts.
            config.update({key: defaults[key] for key in NB6_LEGACY_TIMED_FIELDS})
        if config["controller"] not in ("weighted", "hierarchical"):
            raise ValueError(
                "NB6 accepts QP (weighted) or HQP (hierarchical) four-actuator effort"
            )
        if config["priority"] not in ("balance_first", "legs_first"):
            raise ValueError("Unknown NB6 task priority")
        if config["fault"] not in FAULT_MODES:
            raise ValueError("Unknown NB6 fault")
        settings = self.fixture["whole_body"]
        ranges = {
            "initial_q_deg": tuple(settings["hip_limits_deg"]),
            "initial_lean_deg": (-settings["fall_deg"], settings["fall_deg"]),
            "period_s": (0.001, 1.0),
            "duration_s": (0.001, 60.0),
            "hip_capacity": (0, 1),
            "reference_capacity": (0, 1),
            "drive_speed_m_s": (-math.inf, math.inf),
            "turn_rate_rad_s": (-math.inf, math.inf),
        }
        for name, default in defaults.items():
            value = config[name]
            if isinstance(default, bool) and type(value) is not bool:
                raise ValueError(f"{name} must be a boolean")
            if isinstance(default, list):
                if name.startswith("friction") and type(value) in (int, float):
                    value = [value, value]
                config[name] = vector(
                    value,
                    name,
                    len(default),
                    0 if name.startswith("friction") else -math.inf,
                )
            if type(default) in (float, int) and name not in INTEGER_SCENARIO_FIELDS:
                lower = (
                    0
                    if name.endswith(("_s", "_hz")) or name == "accel_noise_m_s2"
                    else -math.inf
                )
                config[name] = number(value, name, *ranges.get(name, (lower, math.inf)))
        config["sensor_seed"] = integer(
            config["sensor_seed"], "sensor_seed", 0, 2**32 - 1
        )
        config["substeps"] = integer(config["substeps"], "substeps", 1, 500)
        steps = math.ceil(config["duration_s"] / config["period_s"])
        if steps * config["substeps"] > 100_000 or steps > 5_000:
            raise ValueError(
                "NB6 workbench limit is 5,000 command samples and 100,000 integration steps"
            )
        if config["fault_end_s"] < config["fault_start_s"]:
            raise ValueError("fault_end_s precedes fault_start_s")
        events = config.pop("schedule")
        if events is not None:
            ticks = sample_times(config["duration_s"], config["period_s"])[:-1]
            config["schedule"] = [
                {
                    "time_s": change.time_s,
                    "field": change.field,
                    "value": plain(change.value),
                }
                for change in compile_schedule(events, ticks)
            ]
        return config
