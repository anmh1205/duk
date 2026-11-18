"""Reference transitions and acceleration-task construction for NB6."""

from dataclasses import dataclass

import numpy as np

from ..control.sagittal_balance import balance_command
from ..control.task_policy import FeatureTransition, return_to_anchor, wrap_angle
from ..control.whole_body import AccelerationTask


@dataclass
class References:
    """Height (m), yaw rate (rad/s), speed (m/s) and optional-task participation.

    Feature order is bounce, level, anchor and heading throughout the study.
    Height derivatives include the participation ramp between its breakpoints;
    the existing piecewise-linear policy is not an acceleration-smooth envelope.
    """

    requested: tuple[float, float, float]
    effective: tuple[float, float, float]
    height_rate: float
    height_acceleration: float
    amounts: list[float]
    intents: tuple[bool, bool, bool, bool]
    capacity_reason: str


class ReferencePolicy:
    """Own feature transitions and explicitly recorded position/heading anchors."""

    def __init__(self, settings, scenario, home):
        self.settings, self.scenario, self.home = settings, scenario, home
        self.features = [FeatureTransition() for _ in range(4)]
        self.anchor = self.heading_anchor = None

    @property
    def statuses(self) -> tuple[str, ...]:
        return tuple(feature.status for feature in self.features)

    def update(
        self,
        state,
        terms,
        observation,
        time,
        *,
        supervisor_mode,
        sensor_valid,
        fallen,
        previous_usage,
    ) -> References:
        p, c, home, t = self.settings, self.scenario, self.home, time
        period, yaw, modes = p["period_s"], terms.kinematics.tasks[3], observation.modes
        secondary_available = (
            all(mode == "rolling" for mode in modes)
            and sensor_valid
            and not fallen
            and supervisor_mode == "normal"
        )
        bounce_on = c["feature_start_s"] <= t < c["bounce_off_s"]
        level_on = c["level_hold"] and t >= c["feature_start_s"]
        anchor_on = c["anchor"] and t >= c["feature_start_s"]
        heading_on = c["heading_hold"] and t >= c["feature_start_s"]
        if heading_on and self.heading_anchor is None:
            self.heading_anchor = yaw
        elif not heading_on:
            self.heading_anchor = None
        if anchor_on and self.anchor is None:
            self.anchor = state.position[:2].copy()
        elif not anchor_on:
            self.anchor = None
        headroom = 1 / (
            1
            + p["governor_gain"] * max(0.0, previous_usage - p["governor_utilization"])
        )
        capacity = min(c["reference_capacity"], headroom)
        if not sensor_valid or fallen or supervisor_mode != "normal":
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
            for transition, enabled in zip(self.features, intents)
        ]
        phase = 2 * np.pi * c["bounce_hz"] * max(0.0, t - c["feature_start_s"])
        omega = 2 * np.pi * c["bounce_hz"]
        wave = c.get("height_offset_m", 0.0) + c["bounce_amplitude_m"] * np.sin(phase)
        wave_rate = c["bounce_amplitude_m"] * omega * np.cos(phase)
        amount_rate = self.features[0].rate
        h_ref = home + amounts[0] * wave
        h_rate = amounts[0] * wave_rate + amount_rate * wave
        h_acc = -amounts[0] * omega**2 * wave + 2 * amount_rate * wave_rate
        requested_yaw_rate = c["turn_rate_rad_s"] if t < c["turn_end_s"] else 0.0
        requested_speed = c["drive_speed_m_s"] if t < c["drive_end_s"] else 0.0
        speed_ref, yaw_rate_ref = (
            (requested_speed, requested_yaw_rate) if secondary_available else (0.0, 0.0)
        )
        if secondary_available and abs(speed_ref) + abs(yaw_rate_ref) < 1e-12:
            if amounts[2] > 0 and self.anchor is not None:
                speed_ref, yaw_rate_ref = return_to_anchor(
                    state.position[:2],
                    self.anchor,
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
                and self.heading_anchor is not None
            ):
                yaw_rate_ref = amounts[3] * np.clip(
                    p["anchor_yaw_gain"] * wrap_angle(self.heading_anchor - yaw),
                    -p["anchor_yaw_limit_rad_s"],
                    p["anchor_yaw_limit_rad_s"],
                )
        return References(
            requested=(
                home + wave if bounce_on else home,
                requested_yaw_rate,
                requested_speed,
            ),
            effective=(h_ref, yaw_rate_ref, speed_ref),
            height_rate=h_rate,
            height_acceleration=h_acc,
            amounts=amounts,
            intents=intents,
            capacity_reason=capacity_reason,
        )

    def pause(self, references: References) -> None:
        p = self.settings
        references.effective = (np.nan, 0.0, 0.0)
        references.height_rate = references.height_acceleration = 0.0
        references.capacity_reason = "supervisor"
        references.amounts = [
            feature.step(enabled, False, 0.0, p["period_s"], p["feature_ramp_s"])
            for feature, enabled in zip(self.features, references.intents)
        ]


def build_tasks(terms, state, observation, references, setup, settings, priority):
    p, k, rate = settings, terms.kinematics, observation.rate
    lean, height, roll, yaw = k.tasks
    h_ref, yaw_rate_ref, speed_ref = references.effective
    h_acc, h_rate, amounts = (
        references.height_acceleration,
        references.height_rate,
        references.amounts,
    )
    gain, a, b = setup.gain, setup.a, setup.b
    forward_estimate = observation.estimated_speed
    balance_state = [0.0, forward_estimate, lean, rate[0]]
    direct = balance_command(gain, balance_state, target_speed=speed_ref)
    desired_lean = float(
        a[3] @ [0.0, forward_estimate - speed_ref, lean, rate[0]] + b[3, 0] * direct
    )
    desired_height = (
        h_acc + p["height_kp"] * (h_ref - height) + p["height_kd"] * (h_rate - rate[1])
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
    return tasks
