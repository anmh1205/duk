"""Command selection after allocation, including the study's stop/fallback policy."""

from dataclasses import dataclass

import numpy as np

from ..control.sagittal_balance import (
    balance_command,
    project_torque,
    traction_interval,
)
from ..modeling.rolling_reduction import symmetric_forward_wheel_input


@dataclass(frozen=True)
class Command:
    requested: np.ndarray
    applied: np.ndarray
    allocated_force: np.ndarray
    warm_start: np.ndarray | None
    mode: str
    source: str


class CommandPolicy:
    def __init__(self, settings, setup, hip_motor, supervisor):
        self.settings, self.setup = settings, setup
        self.hip_motor, self.supervisor = hip_motor, supervisor

    def select(
        self,
        result,
        state,
        observation,
        effort_bounds,
        previous_force,
        friction_belief,
        *,
        sensor_valid,
        fallen,
    ) -> Command:
        p, setup, hip_motor, supervisor = (
            self.settings,
            self.setup,
            self.hip_motor,
            self.supervisor,
        )
        q0, hip_limits, hip_feedforward = (
            setup.hip_angle,
            setup.hip_limits,
            setup.hip_feedforward,
        )
        pendulum, gain = setup.pendulum, setup.gain
        lean, roll = observation.pose[[0, 2]]
        rate, modes = observation.rate, observation.modes
        balance_state = [0.0, observation.estimated_speed, lean, rate[0]]
        solver_status = result.status
        hips, hip_rates = state.joints[:2], state.velocity[6:8]
        predicted_hips = hips + p["lookahead_s"] * hip_rates
        fallback_valid = (
            all(mode == "rolling" for mode in modes)
            and abs(roll) < np.deg2rad(p["fallback_roll_deg"])
            and np.max(abs(state.joints[:2] - q0))
            < np.deg2rad(p["fallback_hip_error_deg"])
            and np.all(hips > hip_limits[0])
            and np.all(hips < hip_limits[1])
            and np.all(predicted_hips > hip_limits[0])
            and np.all(predicted_hips < hip_limits[1])
            and np.all(abs(hip_rates) < hip_motor.no_load_speed)
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
            command_source = "stop"
        elif result.solution is not None:
            effort = result.effort
            command_source = "allocator"
        elif fallback_valid:
            bound = 2 * min(effort_bounds[2:])
            normal_share = previous_force[:, 2] / previous_force[:, 2].sum()
            # Equal wheel moments must fit the weaker measured support as well.
            budget = 2 * min(normal_share * friction_belief)
            interval = traction_interval(pendulum, lean, rate[0], bound, budget)
            torque = project_torque(
                balance_command(gain, balance_state), bound, interval
            )
            if torque.applied is None:
                effort = np.zeros(4)
                mode = supervisor.mode = "stop"
                command_source = "stop"
            else:
                effort = symmetric_forward_wheel_input(torque.applied)
                requested = symmetric_forward_wheel_input(torque.requested)
                requested[:2] = (
                    hip_feedforward
                    + p["fallback_hip_kp"] * (q0 - hips)
                    - (p["fallback_hip_kd"] * hip_rates)
                )
                effort[:2] = np.clip(
                    requested[:2], -effort_bounds[:2], effort_bounds[:2]
                )
                command_source = "balance_fallback"
        else:
            effort = np.zeros(4)
            mode = supervisor.mode = "stop"
            command_source = "stop"
        return Command(
            requested=requested,
            applied=effort,
            allocated_force=allocated_force,
            warm_start=warm_start,
            mode=mode,
            source=command_source,
        )
