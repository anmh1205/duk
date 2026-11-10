"""Closed-loop research scenarios for constrained spatial allocation.

This module coordinates simulation components. Physical equations, estimation
algorithms and the allocator remain owned by their respective core layers.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from time import perf_counter
from typing import Any, Literal

import numpy as np

from ..control.task_policy import Supervisor
from ..control.whole_body import Allocation, allocate
from ..modeling.hip_load import LinearTorqueSpeed
from ..modeling.spatial_biped import SpatialBiped, SpatialState
from ._sampled import sample_intervals
from ._whole_body_command import CommandPolicy
from ._whole_body_setup import initialize
from ._whole_body_state import ContactObserver, PlantGuard
from ._whole_body_tasks import ReferencePolicy, build_tasks


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
    solver_status: tuple[str, ...]
    supervisor: tuple[str, ...]
    feature_status: tuple[tuple[str, ...], ...]
    contact_modes: tuple[tuple[str, str], ...]
    capacity_reason: tuple[str, ...]
    command_source: tuple[str, ...]
    stop_reason: str | None
    terminal_time: float
    terminal_pose: np.ndarray  # NaN if a travel violation leaves the linkage domain.
    peak_abs_lean_roll: np.ndarray


def _allocate_sample(
    terms,
    state,
    tasks,
    observation,
    setup,
    settings,
    scenario,
    hip_motor,
    effort_bounds,
    friction_belief,
    warm_start,
    *,
    method,
    fault,
    sensor_valid,
    fallen,
) -> Allocation:
    if not sensor_valid or fallen:
        status = "skipped_sensor" if not sensor_valid else "skipped_safety"
        return Allocation(status, None, np.nan, 0.0)
    if fault and scenario["fault"] == "deadline":
        # A synthetic late return makes this fault independent of CPU speed.
        return Allocation("deadline", None, np.nan, 0.0)
    inject_infeasible = fault and scenario["fault"] == "infeasible"
    hip_limits = setup.hip_limits
    return allocate(
        terms,
        state,
        tasks,
        observation.modes,
        friction=np.zeros(2) if inject_infeasible else friction_belief,
        effort_bounds=np.zeros(4) if inject_infeasible else effort_bounds,
        hip_limits=(hip_limits[1] + 0.1, hip_limits[1] + 0.2)
        if inject_infeasible
        else hip_limits,
        hip_speed_limit=hip_motor.no_load_speed,
        lookahead=settings["lookahead_s"],
        velocity_stabilization=settings["contact_stabilization"],
        deadline_s=settings["deadline_s"],
        method=method,
        initial=None if inject_infeasible else warm_start,
    )


def run_whole_body(
    model: SpatialBiped,
    hip_motor: LinearTorqueSpeed,
    weights: Mapping[str, Any],
    settings: Mapping[str, Any],
    scenario: Mapping[str, Any],
    *,
    method: Literal["weighted", "hierarchical"],
    priority: Literal["balance_first", "legs_first"],
) -> WholeBodyTrace:
    """Compare allocation policies on the same compliant spatial plant.

    Sensor inputs are simulated. The independent velocity channel represents
    already-compensated axle acceleration and pose is supplied by the simulator.
    The supervisor and its zero-command stop are policies for this study.
    """
    if priority not in {"balance_first", "legs_first"}:
        raise ValueError("Unknown task priority")
    period, duration, substeps = (
        settings["period_s"],
        scenario["duration_s"],
        settings["substeps"],
    )
    if (
        not np.isfinite([period, duration]).all()
        or min(period, duration) <= 0
        or not isinstance(substeps, (int, np.integer))
        or substeps < 1
    ):
        raise ValueError(
            "Expected positive simulation times and an integer substep count"
        )
    times, ends = sample_intervals(duration, period)
    setup = initialize(model, weights, settings, scenario)
    state = setup.state
    observer = ContactObserver(settings, scenario)
    references = ReferencePolicy(settings, scenario, setup.home_height)
    supervisor = Supervisor(
        settings["fallback_after"],
        settings["recover_after"],
        successes=settings["recover_after"],
    )
    commands = CommandPolicy(settings, setup, hip_motor, supervisor)
    guard = PlantGuard(settings)
    previous_force = np.array([[0.0, 0.0, setup.normal_at_rest]] * 2)
    warm_start, previous_usage = None, 0.0
    rows, labels = [], []

    for tick, (time, end) in enumerate(zip(times, ends)):
        if tick == round(scenario["impulse_at_s"] / period):
            velocity = state.velocity.copy()
            velocity[:3] += scenario["velocity_impulse_m_s"]
            velocity[3:6] += scenario["angular_velocity_impulse_rad_s"]
            state = SpatialState(state.position, state.rotation, state.joints, velocity)
        fault = scenario["fault_start_s"] <= time < scenario["fault_end_s"]
        friction = np.full(
            2,
            scenario["friction_after"]
            if time >= scenario["friction_change_s"]
            else scenario["friction"],
        )
        terms = model.evaluate(state)
        observation = observer.observe(model, state, terms, previous_force, tick)
        fallen = guard.observe(terms.kinematics, previous_force, time)
        sensor_valid = not (fault and scenario["fault"] == "sensor")
        targets = references.update(
            state,
            terms,
            observation,
            time,
            supervisor_mode=supervisor.mode,
            sensor_valid=sensor_valid,
            fallen=fallen,
            previous_usage=previous_usage,
        )
        tasks = build_tasks(
            terms, state, observation, targets, setup, settings, priority
        )
        effort_bounds = np.r_[
            [
                hip_motor.joint_limit(v) * scenario["hip_capacity"]
                for v in state.velocity[6:8]
            ],
            np.full(2, settings["wheel_limit_N_m"]),
        ]
        start = perf_counter()
        allocation = _allocate_sample(
            terms,
            state,
            tasks,
            observation,
            setup,
            settings,
            scenario,
            hip_motor,
            effort_bounds,
            observer.friction_belief,
            warm_start,
            method=method,
            fault=fault,
            sensor_valid=sensor_valid,
            fallen=fallen,
        )
        solve_seconds = perf_counter() - start
        command = commands.select(
            allocation,
            state,
            observation,
            effort_bounds,
            previous_force,
            observer.friction_belief,
            sensor_valid=sensor_valid,
            fallen=fallen,
        )
        warm_start = command.warm_start
        if command.source != "allocator":
            references.pause(targets)
        previous_usage = np.max(
            np.divide(
                abs(command.applied),
                effort_bounds,
                out=np.zeros(4),
                where=effort_bounds > 0,
            )
        )
        # Named columns keep logging independent of dataclass field order.
        rows.append(
            dict(
                time=time,
                pose=observation.pose,
                pose_rate=observation.rate,
                position_xy=state.position[:2].copy(),
                hip_angles=state.joints[:2].copy(),
                contact_gap=terms.kinematics.gaps,
                contact_speed=observation.contact_speed,
                requested_reference=targets.requested,
                effective_reference=targets.effective,
                requested_effort=command.requested,
                applied_effort=command.applied,
                effort_limit=effort_bounds,
                force=previous_force.copy(),
                allocated_force=command.allocated_force,
                feature_amount=targets.amounts,
                feature_intent=targets.intents,
                true_and_estimated_speed=(
                    observation.speed,
                    observation.estimated_speed,
                ),
                solver_seconds=solve_seconds,
                solver_residual=allocation.residual,
            )
        )
        labels.append(
            dict(
                solver_status=allocation.status,
                supervisor=command.mode,
                feature_status=references.statuses,
                contact_modes=observation.modes,
                capacity_reason=targets.capacity_reason,
                command_source=command.source,
            )
        )
        if command.mode == "stop":
            guard.stop_reason = (
                "sensor"
                if not sensor_valid
                else "flight_or_fall"
                if fallen
                else "no_valid_command"
            )
            break
        state, previous_force = guard.advance(
            model,
            state,
            command.applied,
            friction,
            setup,
            time,
            end,
            duration,
        )
        if guard.stop_reason is not None:
            break
    return WholeBodyTrace(
        **{name: np.asarray([row[name] for row in rows]) for name in rows[0]},
        **{name: tuple(row[name] for row in labels) for name in labels[0]},
        stop_reason=guard.stop_reason,
        terminal_time=guard.terminal_time,
        terminal_pose=guard.terminal_pose,
        peak_abs_lean_roll=guard.peak_abs_lean_roll,
    )
