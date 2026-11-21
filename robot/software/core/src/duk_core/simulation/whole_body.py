"""Closed-loop research scenarios for constrained spatial allocation.

This module coordinates simulation components. Physical equations, estimation
algorithms and the allocator remain owned by their respective core layers.
"""

from __future__ import annotations

from collections.abc import Callable, Generator, Mapping
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
from ._whole_body_state import ContactObserver, PlantGuard, contact_modes
from ._whole_body_tasks import ReferencePolicy, build_tasks
from .whole_body_schedule import ScheduledChange, compile_schedule


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
    friction_belief: np.ndarray
    actual_friction: np.ndarray  # Synthetic plant input, not a measured channel.
    position: np.ndarray
    rotation: np.ndarray
    joints: np.ndarray
    velocity: np.ndarray
    wheel_centers: np.ndarray
    wheel_velocity: np.ndarray
    contact_points: np.ndarray
    center_of_mass: np.ndarray
    sensor: np.ndarray  # Mean wheel odometry [m/s], independent acceleration [m/s²].
    sensor_valid: np.ndarray
    slip_detected: np.ndarray
    terminal_state: SpatialState
    terminal_force: np.ndarray
    terminal_contact_modes: tuple[str, str] | None
    terminal_integrated_effort: np.ndarray | None
    task_names: tuple[tuple[str, ...], ...]
    task_targets: np.ndarray
    task_scales: np.ndarray
    active_constraints: tuple[tuple[str, ...], ...]
    fallback_gain: np.ndarray
    fallback_a: np.ndarray
    fallback_b: np.ndarray
    scheduled_changes: tuple[ScheduledChange, ...] = ()

    def state_at(self, index: int) -> SpatialState:
        """World position, body-to-world rotation, joints and generalized velocity."""
        return SpatialState(
            self.position[index],
            self.rotation[index],
            self.joints[index],
            self.velocity[index],
        )


def _friction_pair(value, name: str) -> np.ndarray:
    pair = np.asarray(value, dtype=float)
    if pair.ndim == 0:
        pair = np.full(2, pair)
    if pair.shape != (2,) or not np.isfinite(pair).all() or np.any(pair < 0):
        raise ValueError(
            f"{name} must be nonnegative and finite, scalar or [left, right]"
        )
    return pair


def _allocate_sample(
    terms,
    state,
    tasks,
    observation,
    setup,
    settings,
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
    if fault == "deadline":
        # A synthetic late return makes this fault independent of CPU speed.
        return Allocation("deadline", None, np.nan, 0.0)
    inject_infeasible = fault == "infeasible"
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


def _velocity_jump(state: SpatialState, linear, angular) -> SpatialState:
    velocity = state.velocity.copy()
    velocity[:3] += linear
    velocity[3:6] += angular
    return SpatialState(state.position, state.rotation, state.joints, velocity)


class WholeBodyRun:
    """Advance the sampled NB6 plant one command interval at a time.

    The in-memory run retains the observer, scheduler and solver state between
    calls. A trace may be requested after the first interval, including after
    an early stop. Advance and trace must be called from a single owner.
    """

    def __init__(
        self,
        model: SpatialBiped,
        hip_motor: LinearTorqueSpeed,
        weights: Mapping[str, Any],
        settings: Mapping[str, Any],
        scenario: Mapping[str, Any],
        *,
        method: Literal["weighted", "hierarchical"],
        priority: Literal["balance_first", "legs_first"],
    ) -> None:
        self._steps = _whole_body_steps(
            model,
            hip_motor,
            weights,
            settings,
            scenario,
            method=method,
            priority=priority,
        )
        self._snapshot: Callable[[], WholeBodyTrace] | None = None
        self._result: WholeBodyTrace | None = None
        self.time_s = 0.0

    @property
    def finished(self) -> bool:
        return self._result is not None

    @property
    def has_samples(self) -> bool:
        return self._snapshot is not None or self.finished

    def advance(self) -> float:
        """Advance one command interval, ending sooner if a guard stops it."""
        if self._result is None:
            self._snapshot = None
            try:
                self.time_s, self._snapshot = next(self._steps)
            except StopIteration as stop:
                if stop.value is None:
                    raise RuntimeError(
                        "The whole-body run ended without a trace"
                    ) from None
                self._result = stop.value
                self.time_s = self._result.terminal_time
        return self.time_s

    def trace(self) -> WholeBodyTrace:
        """Return the accepted data through the current integration boundary."""
        if self._result is not None:
            return self._result
        if self._snapshot is None:
            raise ValueError("Advance the whole-body run before reading its trace")
        return self._snapshot()

    def run_to_end(self) -> WholeBodyTrace:
        while not self.finished:
            self.advance()
        return self.trace()


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
    An impulse is delivered at the first command sample at or after its requested
    time, like the sampled friction and fault changes. No impulse is delivered
    after the last command sample, including at the terminal horizon.
    An explicit schedule replaces the legacy grip switch, impulse, fault window
    and drive/turn commands. Other reference settings remain the defaults for
    the schedule. A scheduled height offset is a target step with no impulsive
    feedforward terms.
    """
    return WholeBodyRun(
        model, hip_motor, weights, settings, scenario, method=method, priority=priority
    ).run_to_end()


def _whole_body_steps(
    model: SpatialBiped,
    hip_motor: LinearTorqueSpeed,
    weights: Mapping[str, Any],
    settings: Mapping[str, Any],
    scenario: Mapping[str, Any],
    *,
    method: Literal["weighted", "hierarchical"],
    priority: Literal["balance_first", "legs_first"],
) -> Generator[tuple[float, Callable[[], WholeBodyTrace]], None, WholeBodyTrace]:
    if method not in {"weighted", "hierarchical"}:
        raise ValueError("Unknown allocation method")
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
    friction_before = _friction_pair(scenario["friction"], "friction")
    schedule = scenario.get("schedule")
    scheduled_changes = ()
    overrides = {}
    by_tick = {}
    if schedule is None:
        friction_after = _friction_pair(scenario["friction_after"], "friction_after")
        friction_change = scenario["friction_change_s"]
        if not np.isfinite(friction_change) or friction_change < 0:
            raise ValueError("friction_change_s must be finite and nonnegative")
        impulse_time = scenario["impulse_at_s"]
        linear_impulse = np.asarray(scenario["velocity_impulse_m_s"], dtype=float)
        angular_impulse = np.asarray(
            scenario["angular_velocity_impulse_rad_s"], dtype=float
        )
        if (
            not np.isfinite(impulse_time)
            or impulse_time < 0
            or linear_impulse.shape != (3,)
            or angular_impulse.shape != (3,)
            or not np.isfinite(np.r_[linear_impulse, angular_impulse]).all()
        ):
            raise ValueError(
                "Expected a finite nonnegative impulse time and two finite 3-vectors"
            )
        impulse_tick = np.searchsorted(times, impulse_time)
    else:
        scheduled_changes = compile_schedule(schedule, times)
        for change in scheduled_changes:
            if change.tick is not None:
                by_tick.setdefault(change.tick, {})[change.field] = change.value
        overrides = {"drive_speed_m_s": 0.0, "turn_rate_rad_s": 0.0}
    setup = initialize(model, weights, settings, scenario)
    state = setup.state
    observer = ContactObserver(settings, {**scenario, "friction": friction_before})
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
    integrated_effort = None
    rows, labels = [], []

    def snapshot() -> WholeBodyTrace:
        terminal_modes = None
        if guard.terminal_time == rows[-1]["time"]:
            terminal_modes = labels[-1]["contact_modes"]
        elif np.isfinite(guard.terminal_pose).all():
            current = model.kinematics(state)
            supported = (current.gaps <= settings["contact_gap_m"]) & (
                previous_force[:, 2] > 0
            )
            slip = (current.contact @ state.velocity)[:, :2]
            terminal_modes = contact_modes(
                supported,
                np.linalg.norm(slip, axis=1) > settings["spatial_slip_threshold_m_s"],
            )
        return WholeBodyTrace(
            **{name: np.asarray([row[name] for row in rows]) for name in rows[0]},
            **{name: tuple(row[name] for row in labels) for name in labels[0]},
            stop_reason=guard.stop_reason,
            terminal_time=guard.terminal_time,
            terminal_pose=guard.terminal_pose,
            peak_abs_lean_roll=guard.peak_abs_lean_roll,
            terminal_state=state,
            terminal_force=previous_force,
            terminal_contact_modes=terminal_modes,
            terminal_integrated_effort=integrated_effort,
            fallback_gain=setup.gain,
            fallback_a=setup.a[1:, 1:],
            fallback_b=setup.b[1:],
            scheduled_changes=scheduled_changes,
        )

    for tick, (time, end) in enumerate(zip(times, ends)):
        if schedule is None:
            if tick == impulse_tick:
                state = _velocity_jump(state, linear_impulse, angular_impulse)
            fault = (
                scenario["fault"]
                if scenario["fault_start_s"] <= time < scenario["fault_end_s"]
                else "none"
            )
            friction = friction_after if time >= friction_change else friction_before
        else:
            changes = by_tick.get(tick)
            if changes:
                linear = changes.get("velocity_impulse_m_s")
                angular = changes.get("angular_velocity_impulse_rad_s")
                if linear is not None or angular is not None:
                    state = _velocity_jump(state, linear or 0.0, angular or 0.0)
                overrides.update(changes)
            fault = overrides.get("fault", "none")
            friction = np.array(
                [
                    overrides.get("friction_left", friction_before[0]),
                    overrides.get("friction_right", friction_before[1]),
                ]
            )
        terms = model.evaluate(state)
        observation = observer.observe(model, state, terms, previous_force, tick)
        fallen = guard.observe(terms.kinematics, previous_force, time)
        sensor_valid = fault != "sensor"
        targets = references.update(
            state,
            terms,
            observation,
            time,
            supervisor_mode=supervisor.mode,
            sensor_valid=sensor_valid,
            fallen=fallen,
            previous_usage=previous_usage,
            overrides=overrides,
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
                position=state.position.copy(),
                rotation=state.rotation.copy(),
                joints=state.joints.copy(),
                velocity=state.velocity.copy(),
                hip_angles=state.joints[:2].copy(),
                contact_gap=terms.kinematics.gaps,
                contact_speed=observation.contact_speed,
                wheel_centers=terms.kinematics.centers[model.wheels],
                wheel_velocity=terms.kinematics.linear[model.wheels] @ state.velocity,
                contact_points=terms.kinematics.contact_points,
                center_of_mass=np.average(
                    terms.kinematics.centers, axis=0, weights=model.masses
                ),
                sensor=(observation.odometry, observation.acceleration)
                if sensor_valid
                else (np.nan, np.nan),
                sensor_valid=sensor_valid,
                slip_detected=observation.slip_detected,
                requested_reference=targets.requested,
                effective_reference=targets.effective,
                requested_effort=command.requested,
                applied_effort=command.applied,
                effort_limit=effort_bounds,
                force=previous_force.copy(),
                allocated_force=command.allocated_force,
                friction_belief=observer.friction_belief.copy(),
                actual_friction=friction.copy(),
                feature_amount=targets.amounts,
                feature_intent=targets.intents,
                true_and_estimated_speed=(
                    observation.speed,
                    observation.estimated_speed,
                ),
                task_targets=[task.target for task in tasks],
                task_scales=[task.scale for task in tasks],
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
                task_names=tuple(task.name for task in tasks),
                active_constraints=allocation.active_constraints,
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
        integrated_effort = command.applied
        if guard.stop_reason is not None:
            break
        if tick + 1 < len(times):
            yield guard.terminal_time, snapshot

    return snapshot()
