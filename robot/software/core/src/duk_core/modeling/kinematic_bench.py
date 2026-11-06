"""Fixed-body kinematic bench; no physical actuators, loads or contact model."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .leg_differential import leg_second_derivative
from .leg_linkage import LegGeometry, LegPose, leg_jacobian, solve_leg_pose


@dataclass(frozen=True, slots=True)
class BenchSample:
    """Calculated profile signals, not measured physical motion."""

    pose: LegPose
    q_dot_rad_s: float
    q_ddot_rad_s2: float
    wheel_velocity_mm_s: tuple[float, float]
    wheel_acceleration_mm_s2: tuple[float, float]
    closure_error_mm: float
    carrier_error_mm: float
    carrier_area_error_mm2: float
    boundary_margin_rad: float
    loop_sine: float


def sample_bench_leg(
    geometry: LegGeometry,
    q_rad: float,
    q_dot_rad_s: float = 0,
    q_ddot_rad_s2: float = 0,
) -> BenchSample:
    """Use canonical geometry and analytic derivatives to inspect one leg."""
    if not all(math.isfinite(v) for v in (q_dot_rad_s, q_ddot_rad_s2)):
        raise ValueError("Profile rates must be finite")
    pose = solve_leg_pose(geometry, q_rad)
    jac = leg_jacobian(geometry, pose)
    second = leg_second_derivative(geometry, pose)
    velocity = tuple(v * q_dot_rad_s for v in jac.d_o)
    acceleration = tuple(
        j * q_ddot_rad_s2 + h * q_dot_rad_s**2 for j, h in zip(jac.d_o, second.dd_o)
    )

    def distances(a, b):
        return math.dist(pose.point(a), pose.point(b))

    closure = max(
        abs(distances(a, b) - length)
        for a, b, length in (
            ("A", "B", geometry.ab_mm),
            ("B", "C", geometry.bc_mm),
            ("C", "D", geometry.cd_mm),
            ("A", "D", geometry.ad_mm),
        )
    )
    carrier = max(
        abs(distances(a, b) - length)
        for a, b, length in (
            ("B", "C", geometry.bc_mm),
            ("B", "O", geometry.bo_mm),
            ("C", "O", geometry.co_mm),
        )
    )
    bc = (pose.c[0] - pose.b[0], pose.c[1] - pose.b[1])
    bo = (pose.o[0] - pose.b[0], pose.o[1] - pose.b[1])
    dc = (pose.c[0] - pose.d[0], pose.c[1] - pose.d[1])
    area = bc[0] * bo[1] - bc[1] * bo[0]
    lower, upper = geometry.geometric_linkage_angle_limits_rad()
    return BenchSample(
        pose,
        q_dot_rad_s,
        q_ddot_rad_s2,
        velocity,
        acceleration,
        closure,
        carrier,
        abs(area - geometry.bco_body.signed_double_area_mm2),
        min(q_rad - lower, upper - q_rad),
        abs(bc[0] * dc[1] - bc[1] * dc[0]) / (geometry.bc_mm * geometry.cd_mm),
    )


class KinematicBench:
    """Two independent geometric q values with a bounded cosine test profile.

    The 5-degree reserve and 10-second nominal period are visualization choices,
    not physical operating limits. Manual pose changes and pause are geometric
    inspection operations, not torque-controlled or acceleration-limited motion.
    """

    def __init__(self, geometry: LegGeometry, initial_q_rad=math.radians(75)):
        self.geometry = geometry
        lower, upper = geometry.geometric_linkage_angle_limits_rad()
        self.reserve_rad = math.radians(5)
        self.bounds = (lower + self.reserve_rad, upper - self.reserve_rad)
        if self.bounds[0] >= self.bounds[1]:
            raise ValueError("No workspace remains after the numerical preview reserve")
        self.playing = False
        self.speed = 1.0
        self.wall_time_s = 0.0
        self.phase = 0.0
        self.pose_revision = 0
        self._start = (initial_q_rad, initial_q_rad)
        self._amplitude = (0.0, 0.0)
        self.samples = ()
        self.set_angles(initial_q_rad, initial_q_rad)

    def validate_angle(self, q):
        """Reject angles outside the reserved numerical preview workspace."""
        if not math.isfinite(q) or not self.bounds[0] <= q <= self.bounds[1]:
            raise ValueError(
                f"q must be inside preview bounds {math.degrees(self.bounds[0]):.3f}"
                f" .. {math.degrees(self.bounds[1]):.3f} degrees"
            )

    def set_angles(self, left_q_rad: float, right_q_rad: float):
        """Atomically accept both poses; invalid requests preserve the last state."""
        angles = (left_q_rad, right_q_rad)
        for q in angles:
            self.validate_angle(q)
        samples = tuple(sample_bench_leg(self.geometry, q) for q in angles)
        self.samples = samples
        self.pose_revision += 1
        self.playing = False
        self.phase = 0.0

    def set_playing(self, playing: bool):
        if playing and not self.playing:
            self._start = tuple(s.pose.linkage_angle_rad for s in self.samples)
            amplitudes = []
            for q in self._start:
                down, up = q - self.bounds[0], self.bounds[1] - q
                # q=start+A*(1-cos phase), initially stationary, with no jump.
                amplitudes.append(0.4 * up if up >= down else -0.4 * down)
            self._amplitude = tuple(amplitudes)
            self.phase = 0.0
        if not playing:
            self.samples = tuple(
                sample_bench_leg(self.geometry, s.pose.linkage_angle_rad)
                for s in self.samples
            )
        self.playing = bool(playing)

    def set_speed(self, speed: float):
        if not math.isfinite(speed) or not 0.25 <= speed <= 2:
            raise ValueError("Preview speed must be between 0.25 and 2")
        self.speed = speed

    def advance(self, dt_s: float):
        if not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError("Elapsed time must be finite and non-negative")
        self.wall_time_s += dt_s
        if self.playing:
            omega = 2 * math.pi / 10 * self.speed
            self.phase = (self.phase + omega * dt_s) % (2 * math.pi)
            self.samples = tuple(
                sample_bench_leg(
                    self.geometry,
                    start + amplitude * (1 - math.cos(self.phase)),
                    amplitude * omega * math.sin(self.phase),
                    amplitude * omega**2 * math.cos(self.phase),
                )
                for start, amplitude in zip(self._start, self._amplitude)
            )
        return self.samples
