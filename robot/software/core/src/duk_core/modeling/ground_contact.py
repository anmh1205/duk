"""Rigid placement on flat ground; canonical linkage coordinates stay unchanged.

These are normal-contact geometry constraints, not force/contact dynamics or a
no-slip rolling law. Preview wheels are finite-width circular cylinders.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .leg_linkage import LegGeometry, LegPose, Point2

Point3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class GroundedLegPose:
    """One planar leg expressed in a world frame whose ground is z=0 [mm]."""

    local_pose: LegPose
    body_height_mm: float

    @property
    def linkage_angle_rad(self):
        return self.local_pose.linkage_angle_rad

    def point(self, name: str) -> Point2:
        x, z = self.local_pose.point(name)
        return x, z + self.body_height_mm

    @property
    def a(self):
        return self.point("A")

    @property
    def b(self):
        return self.point("B")

    @property
    def c(self):
        return self.point("C")

    @property
    def d(self):
        return self.point("D")

    @property
    def h(self):
        return self.point("H")

    @property
    def o(self):
        return self.point("O")

    @property
    def bco_triangle(self):
        return self.b, self.c, self.o


def place_leg_on_ground(geometry: LegGeometry, pose: LegPose) -> GroundedLegPose:
    """Translate the whole leg/body, never the wheel alone: z_O=wheel radius."""
    return GroundedLegPose(pose, geometry.wheel_radius_mm - pose.o[1])


@dataclass(frozen=True, slots=True)
class BipedGroundPlacement:
    """One common body transform for both legs; all lengths use millimetres."""

    body_height_mm: float
    roll_rad: float
    leg_spacing_mm: float
    wheel_radius_mm: float
    wheel_width_mm: float
    support_levels_mm: tuple[float, float] = (0.0, 0.0)
    sagittal_mounts_mm: tuple[Point2, Point2] = ((0.0, 0.0), (0.0, 0.0))
    lateral_center_mm: float = 0.0

    def transform_point(self, point: tuple[float, float, float]):
        x, y, z = point
        c, s = math.cos(self.roll_rad), math.sin(self.roll_rad)
        return x, c * y - s * z, self.body_height_mm + s * y + c * z

    def leg_point(self, pose: LegPose, side: str, name: str):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        x, z = pose.point(name)
        index = 0 if side == "left" else 1
        dx, dz = self.sagittal_mounts_mm[index]
        y = self.lateral_center_mm + self.leg_spacing_mm / 2 * (
            1 if side == "left" else -1
        )
        return self.transform_point((x + dx, y, z + dz))

    @property
    def wheel_vertical_extent_mm(self):
        # Exact support of a finite cylinder tilted about body x. Its rim/edge,
        # not just the wheel center, must reach the plane without penetration.
        return self.wheel_radius_mm * math.cos(
            self.roll_rad
        ) + self.wheel_width_mm / 2 * abs(math.sin(self.roll_rad))

    def ground_gap_mm(self, pose: LegPose, side: str):
        level = self.support_levels_mm[0 if side == "left" else 1]
        return (
            self.leg_point(pose, side, "O")[2] - self.wheel_vertical_extent_mm - level
        )


def place_biped_on_ground(
    geometry: LegGeometry,
    left: LegPose,
    right: LegPose,
    *,
    leg_spacing_mm: float,
    wheel_width_mm: float,
    support_levels_mm: tuple[float, float] = (0.0, 0.0),
    sagittal_mounts_mm: tuple[Point2, Point2] = ((0.0, 0.0), (0.0, 0.0)),
    lateral_center_mm: float = 0.0,
) -> BipedGroundPlacement:
    """Solve body heave/roll with an independent horizontal support per tire.

    With Δz=z_OL-z_OR and spacing S, S sin(roll)+Δz cos(roll)=Δsupport.
    The principal solution keeps the body upright (cos(roll)>0). Horizontal body
    origin is a frame choice; tangential slip, forces and impacts are not solved.
    """
    if not math.isfinite(leg_spacing_mm) or leg_spacing_mm <= 0:
        raise ValueError("leg_spacing_mm must be finite and positive")
    if not math.isfinite(wheel_width_mm) or wheel_width_mm < 0:
        raise ValueError("wheel_width_mm must be finite and non-negative")
    if len(support_levels_mm) != 2 or not all(map(math.isfinite, support_levels_mm)):
        raise ValueError("Supply two finite support levels in left/right order")
    if (
        not math.isfinite(lateral_center_mm)
        or not all(
            len(mount) == 2 and all(map(math.isfinite, mount))
            for mount in sagittal_mounts_mm
        )
        or len(sagittal_mounts_mm) != 2
    ):
        raise ValueError("Expected finite hip mount offsets")
    zl, zr = (
        pose.o[1] + mount[1] for pose, mount in zip((left, right), sagittal_mounts_mm)
    )
    dz = zl - zr
    difference = support_levels_mm[0] - support_levels_mm[1]
    ratio = difference / math.hypot(leg_spacing_mm, dz)
    if abs(ratio) >= 1:
        raise ValueError("The support levels do not admit a regular stance")
    roll = math.asin(ratio) - math.atan2(dz, leg_spacing_mm)
    if abs(roll) >= math.pi / 2:
        raise ValueError("The support levels do not admit an upright stance")
    support = geometry.wheel_radius_mm * math.cos(roll) + wheel_width_mm / 2 * abs(
        math.sin(roll)
    )
    height = (
        sum(support_levels_mm) / 2
        + support
        - (zl + zr) / 2 * math.cos(roll)
        - lateral_center_mm * math.sin(roll)
    )
    return BipedGroundPlacement(
        height,
        roll,
        leg_spacing_mm,
        geometry.wheel_radius_mm,
        wheel_width_mm,
        tuple(support_levels_mm),
        sagittal_mounts_mm,
        lateral_center_mm,
    )


def grounded_thin_disk_motion(
    placement: BipedGroundPlacement,
    left: LegPose,
    right: LegPose,
    velocities_mm_s: tuple[Point2, Point2],
    accelerations_mm_s2: tuple[Point2, Point2],
) -> tuple[tuple[Point3, Point3], tuple[Point3, Point3]]:
    """World wheel-center derivatives for fixed supports and zero-width disks."""
    if placement.wheel_width_mm != 0:
        raise ValueError("Differential support requires the thin-disk placement")
    c, s = math.cos(placement.roll_rad), math.sin(placement.roll_rad)
    spacing = placement.leg_spacing_mm
    zl, zr = (
        pose.o[1] + mount[1]
        for pose, mount in zip((left, right), placement.sagittal_mounts_mm)
    )
    dz = zl - zr
    dv = velocities_mm_s[0][1] - velocities_mm_s[1][1]
    da = accelerations_mm_s2[0][1] - accelerations_mm_s2[1][1]
    denominator = spacing * c - dz * s
    roll_rate = -dv * c / denominator
    support_delta = placement.support_levels_mm[0] - placement.support_levels_mm[1]
    roll_acceleration = (
        -da * c + 2 * dv * s * roll_rate + support_delta * roll_rate**2
    ) / denominator
    velocities, accelerations = [], []
    radius = placement.wheel_radius_mm
    for side, z, (vx, vz), (ax, az) in zip(
        (1, -1),
        (zl, zr),
        velocities_mm_s,
        accelerations_mm_s2,
    ):
        y = placement.lateral_center_mm + side * spacing / 2
        dy_dr = -y * s - z * c
        d2y_dr2 = -y * c + z * s
        velocities.append((vx, dy_dr * roll_rate - vz * s, -radius * s * roll_rate))
        accelerations.append(
            (
                ax,
                dy_dr * roll_acceleration
                + d2y_dr2 * roll_rate**2
                - 2 * vz * c * roll_rate
                - az * s,
                -radius * (s * roll_acceleration + c * roll_rate**2),
            )
        )
    return (velocities[0], velocities[1]), (accelerations[0], accelerations[1])
