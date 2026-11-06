"""Rigid placement on flat ground; canonical linkage coordinates stay unchanged.

These are normal-contact geometry constraints, not force/contact dynamics or a
no-slip rolling law. Preview wheels are finite-width circular cylinders.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .leg_linkage import LegGeometry, LegPose, Point2


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

    def transform_point(self, point: tuple[float, float, float]):
        x, y, z = point
        c, s = math.cos(self.roll_rad), math.sin(self.roll_rad)
        return x, c * y - s * z, self.body_height_mm + s * y + c * z

    def leg_point(self, pose: LegPose, side: str, name: str):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        x, z = pose.point(name)
        y = self.leg_spacing_mm / 2 * (1 if side == "left" else -1)
        return self.transform_point((x, y, z))

    @property
    def wheel_vertical_extent_mm(self):
        # Exact support of a finite cylinder tilted about body x. Its rim/edge,
        # not just the wheel center, must reach the plane without penetration.
        return self.wheel_radius_mm * math.cos(
            self.roll_rad
        ) + self.wheel_width_mm / 2 * abs(math.sin(self.roll_rad))

    def ground_gap_mm(self, pose: LegPose, side: str):
        return self.leg_point(pose, side, "O")[2] - self.wheel_vertical_extent_mm


def place_biped_on_ground(
    geometry: LegGeometry,
    left: LegPose,
    right: LegPose,
    *,
    leg_spacing_mm: float,
    wheel_width_mm: float,
) -> BipedGroundPlacement:
    """Solve body heave/roll so both equal-size cylindrical tires touch z=0.

    With Δz=z_OL-z_OR and spacing S, S sin(roll)+Δz cos(roll)=0.
    The principal solution keeps the body upright (cos(roll)>0). Horizontal body
    origin is a frame choice; tangential slip, forces and impacts are not solved.
    """
    if not math.isfinite(leg_spacing_mm) or leg_spacing_mm <= 0:
        raise ValueError("leg_spacing_mm must be finite and positive")
    if not math.isfinite(wheel_width_mm) or wheel_width_mm < 0:
        raise ValueError("wheel_width_mm must be finite and non-negative")
    roll = math.atan2(right.o[1] - left.o[1], leg_spacing_mm)
    support = geometry.wheel_radius_mm * math.cos(roll) + wheel_width_mm / 2 * abs(
        math.sin(roll)
    )
    height = support - (left.o[1] + right.o[1]) / 2 * math.cos(roll)
    return BipedGroundPlacement(
        height, roll, leg_spacing_mm, geometry.wheel_radius_mm, wheel_width_mm
    )
