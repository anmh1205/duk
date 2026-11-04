"""Planar mass-property helpers that bridge DUK geometry to balance models.

No DUK mass or inertia values are embedded here. Callers supply CAD-derived or
measured properties in a common frame. Positions use millimetres; mass uses kg;
rotational inertia uses kg*m^2.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

Point2 = tuple[float, float]


@dataclass(frozen=True, slots=True)
class PlanarRigidBody:
    """Mass properties of one body expressed in a common planar frame."""

    mass_kg: float
    center_mm: Point2
    inertia_pitch_at_center_kg_m2: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_kg) or self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be finite and positive")
        if not all(math.isfinite(v) for v in self.center_mm):
            raise ValueError("center_mm coordinates must be finite")
        if (
            not math.isfinite(self.inertia_pitch_at_center_kg_m2)
            or self.inertia_pitch_at_center_kg_m2 < 0.0
        ):
            raise ValueError("center inertia must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class CombinedMassProperties:
    """Combined mass, center of mass, and pitch inertia about that center."""

    mass_kg: float
    center_mm: Point2
    inertia_pitch_at_center_kg_m2: float


@dataclass(frozen=True, slots=True)
class EquivalentPendulum2D:
    """Geometric/mass reduction about one support reference in the sagittal plane."""

    mass_kg: float
    support_point_mm: Point2
    center_mm: Point2
    length_m: float
    angle_from_vertical_rad: float
    inertia_about_center_kg_m2: float
    inertia_about_support_kg_m2: float


def combine_planar_mass_properties(
    bodies: Sequence[PlanarRigidBody],
) -> CombinedMassProperties:
    """Combine planar rigid bodies using a mass-weighted CoM and parallel axis."""
    if not bodies:
        raise ValueError("bodies must not be empty")
    total_mass = sum(body.mass_kg for body in bodies)
    center_x = sum(body.mass_kg * body.center_mm[0] for body in bodies) / total_mass
    center_y = sum(body.mass_kg * body.center_mm[1] for body in bodies) / total_mass
    center = (center_x, center_y)

    inertia = 0.0
    for body in bodies:
        dx_m = (body.center_mm[0] - center_x) / 1000.0
        dy_m = (body.center_mm[1] - center_y) / 1000.0
        inertia += body.inertia_pitch_at_center_kg_m2 + body.mass_kg * (
            dx_m * dx_m + dy_m * dy_m
        )
    return CombinedMassProperties(
        mass_kg=total_mass,
        center_mm=center,
        inertia_pitch_at_center_kg_m2=inertia,
    )


def equivalent_pendulum_2d(
    properties: CombinedMassProperties,
    support_point_mm: Point2,
) -> EquivalentPendulum2D:
    """Reduce combined mass properties to a pendulum about a support reference.

    Coordinates are interpreted as (forward, up) in the caller's chosen
    sagittal frame. Positive angle means the center of mass is forward of the
    support point.
    """
    if not all(math.isfinite(v) for v in support_point_mm):
        raise ValueError("support_point_mm coordinates must be finite")
    dx_m = (properties.center_mm[0] - support_point_mm[0]) / 1000.0
    dz_m = (properties.center_mm[1] - support_point_mm[1]) / 1000.0
    length = math.hypot(dx_m, dz_m)
    if length <= 0.0:
        raise ValueError("center of mass must not coincide with the support point")
    angle = math.atan2(dx_m, dz_m)
    inertia_support = (
        properties.inertia_pitch_at_center_kg_m2
        + properties.mass_kg * length**2
    )
    return EquivalentPendulum2D(
        mass_kg=properties.mass_kg,
        support_point_mm=support_point_mm,
        center_mm=properties.center_mm,
        length_m=length,
        angle_from_vertical_rad=angle,
        inertia_about_center_kg_m2=properties.inertia_pitch_at_center_kg_m2,
        inertia_about_support_kg_m2=inertia_support,
    )
