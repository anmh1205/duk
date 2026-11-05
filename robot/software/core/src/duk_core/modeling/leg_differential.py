"""Second-order and inverse kinematics in the canonical planar leg frame."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .leg_linkage import (
    KinematicSingularityError,
    LegGeometry,
    LegPose,
    Point2,
    UnreachableConfigurationError,
    _add,
    _cross,
    _dot,
    _left_normal,
    _norm,
    _normalize_angle_rad,
    _scale,
    _sub,
    leg_jacobian,
    solve_leg_pose,
)


@dataclass(frozen=True, slots=True)
class LegSecondDerivative:
    """d²(point)/dq² in mm/rad²; this is not yet a time acceleration."""

    dd_b: Point2
    dd_c: Point2
    dd_h: Point2
    dd_o: Point2

    def point(self, name: str) -> Point2:
        key = name.lower()
        if key not in {"b", "c", "h", "o"}:
            raise KeyError(f"Second derivative is not defined for fixed point {name}")
        return getattr(self, f"dd_{key}")


def leg_second_derivative(geometry: LegGeometry, pose: LegPose) -> LegSecondDerivative:
    """Differentiate both loop constraints again, using the same physical branch.

    With r=C-B and s=C-D, the linear system is
    r·C'' = r·B'' - |C'-B'|², s·C'' = -|C'|².
    Singular configurations are rejected by the first-order solver.
    """
    jac = leg_jacobian(geometry, pose)
    dd_b = _scale(-1, _sub(pose.b, pose.a))
    r = _sub(pose.c, pose.b)
    s = _sub(pose.c, pose.d)
    relative_velocity = _sub(jac.d_c, jac.d_b)
    rhs_r = _dot(r, dd_b) - _dot(relative_velocity, relative_velocity)
    rhs_s = -_dot(jac.d_c, jac.d_c)
    determinant = _cross(r, s)
    dd_c = (
        (rhs_r * s[1] - r[1] * rhs_s) / determinant,
        (r[0] * rhs_s - rhs_r * s[0]) / determinant,
    )
    # H/O are affine in B/C and their rotated difference, at every order.
    dd_h, dd_o = geometry.bco_body.point_derivatives(dd_b, dd_c)
    return LegSecondDerivative(dd_b, dd_c, dd_h, dd_o)


def point_acceleration(
    geometry: LegGeometry,
    pose: LegPose,
    point: str,
    q_dot_rad_s: float,
    q_ddot_rad_s2: float,
) -> Point2:
    """Return p_ddot = J(q) q_ddot + p''(q) q_dot², in mm/s²."""
    if not all(math.isfinite(v) for v in (q_dot_rad_s, q_ddot_rad_s2)):
        raise ValueError("q velocity and acceleration must be finite")
    first = leg_jacobian(geometry, pose).point(point)
    second = leg_second_derivative(geometry, pose).point(point)
    return _add(_scale(q_ddot_rad_s2, first), _scale(q_dot_rad_s**2, second))


def inverse_wheel_position(
    geometry: LegGeometry,
    target_mm: Point2,
    *,
    tolerance_mm: float = 1e-6,
) -> tuple[LegPose, ...]:
    """Return all physical-branch poses reaching O=(x,y), without projection.

    One DOF cannot reach arbitrary independent x/y targets. B must lie on
    circles (A, AB) and (O, BO); each candidate is checked against the existing
    forward solver, including the rigid carrier orientation and topology.
    Results use principal q angles in [-pi, pi), sorted in increasing order.
    """
    if len(target_mm) != 2 or not all(math.isfinite(v) for v in target_mm):
        raise ValueError("target_mm must contain two finite coordinates")
    if not math.isfinite(tolerance_mm) or tolerance_mm <= 0:
        raise ValueError("tolerance_mm must be finite and positive")
    distance = _norm(target_mm)
    ab, bo = geometry.ab_mm, geometry.bo_mm
    if distance <= tolerance_mm:
        if abs(ab - bo) <= tolerance_mm:
            raise KinematicSingularityError("Inverse B circles are concentric")
        raise UnreachableConfigurationError("Wheel target cannot be reached")
    if distance < abs(ab - bo) - tolerance_mm or distance > ab + bo + tolerance_mm:
        raise UnreachableConfigurationError(
            "Wheel target lies outside inverse B circles"
        )
    along = (ab**2 - bo**2 + distance**2) / (2 * distance)
    height_sq = ab**2 - along**2
    # Squared-length roundoff is compared in mm², not directly against mm.
    if height_sq < -tolerance_mm * max(ab, bo, distance):
        raise UnreachableConfigurationError("Inverse B circles do not intersect")
    axis = _scale(1 / distance, target_mm)
    normal = _left_normal(axis)
    found = []
    for sign in (-1, 1):
        b = _add(
            _scale(along, axis), _scale(sign * math.sqrt(max(0, height_sq)), normal)
        )
        q = _normalize_angle_rad(geometry.da_angle_rad - math.atan2(b[1], b[0]))
        try:
            pose = solve_leg_pose(geometry, q)
        except (UnreachableConfigurationError, KinematicSingularityError):
            continue
        if _norm(_sub(pose.o, target_mm)) <= tolerance_mm and not any(
            _norm(_sub(pose.b, other.b)) <= tolerance_mm for other in found
        ):
            found.append(pose)
    if not found:
        raise UnreachableConfigurationError(
            "No physical-branch pose reaches the wheel target"
        )
    return tuple(sorted(found, key=lambda pose: pose.linkage_angle_rad))
