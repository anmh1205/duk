"""DUK planar four-bar leg and wheel-center kinematics.

Two coordinate frames are deliberately separated.

Solver frame:
- A = (0, 0)
- D = (AD, 0)
- +x is aligned with the directed vector A->D
- input_angle_rad is the directed angle from A->D to A->B

Robot frame:
- +Ox points right
- +Oy points up
- positive rotation is counter-clockwise
- the configured directed vector D->A defines mechanism orientation

The physical DUK convention is angle(D->A, +Ox) = -30 deg. The opposite
vector A->D is therefore +150 deg. That derived A->D angle is exactly the
rotation from the solver frame into the robot frame.

The wheel-center construction keeps H, B and C collinear in the order H-B-C,
with BH opposite B->C and OH perpendicular to BC.

All lengths are millimetres. Public API angle values are radians; JSON
configuration angles use the declared degree unit.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

Point2 = tuple[float, float]

LEG_GEOMETRY_SCHEMA_VERSION = 2
ROBOT_FRAME_CONVENTION = {
    "origin": "A",
    "x_axis": "+Ox points right",
    "y_axis": "+Oy points up",
    "positive_rotation": "counter_clockwise",
    "reference_vector": "DA",
}


class LinkageError(ValueError):
    """Base class for DUK linkage geometry errors."""


class UnreachableConfigurationError(LinkageError):
    """Raised when the four-bar circles do not intersect."""


class KinematicSingularityError(LinkageError):
    """Raised when the loop-constraint Jacobian is singular."""


@dataclass(frozen=True, slots=True)
class LegGeometry:
    """Immutable DUK leg geometry and robot-frame orientation.

    da_angle_rad is the directed angle of D->A measured from robot +Ox.
    Solver loop closure is orientation-independent; robot orientation is
    applied only when transforming solver results into the robot frame.
    """

    ab_mm: float
    bc_mm: float
    cd_mm: float
    ad_mm: float
    bh_mm: float
    oh_mm: float
    wheel_radius_mm: float
    da_angle_rad: float
    assembly_branch: int
    wheel_side: int = -1

    def __post_init__(self) -> None:
        values = (
            self.ab_mm,
            self.bc_mm,
            self.cd_mm,
            self.ad_mm,
            self.bh_mm,
            self.oh_mm,
            self.wheel_radius_mm,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError(
                "all geometry lengths and wheel radius must be finite and positive"
            )
        if not math.isfinite(self.da_angle_rad):
            raise ValueError("da_angle_rad must be finite")
        _validate_branch(self.assembly_branch)
        if self.wheel_side not in (-1, 1):
            raise ValueError("wheel_side must be either -1 or +1")

    @property
    def wheel_diameter_mm(self) -> float:
        return 2.0 * self.wheel_radius_mm

    @property
    def ad_angle_rad(self) -> float:
        """Directed A->D angle in the robot frame."""
        return _normalize_angle_rad(self.da_angle_rad + math.pi)

    @property
    def solver_to_robot_rotation_rad(self) -> float:
        """Rotation that maps solver-frame vectors into the robot frame."""
        return self.ad_angle_rad

    @property
    def bo_mm(self) -> float:
        """Derived direct B-to-wheel-center distance."""
        return math.hypot(self.bh_mm, self.oh_mm)

    @property
    def co_mm(self) -> float:
        """Derived direct C-to-wheel-center distance."""
        return math.hypot(self.bh_mm + self.bc_mm, self.oh_mm)

    def geometric_angle_limits_rad(self) -> tuple[float, float]:
        """Return four-bar existence limits for input angles in [0, pi].

        At either returned boundary the B/C circle-intersection construction is
        tangent, so the boundary itself is geometrically singular and should
        not be treated as an operating limit.
        """
        bd_lower = max(
            abs(self.cd_mm - self.bc_mm),
            abs(self.ab_mm - self.ad_mm),
        )
        bd_upper = min(
            self.cd_mm + self.bc_mm,
            self.ab_mm + self.ad_mm,
        )
        if bd_lower > bd_upper:
            raise UnreachableConfigurationError(
                "link lengths admit no four-bar assembly"
            )

        def angle_for_bd(distance_mm: float) -> float:
            numerator = self.ab_mm**2 + self.ad_mm**2 - distance_mm**2
            denominator = 2.0 * self.ab_mm * self.ad_mm
            cosine = max(-1.0, min(1.0, numerator / denominator))
            return math.acos(cosine)

        return angle_for_bd(bd_lower), angle_for_bd(bd_upper)


@dataclass(frozen=True, slots=True)
class LegPose:
    """Solved planar pose of one DUK leg in a common 2-D frame."""

    input_angle_rad: float
    assembly_branch: int
    a: Point2
    b: Point2
    c: Point2
    d: Point2
    h: Point2
    o: Point2
    e_bc: Point2
    left_normal_bc: Point2

    def point(self, name: str) -> Point2:
        """Return a named point using A/B/C/D/H/O (case-insensitive)."""
        key = name.lower()
        if key not in {"a", "b", "c", "d", "h", "o"}:
            raise KeyError(f"unknown linkage point: {name}")
        return getattr(self, key)


@dataclass(frozen=True, slots=True)
class LegJacobian:
    """Derivatives of solved points with respect to input angle [mm/rad]."""

    d_b: Point2
    d_c: Point2
    d_h: Point2
    d_o: Point2

    def point(self, name: str) -> Point2:
        key = name.lower()
        if key not in {"b", "c", "h", "o"}:
            raise KeyError(f"Jacobian is not defined for fixed point {name}")
        return getattr(self, f"d_{key}")


def _add(a: Point2, b: Point2) -> Point2:
    return a[0] + b[0], a[1] + b[1]


def _sub(a: Point2, b: Point2) -> Point2:
    return a[0] - b[0], a[1] - b[1]


def _scale(k: float, p: Point2) -> Point2:
    return k * p[0], k * p[1]


def _dot(a: Point2, b: Point2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _norm(p: Point2) -> float:
    return math.hypot(p[0], p[1])


def _left_normal(unit_vector: Point2) -> Point2:
    return -unit_vector[1], unit_vector[0]


def _normalize_angle_rad(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _validate_branch(assembly_branch: int) -> None:
    if assembly_branch not in (-1, 1):
        raise ValueError("assembly_branch must be either -1 or +1")


def _resolve_branch(
    geometry: LegGeometry,
    assembly_branch: int | None,
) -> int:
    """Use an explicit branch override or the physical branch from config."""
    branch = geometry.assembly_branch if assembly_branch is None else assembly_branch
    _validate_branch(branch)
    return branch


def solve_leg_pose(
    geometry: LegGeometry,
    input_angle_rad: float,
    *,
    assembly_branch: int | None = None,
    tolerance_mm: float = 1e-9,
) -> LegPose:
    """Solve A/B/C/D/H/O for one input angle.

    ``assembly_branch`` optionally overrides the physical branch loaded from
    configuration. If omitted, ``geometry.assembly_branch`` is used. Explicit
    overrides remain useful for comparing the two mathematical solutions.
    """
    branch = _resolve_branch(geometry, assembly_branch)
    if not math.isfinite(input_angle_rad):
        raise ValueError("input_angle_rad must be finite")
    if not math.isfinite(tolerance_mm) or tolerance_mm <= 0.0:
        raise ValueError("tolerance_mm must be finite and positive")

    a = (0.0, 0.0)
    d = (geometry.ad_mm, 0.0)
    b = (
        geometry.ab_mm * math.cos(input_angle_rad),
        geometry.ab_mm * math.sin(input_angle_rad),
    )

    bd = _sub(d, b)
    distance = _norm(bd)
    min_distance = abs(geometry.cd_mm - geometry.bc_mm)
    max_distance = geometry.cd_mm + geometry.bc_mm
    if (
        distance < min_distance - tolerance_mm
        or distance > max_distance + tolerance_mm
    ):
        raise UnreachableConfigurationError(
            f"input angle has no four-bar solution: BD={distance:.6g} mm, "
            f"required [{min_distance:.6g}, {max_distance:.6g}] mm"
        )
    if distance <= tolerance_mm:
        raise KinematicSingularityError(
            "B and D coincide; circle intersection is undefined"
        )

    ex = _scale(1.0 / distance, bd)
    ey = _left_normal(ex)
    along = (
        geometry.bc_mm**2 - geometry.cd_mm**2 + distance**2
    ) / (2.0 * distance)
    height_sq = geometry.bc_mm**2 - along**2
    if height_sq < -tolerance_mm:
        raise UnreachableConfigurationError(
            "circle intersection failed after tolerance check"
        )
    height = math.sqrt(max(0.0, height_sq))
    c = _add(
        b,
        _add(
            _scale(along, ex),
            _scale(branch * height, ey),
        ),
    )

    bc_vector = _sub(c, b)
    bc_length = _norm(bc_vector)
    if bc_length <= tolerance_mm:
        raise KinematicSingularityError("B and C coincide")
    e_bc = _scale(1.0 / bc_length, bc_vector)
    left_normal_bc = _left_normal(e_bc)
    h = _sub(b, _scale(geometry.bh_mm, e_bc))
    o = _add(
        h,
        _scale(geometry.wheel_side * geometry.oh_mm, left_normal_bc),
    )

    return LegPose(
        input_angle_rad=input_angle_rad,
        assembly_branch=branch,
        a=a,
        b=b,
        c=c,
        d=d,
        h=h,
        o=o,
        e_bc=e_bc,
        left_normal_bc=left_normal_bc,
    )


def leg_jacobian(
    geometry: LegGeometry,
    pose: LegPose,
    *,
    singularity_tolerance: float = 1e-10,
) -> LegJacobian:
    """Return the analytic point Jacobian d(point)/d(input_angle).

    C is differentiated implicitly from the two loop-closure constraints. This
    avoids finite-difference noise and exposes the actual four-bar singularity
    through the 2x2 constraint determinant.
    """
    if (
        not math.isfinite(singularity_tolerance)
        or singularity_tolerance <= 0.0
    ):
        raise ValueError(
            "singularity_tolerance must be finite and positive"
        )

    alpha = pose.input_angle_rad
    d_b = (
        -geometry.ab_mm * math.sin(alpha),
        geometry.ab_mm * math.cos(alpha),
    )
    r_bc = _sub(pose.c, pose.b)
    r_dc = _sub(pose.c, pose.d)
    rhs = _dot(r_bc, d_b)
    determinant = r_bc[0] * r_dc[1] - r_bc[1] * r_dc[0]
    scale = max(1.0, geometry.bc_mm * geometry.cd_mm)
    if abs(determinant) <= singularity_tolerance * scale:
        raise KinematicSingularityError(
            "four-bar loop-constraint Jacobian is singular"
        )

    d_c = (
        rhs * r_dc[1] / determinant,
        -rhs * r_dc[0] / determinant,
    )
    d_e_bc = _scale(
        1.0 / geometry.bc_mm,
        _sub(d_c, d_b),
    )
    d_left_normal = _left_normal(d_e_bc)
    d_h = _sub(
        d_b,
        _scale(geometry.bh_mm, d_e_bc),
    )
    d_o = _add(
        d_h,
        _scale(
            geometry.wheel_side * geometry.oh_mm,
            d_left_normal,
        ),
    )
    return LegJacobian(
        d_b=d_b,
        d_c=d_c,
        d_h=d_h,
        d_o=d_o,
    )


def sample_leg_trajectory(
    geometry: LegGeometry,
    start_angle_rad: float,
    end_angle_rad: float,
    sample_count: int,
    *,
    assembly_branch: int | None = None,
) -> tuple[LegPose, ...]:
    """Sample a continuous input-angle sweep, including both endpoints."""
    branch = _resolve_branch(geometry, assembly_branch)
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    if (
        not math.isfinite(start_angle_rad)
        or not math.isfinite(end_angle_rad)
    ):
        raise ValueError("trajectory angle bounds must be finite")

    step = (end_angle_rad - start_angle_rad) / (sample_count - 1)
    return tuple(
        solve_leg_pose(
            geometry,
            start_angle_rad + index * step,
            assembly_branch=branch,
        )
        for index in range(sample_count)
    )


def transform_vector(
    vector: Point2,
    rotation_rad: float,
) -> Point2:
    """Rotate a 2-D vector without translation."""
    if not all(math.isfinite(v) for v in (*vector, rotation_rad)):
        raise ValueError("vector and rotation must be finite")
    c = math.cos(rotation_rad)
    s = math.sin(rotation_rad)
    return (
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
    )


def transform_point(
    point: Point2,
    rotation_rad: float,
    translation_mm: Point2 = (0.0, 0.0),
) -> Point2:
    """Apply a rigid 2-D transform to a point."""
    if not all(
        math.isfinite(v)
        for v in (*point, rotation_rad, *translation_mm)
    ):
        raise ValueError(
            "point, rotation, and translation must be finite"
        )
    rotated = transform_vector(point, rotation_rad)
    return (
        translation_mm[0] + rotated[0],
        translation_mm[1] + rotated[1],
    )


def transform_pose(
    pose: LegPose,
    rotation_rad: float,
    translation_mm: Point2 = (0.0, 0.0),
) -> LegPose:
    """Transform all pose points and orientation vectors into another frame."""

    def rotate_vector(vector: Point2) -> Point2:
        return transform_vector(vector, rotation_rad)

    return LegPose(
        input_angle_rad=pose.input_angle_rad,
        assembly_branch=pose.assembly_branch,
        a=transform_point(pose.a, rotation_rad, translation_mm),
        b=transform_point(pose.b, rotation_rad, translation_mm),
        c=transform_point(pose.c, rotation_rad, translation_mm),
        d=transform_point(pose.d, rotation_rad, translation_mm),
        h=transform_point(pose.h, rotation_rad, translation_mm),
        o=transform_point(pose.o, rotation_rad, translation_mm),
        e_bc=rotate_vector(pose.e_bc),
        left_normal_bc=rotate_vector(pose.left_normal_bc),
    )


def transform_jacobian(
    jacobian: LegJacobian,
    rotation_rad: float,
) -> LegJacobian:
    """Rotate all point-derivative vectors into another 2-D frame."""
    return LegJacobian(
        d_b=transform_vector(jacobian.d_b, rotation_rad),
        d_c=transform_vector(jacobian.d_c, rotation_rad),
        d_h=transform_vector(jacobian.d_h, rotation_rad),
        d_o=transform_vector(jacobian.d_o, rotation_rad),
    )


def pose_in_robot_frame(
    geometry: LegGeometry,
    pose: LegPose,
    origin_mm: Point2 = (0.0, 0.0),
) -> LegPose:
    """Transform a solver-frame pose into the DUK robot frame.

    Solver +x is A->D. The transform rotation is therefore the derived
    robot-frame angle of A->D, opposite to the configured D->A vector.
    """
    return transform_pose(
        pose,
        geometry.solver_to_robot_rotation_rad,
        origin_mm,
    )


def jacobian_in_robot_frame(
    geometry: LegGeometry,
    jacobian: LegJacobian,
) -> LegJacobian:
    """Rotate a solver-frame Jacobian into the DUK robot frame."""
    return transform_jacobian(
        jacobian,
        geometry.solver_to_robot_rotation_rad,
    )


def geometry_from_mapping(
    data: Mapping[str, Any],
) -> LegGeometry:
    """Create LegGeometry from the repository physical-config schema."""
    schema_version = data.get("schema_version")
    if schema_version != LEG_GEOMETRY_SCHEMA_VERSION:
        raise ValueError(
            "unsupported leg geometry schema_version: "
            f"{schema_version!r}; expected {LEG_GEOMETRY_SCHEMA_VERSION}"
        )

    units = data.get("units", {})
    if (
        units.get("length") != "mm"
        or units.get("angle") != "deg"
    ):
        raise ValueError(
            "leg geometry config must declare length=mm and angle=deg"
        )

    links = data["links_mm"]
    wheel_offset = data["wheel_center_offset_mm"]
    wheel = data["wheel_mm"]
    robot_frame = data["robot_frame"]

    for key, expected in ROBOT_FRAME_CONVENTION.items():
        if robot_frame.get(key) != expected:
            raise ValueError(
                f"robot_frame.{key} must be {expected!r}; "
                f"got {robot_frame.get(key)!r}"
            )
    radius = float(wheel["radius"])
    diameter = float(wheel["diameter"])
    if not math.isclose(
        diameter,
        2.0 * radius,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "wheel diameter must equal twice the wheel radius"
        )

    return LegGeometry(
        ab_mm=float(links["AB"]),
        bc_mm=float(links["BC"]),
        cd_mm=float(links["CD"]),
        ad_mm=float(links["AD"]),
        bh_mm=float(wheel_offset["BH"]),
        oh_mm=float(wheel_offset["OH"]),
        wheel_radius_mm=radius,
        da_angle_rad=math.radians(
            float(robot_frame["da_angle_deg"])
        ),
        assembly_branch=int(data["assembly_branch"]),
        wheel_side=int(wheel_offset["wheel_side"]),
    )


def load_leg_geometry_json(
    path: str | Path,
) -> LegGeometry:
    """Load DUK leg geometry from a JSON physical-configuration file."""
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, Mapping):
        raise ValueError(
            "leg geometry JSON root must be an object"
        )
    return geometry_from_mapping(data)
