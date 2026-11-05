"""Canonical planar kinematics for the DUK leg mechanism.

All solved points are expressed directly in one robot frame:

- A is the origin.
- +Ox points right.
- +Oy points up.
- positive rotation is counter-clockwise.
- directed D->A is -30 degrees from +Ox.
- therefore directed A->D is +150 degrees from +Ox.
- ab_angle_rad is the absolute directed angle of A->B from robot +Ox.

Only A is fixed numerically at (0, 0). Every other point is derived from
geometry, topology and the current configuration angle. There is no
post-solution mirror, solver-to-robot rotation, or display-time correction in
this physical model.

B, C and O are fixed material points on one rigid wheel-carrier body. H is an
auxiliary construction point in that same body. Physical sidedness is stored
with named left/right relations; numeric +/- branch signs remain internal to
the circle-intersection calculation.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Mapping

Point2 = tuple[float, float]
Side = Literal["left", "right"]

LEG_GEOMETRY_SCHEMA_VERSION = 4
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
    """Raised when the requested physical four-bar pose is unreachable."""


class KinematicSingularityError(LinkageError):
    """Raised when the loop-constraint Jacobian is singular."""


def _add(a: Point2, b: Point2) -> Point2:
    return a[0] + b[0], a[1] + b[1]


def _sub(a: Point2, b: Point2) -> Point2:
    return a[0] - b[0], a[1] - b[1]


def _scale(k: float, p: Point2) -> Point2:
    return k * p[0], k * p[1]


def _dot(a: Point2, b: Point2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _cross(a: Point2, b: Point2) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _norm(p: Point2) -> float:
    return math.hypot(p[0], p[1])


def _left_normal(unit_vector: Point2) -> Point2:
    return -unit_vector[1], unit_vector[0]


def _normalize_angle_rad(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _validate_side(side: str, *, field_name: str) -> Side:
    if side not in ("left", "right"):
        raise ValueError(
            f"{field_name} must be 'left' or 'right'; got {side!r}"
        )
    return side


def _side_sign(side: Side) -> int:
    return 1 if side == "left" else -1


def _signed_side(
    reference_vector: Point2,
    test_vector: Point2,
    *,
    tolerance: float = 1e-12,
) -> int:
    value = _cross(reference_vector, test_vector)
    if abs(value) <= tolerance:
        return 0
    return 1 if value > 0.0 else -1


@dataclass(frozen=True, slots=True)
class BCORigidBodyGeometry:
    """Geometry of the rigid B-C-O wheel-carrier link."""

    bc_mm: float
    bh_mm: float
    oh_mm: float
    o_side_of_bc: Side

    def __post_init__(self) -> None:
        lengths = (self.bc_mm, self.bh_mm, self.oh_mm)
        if not all(math.isfinite(v) and v > 0.0 for v in lengths):
            raise ValueError("BC, BH and OH must be finite and positive")
        _validate_side(
            self.o_side_of_bc,
            field_name="o_side_of_bc",
        )

    @property
    def bo_mm(self) -> float:
        return math.hypot(self.bh_mm, self.oh_mm)

    @property
    def co_mm(self) -> float:
        return math.hypot(
            self.bh_mm + self.bc_mm,
            self.oh_mm,
        )

    @property
    def signed_double_area_mm2(self) -> float:
        return (
            self.bc_mm
            * _side_sign(self.o_side_of_bc)
            * self.oh_mm
        )

    def place_from_bc(
        self,
        b: Point2,
        c: Point2,
        *,
        tolerance_mm: float = 1e-8,
    ) -> tuple[Point2, Point2, Point2, Point2]:
        """Derive H/O from B/C while preserving the rigid BCO body."""
        bc_vector = _sub(c, b)
        bc_length = _norm(bc_vector)
        if not math.isclose(
            bc_length,
            self.bc_mm,
            rel_tol=0.0,
            abs_tol=tolerance_mm,
        ):
            raise LinkageError(
                "B-C distance violates rigid BCO geometry: "
                f"{bc_length:.9g} mm != {self.bc_mm:.9g} mm"
            )

        e_bc = _scale(1.0 / self.bc_mm, bc_vector)
        left_normal_bc = _left_normal(e_bc)
        h = _sub(b, _scale(self.bh_mm, e_bc))
        o = _add(
            h,
            _scale(
                _side_sign(self.o_side_of_bc) * self.oh_mm,
                left_normal_bc,
            ),
        )
        return h, o, e_bc, left_normal_bc

    def point_derivatives(
        self,
        d_b: Point2,
        d_c: Point2,
    ) -> tuple[Point2, Point2]:
        """Derive dH/dalpha and dO/dalpha from rigid-body kinematics."""
        d_e_bc = _scale(
            1.0 / self.bc_mm,
            _sub(d_c, d_b),
        )
        d_left_normal = _left_normal(d_e_bc)
        d_h = _sub(
            d_b,
            _scale(self.bh_mm, d_e_bc),
        )
        d_o = _add(
            d_h,
            _scale(
                _side_sign(self.o_side_of_bc) * self.oh_mm,
                d_left_normal,
            ),
        )
        return d_h, d_o


@dataclass(frozen=True, slots=True)
class LegGeometry:
    """Immutable DUK geometry in the canonical robot frame."""

    ab_mm: float
    cd_mm: float
    ad_mm: float
    bco_body: BCORigidBodyGeometry
    wheel_radius_mm: float
    da_angle_rad: float
    b_side_of_ad: Side
    c_side_of_bd: Side

    def __post_init__(self) -> None:
        values = (
            self.ab_mm,
            self.cd_mm,
            self.ad_mm,
            self.wheel_radius_mm,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError(
                "link lengths and wheel radius must be finite and positive"
            )
        if not math.isfinite(self.da_angle_rad):
            raise ValueError("da_angle_rad must be finite")
        _validate_side(
            self.b_side_of_ad,
            field_name="b_side_of_ad",
        )
        _validate_side(
            self.c_side_of_bd,
            field_name="c_side_of_bd",
        )

    @property
    def bc_mm(self) -> float:
        return self.bco_body.bc_mm

    @property
    def bh_mm(self) -> float:
        return self.bco_body.bh_mm

    @property
    def oh_mm(self) -> float:
        return self.bco_body.oh_mm

    @property
    def wheel_diameter_mm(self) -> float:
        return 2.0 * self.wheel_radius_mm

    @property
    def bo_mm(self) -> float:
        return self.bco_body.bo_mm

    @property
    def co_mm(self) -> float:
        return self.bco_body.co_mm

    @property
    def ad_angle_rad(self) -> float:
        """Absolute robot-frame direction of A->D."""
        return _normalize_angle_rad(self.da_angle_rad + math.pi)

    @property
    def d_point(self) -> Point2:
        """Point D derived from AD length and A->D direction."""
        return (
            self.ad_mm * math.cos(self.ad_angle_rad),
            self.ad_mm * math.sin(self.ad_angle_rad),
        )

    def relative_angle_limits_rad(self) -> tuple[float, float]:
        """Return positive absolute A->D/A->B separation limits."""
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

    def geometric_ab_angle_limits_rad(self) -> tuple[float, float]:
        """Return physical absolute A->B angle limits in robot coordinates."""
        relative_lower, relative_upper = self.relative_angle_limits_rad()
        sign = _side_sign(self.b_side_of_ad)
        first = self.ad_angle_rad + sign * relative_lower
        second = self.ad_angle_rad + sign * relative_upper
        return min(first, second), max(first, second)


@dataclass(frozen=True, slots=True)
class LegPose:
    """Solved DUK leg pose directly in canonical robot coordinates."""

    ab_angle_rad: float
    a: Point2
    b: Point2
    c: Point2
    d: Point2
    h: Point2
    o: Point2
    e_bc: Point2
    left_normal_bc: Point2

    def point(self, name: str) -> Point2:
        key = name.lower()
        if key not in {"a", "b", "c", "d", "h", "o"}:
            raise KeyError(f"unknown linkage point: {name}")
        return getattr(self, key)

    @property
    def bco_triangle(self) -> tuple[Point2, Point2, Point2]:
        return self.b, self.c, self.o


@dataclass(frozen=True, slots=True)
class LegJacobian:
    """d(point)/d(ab_angle) in canonical robot coordinates [mm/rad]."""

    d_b: Point2
    d_c: Point2
    d_h: Point2
    d_o: Point2

    def point(self, name: str) -> Point2:
        key = name.lower()
        if key not in {"b", "c", "h", "o"}:
            raise KeyError(f"Jacobian is not defined for fixed point {name}")
        return getattr(self, f"d_{key}")


def solve_leg_pose(
    geometry: LegGeometry,
    ab_angle_rad: float,
    *,
    c_side_of_bd: Side | None = None,
    tolerance_mm: float = 1e-9,
) -> LegPose:
    """Solve A/B/C/D/H/O directly in canonical robot coordinates.

    ab_angle_rad is the absolute A->B direction from robot +Ox. An optional
    c_side_of_bd override exists only for mathematical comparison; normal robot
    use relies on the physical topology from configuration.
    """
    if not math.isfinite(ab_angle_rad):
        raise ValueError("ab_angle_rad must be finite")
    if not math.isfinite(tolerance_mm) or tolerance_mm <= 0.0:
        raise ValueError("tolerance_mm must be finite and positive")

    a = (0.0, 0.0)
    d = geometry.d_point
    b = (
        geometry.ab_mm * math.cos(ab_angle_rad),
        geometry.ab_mm * math.sin(ab_angle_rad),
    )

    ad = _sub(d, a)
    ab = _sub(b, a)
    b_side = _signed_side(ad, ab)
    expected_b_side = _side_sign(geometry.b_side_of_ad)
    if b_side != expected_b_side:
        raise UnreachableConfigurationError(
            "A->B is on the wrong side of directed A->D for the physical "
            f"topology: expected {geometry.b_side_of_ad}"
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
            f"AB angle has no four-bar solution: BD={distance:.6g} mm, "
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

    c_side = (
        geometry.c_side_of_bd
        if c_side_of_bd is None
        else _validate_side(
            c_side_of_bd,
            field_name="c_side_of_bd",
        )
    )
    c = _add(
        b,
        _add(
            _scale(along, ex),
            _scale(_side_sign(c_side) * height, ey),
        ),
    )

    h, o, e_bc, left_normal_bc = geometry.bco_body.place_from_bc(
        b,
        c,
        tolerance_mm=max(tolerance_mm, 1e-8),
    )

    return LegPose(
        ab_angle_rad=ab_angle_rad,
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
    """Return analytic d(point)/d(ab_angle) in robot coordinates."""
    if (
        not math.isfinite(singularity_tolerance)
        or singularity_tolerance <= 0.0
    ):
        raise ValueError(
            "singularity_tolerance must be finite and positive"
        )

    alpha = pose.ab_angle_rad
    d_b = (
        -geometry.ab_mm * math.sin(alpha),
        geometry.ab_mm * math.cos(alpha),
    )
    r_bc = _sub(pose.c, pose.b)
    r_dc = _sub(pose.c, pose.d)
    rhs = _dot(r_bc, d_b)
    determinant = _cross(r_bc, r_dc)
    scale = max(1.0, geometry.bc_mm * geometry.cd_mm)
    if abs(determinant) <= singularity_tolerance * scale:
        raise KinematicSingularityError(
            "four-bar loop-constraint Jacobian is singular"
        )

    d_c = (
        rhs * r_dc[1] / determinant,
        -rhs * r_dc[0] / determinant,
    )
    d_h, d_o = geometry.bco_body.point_derivatives(
        d_b,
        d_c,
    )
    return LegJacobian(
        d_b=d_b,
        d_c=d_c,
        d_h=d_h,
        d_o=d_o,
    )


def sample_leg_trajectory(
    geometry: LegGeometry,
    start_ab_angle_rad: float,
    end_ab_angle_rad: float,
    sample_count: int,
) -> tuple[LegPose, ...]:
    """Sample a continuous physical A->B angle sweep."""
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    if (
        not math.isfinite(start_ab_angle_rad)
        or not math.isfinite(end_ab_angle_rad)
    ):
        raise ValueError("trajectory angle bounds must be finite")

    step = (
        end_ab_angle_rad - start_ab_angle_rad
    ) / (sample_count - 1)
    return tuple(
        solve_leg_pose(
            geometry,
            start_ab_angle_rad + index * step,
        )
        for index in range(sample_count)
    )


def geometry_from_mapping(
    data: Mapping[str, Any],
) -> LegGeometry:
    """Validate and convert the physical geometry mapping."""
    schema_version = data.get("schema_version")
    if schema_version != LEG_GEOMETRY_SCHEMA_VERSION:
        raise ValueError(
            "unsupported leg geometry schema_version: "
            f"{schema_version!r}; expected {LEG_GEOMETRY_SCHEMA_VERSION}"
        )

    units = data.get("units", {})
    if units.get("length") != "mm" or units.get("angle") != "deg":
        raise ValueError(
            "leg geometry requires length='mm' and angle='deg'"
        )

    links = data["links_mm"]
    bco = data["bco_rigid_body"]
    wheel = data["wheel_mm"]
    robot_frame = data["robot_frame"]
    four_bar = data["four_bar_assembly"]

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
            "wheel diameter must equal two times wheel radius"
        )

    return LegGeometry(
        ab_mm=float(links["AB"]),
        cd_mm=float(links["CD"]),
        ad_mm=float(links["AD"]),
        bco_body=BCORigidBodyGeometry(
            bc_mm=float(bco["BC_mm"]),
            bh_mm=float(bco["BH_mm"]),
            oh_mm=float(bco["OH_mm"]),
            o_side_of_bc=_validate_side(
                str(bco["o_side_of_directed_bc"]),
                field_name="o_side_of_directed_bc",
            ),
        ),
        wheel_radius_mm=radius,
        da_angle_rad=math.radians(
            float(robot_frame["da_angle_deg"])
        ),
        b_side_of_ad=_validate_side(
            str(four_bar["b_side_of_directed_ad"]),
            field_name="b_side_of_directed_ad",
        ),
        c_side_of_bd=_validate_side(
            str(four_bar["c_side_of_directed_bd"]),
            field_name="c_side_of_directed_bd",
        ),
    )


def load_leg_geometry_json(
    path: str | Path,
) -> LegGeometry:
    """Load DUK leg geometry from a JSON physical-configuration file."""
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    return geometry_from_mapping(data)
