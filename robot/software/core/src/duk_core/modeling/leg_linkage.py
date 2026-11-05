"""Canonical planar kinematics for the DUK leg mechanism.

The physical model is expressed directly in one robot frame:

- A is the only fixed point and is defined as A=(0,0).
- +Ox points right.
- +Oy points up.
- positive rotation is counter-clockwise.
- directed D->A is fixed at -30 degrees from +Ox.
- q is the directed counter-clockwise angle from A->B to D->A.

Every other point is derived from equations. No display-time mirror,
solver-to-robot rotation, or post-hoc coordinate correction is used.

B, C and O are fixed material points on one rigid wheel-carrier body. H is an
auxiliary construction point on that same body.
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
    """Raised when the requested four-bar pose is unreachable."""


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
    """Invariant geometry of the rigid B-C-O wheel carrier."""

    bc_mm: float
    bh_mm: float
    oh_mm: float
    o_side_of_bc: Side

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(v) and v > 0.0
            for v in (self.bc_mm, self.bh_mm, self.oh_mm)
        ):
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
        """Derive H and O from the current rigid-body B->C basis."""
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

        h = _sub(
            b,
            _scale(self.bh_mm, e_bc),
        )
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
        """Derive dH/dq and dO/dq from rigid-body kinematics."""
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
    """Immutable DUK geometry and physical topology."""

    ab_mm: float
    cd_mm: float
    ad_mm: float
    bco_body: BCORigidBodyGeometry
    wheel_radius_mm: float
    da_angle_rad: float
    b_side_of_ad: Side
    c_side_of_bd: Side

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(v) and v > 0.0
            for v in (
                self.ab_mm,
                self.cd_mm,
                self.ad_mm,
                self.wheel_radius_mm,
            )
        ):
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
    def bo_mm(self) -> float:
        return self.bco_body.bo_mm

    @property
    def co_mm(self) -> float:
        return self.bco_body.co_mm

    @property
    def wheel_diameter_mm(self) -> float:
        return 2.0 * self.wheel_radius_mm

    @property
    def ad_angle_rad(self) -> float:
        """Directed A->D angle derived from the configured D->A angle."""
        return _normalize_angle_rad(self.da_angle_rad + math.pi)

    def d_point(self, a: Point2 = (0.0, 0.0)) -> Point2:
        """Derive D from A, AD and the A->D direction."""
        return _add(
            a,
            (
                self.ad_mm * math.cos(self.ad_angle_rad),
                self.ad_mm * math.sin(self.ad_angle_rad),
            ),
        )

    def ab_angle_rad(self, linkage_angle_rad: float) -> float:
        """Derive absolute A->B direction from q=directed angle AB->DA."""
        return _normalize_angle_rad(
            self.da_angle_rad - linkage_angle_rad
        )

    def b_point(
        self,
        linkage_angle_rad: float,
        a: Point2 = (0.0, 0.0),
    ) -> Point2:
        """Derive B from A, AB and the current linkage angle q."""
        theta_ab = self.ab_angle_rad(linkage_angle_rad)
        return _add(
            a,
            (
                self.ab_mm * math.cos(theta_ab),
                self.ab_mm * math.sin(theta_ab),
            ),
        )

    def geometric_linkage_angle_limits_rad(
        self,
    ) -> tuple[float, float]:
        """Return the geometric q limits for q=directed angle AB->DA.

        For q in [0, pi], the B-D distance is

            BD^2 = AB^2 + AD^2 + 2*AB*AD*cos(q)

        because A->D is opposite D->A.
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

        denominator = 2.0 * self.ab_mm * self.ad_mm

        def q_for_bd(distance_mm: float) -> float:
            cosine = (
                distance_mm**2
                - self.ab_mm**2
                - self.ad_mm**2
            ) / denominator
            cosine = max(-1.0, min(1.0, cosine))
            return math.acos(cosine)

        q_at_upper_bd = q_for_bd(bd_upper)
        q_at_lower_bd = q_for_bd(bd_lower)
        return (
            min(q_at_upper_bd, q_at_lower_bd),
            max(q_at_upper_bd, q_at_lower_bd),
        )


@dataclass(frozen=True, slots=True)
class LegPose:
    """Solved DUK pose directly in canonical robot coordinates."""

    linkage_angle_rad: float
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
    """d(point)/dq in canonical robot coordinates [mm/rad]."""

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
    linkage_angle_rad: float,
    *,
    c_side_of_bd: Side | None = None,
    tolerance_mm: float = 1e-9,
) -> LegPose:
    """Solve A/B/C/D/H/O from the linkage angle q.

    q is the directed counter-clockwise angle from A->B to D->A:

        theta_AB = theta_DA - q

    The optional C-side override exists only for mathematical comparison.
    """
    if not math.isfinite(linkage_angle_rad):
        raise ValueError("linkage_angle_rad must be finite")
    if not math.isfinite(tolerance_mm) or tolerance_mm <= 0.0:
        raise ValueError("tolerance_mm must be finite and positive")

    # Step 1 — derive the two points connected directly to A.
    # A is the only fixed coordinate. D follows from AD and its fixed directed
    # angle; B follows from AB and the current one-DOF coordinate q.
    a = (0.0, 0.0)
    d = geometry.d_point(a)
    b = geometry.b_point(linkage_angle_rad, a)

    # Step 2 — enforce the physical side of AB relative to the fixed AD link.
    # This is a topology check, not a numerical branch selector.
    ad = _sub(d, a)
    ab = _sub(b, a)
    b_side = _signed_side(ad, ab)
    expected_b_side = _side_sign(geometry.b_side_of_ad)
    if b_side != expected_b_side:
        raise UnreachableConfigurationError(
            "A->B is on the wrong side of directed A->D for the physical "
            f"topology: expected {geometry.b_side_of_ad}"
        )

    # Step 3 — solve C as the intersection of the B/BC and D/CD circles.
    # The named side of directed B->D picks the physical assembly topology.
    bd = _sub(d, b)
    distance = _norm(bd)
    min_distance = abs(geometry.cd_mm - geometry.bc_mm)
    max_distance = geometry.cd_mm + geometry.bc_mm

    if (
        distance < min_distance - tolerance_mm
        or distance > max_distance + tolerance_mm
    ):
        raise UnreachableConfigurationError(
            f"linkage angle has no four-bar solution: BD={distance:.6g} mm, "
            f"required [{min_distance:.6g}, {max_distance:.6g}] mm"
        )
    if distance <= tolerance_mm:
        raise KinematicSingularityError(
            "B and D coincide; circle intersection is undefined"
        )

    ex = _scale(1.0 / distance, bd)
    ey = _left_normal(ex)
    along = (
        geometry.bc_mm**2
        - geometry.cd_mm**2
        + distance**2
    ) / (2.0 * distance)
    height_sq = geometry.bc_mm**2 - along**2

    squared_tolerance = tolerance_mm * max(geometry.bc_mm, geometry.cd_mm, distance)
    if height_sq < -squared_tolerance:
        raise UnreachableConfigurationError(
            "circle intersection failed after tolerance check"
        )

    # At tangency, roundoff in the squared radius can create an artificial
    # nonzero height and hide the exact loop singularity from derivatives.
    tangent = (
        abs(distance - min_distance) <= tolerance_mm
        or abs(distance - max_distance) <= tolerance_mm
    )
    height = 0.0 if tangent else math.sqrt(max(0.0, height_sq))
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
            _scale(
                _side_sign(c_side) * height,
                ey,
            ),
        ),
    )

    # Step 4 — place H and O from the rigid BCO body. No additional degree
    # of freedom exists here: once B and C are known, H and O are determined.
    h, o, e_bc, left_normal_bc = geometry.bco_body.place_from_bc(
        b,
        c,
        tolerance_mm=max(tolerance_mm, 1e-8),
    )

    return LegPose(
        linkage_angle_rad=linkage_angle_rad,
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
    """Return analytic d(point)/dq in canonical robot coordinates."""
    if (
        not math.isfinite(singularity_tolerance)
        or singularity_tolerance <= 0.0
    ):
        raise ValueError(
            "singularity_tolerance must be finite and positive"
        )

    # B is explicit in q, so its derivative is available directly.
    theta_ab = geometry.ab_angle_rad(
        pose.linkage_angle_rad
    )

    # theta_AB = theta_DA - q, hence d(theta_AB)/dq = -1.
    d_b = (
        geometry.ab_mm * math.sin(theta_ab),
        -geometry.ab_mm * math.cos(theta_ab),
    )

    # C is implicit. Differentiate the two constant-length constraints
    # |C-B|=BC and |C-D|=CD, then solve the resulting 2x2 linear system.
    r_bc = _sub(pose.c, pose.b)
    r_dc = _sub(pose.c, pose.d)
    rhs = _dot(r_bc, d_b)
    determinant = _cross(r_bc, r_dc)
    scale = max(
        1.0,
        geometry.bc_mm * geometry.cd_mm,
    )

    if abs(determinant) <= singularity_tolerance * scale:
        raise KinematicSingularityError(
            "four-bar loop-constraint Jacobian is singular"
        )

    d_c = (
        rhs * r_dc[1] / determinant,
        -rhs * r_dc[0] / determinant,
    )
    # H/O derivatives follow from the same rigid BCO body transform used for
    # their positions, which preserves BC/BO/CO at the velocity level too.
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
    start_linkage_angle_rad: float,
    end_linkage_angle_rad: float,
    sample_count: int,
) -> tuple[LegPose, ...]:
    """Sample a continuous q sweep, including both endpoints."""
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    if (
        not math.isfinite(start_linkage_angle_rad)
        or not math.isfinite(end_linkage_angle_rad)
    ):
        raise ValueError("trajectory angle bounds must be finite")

    step = (
        end_linkage_angle_rad
        - start_linkage_angle_rad
    ) / (sample_count - 1)

    return tuple(
        solve_leg_pose(
            geometry,
            start_linkage_angle_rad + index * step,
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
    if (
        units.get("length") != "mm"
        or units.get("angle") != "deg"
    ):
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
