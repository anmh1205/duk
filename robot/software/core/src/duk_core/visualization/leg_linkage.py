"""Matplotlib visualization for solved DUK leg-linkage results."""

from __future__ import annotations

import math
from collections.abc import Sequence

from duk_core.modeling.leg_linkage import (
    LegGeometry,
    LegPose,
    leg_jacobian,
)


def _plt():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Install duk-core[research] to use visualization helpers"
        ) from exc
    return plt


def draw_leg_linkage(
    pose: LegPose,
    geometry: LegGeometry,
    ax=None,
    *,
    show_wheel: bool = True,
    show_auxiliary: bool = True,
    show_rigid_carrier: bool = True,
    annotate: bool = True,
):
    """Draw one DUK leg pose with the rigid B-C-O carrier made explicit."""
    plt = _plt()
    if ax is None:
        _, ax = plt.subplots()

    # AB, CD and DA are ordinary four-bar links. BC belongs to the rigid
    # B-C-O wheel-carrier body and is drawn as an edge of that triangle below.
    ax.plot(
        (pose.a[0], pose.b[0]),
        (pose.a[1], pose.b[1]),
        "o-",
    )
    ax.plot(
        (pose.c[0], pose.d[0], pose.a[0]),
        (pose.c[1], pose.d[1], pose.a[1]),
        "o-",
    )

    if show_rigid_carrier:
        from matplotlib.patches import Polygon

        carrier = Polygon(
            pose.bco_triangle,
            closed=True,
            fill=True,
            alpha=0.12,
        )
        ax.add_patch(carrier)
        carrier_chain = (*pose.bco_triangle, pose.b)
        xs, ys = zip(*carrier_chain)
        ax.plot(xs, ys, "o-", linewidth=2.0)
    else:
        ax.plot(
            (pose.b[0], pose.c[0]),
            (pose.b[1], pose.c[1]),
            "o-",
        )

    if show_auxiliary:
        ax.plot((pose.h[0], pose.b[0]), (pose.h[1], pose.b[1]), "--")
        ax.plot((pose.h[0], pose.o[0]), (pose.h[1], pose.o[1]), "--")

    if show_wheel:
        from matplotlib.patches import Circle

        ax.add_patch(Circle(pose.o, geometry.wheel_radius_mm, fill=False))
        ax.plot(*pose.o, "o")

    if annotate:
        for name in ("A", "B", "C", "D", "H", "O"):
            point = pose.point(name)
            ax.annotate(name, point, xytext=(5, 5), textcoords="offset points")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.grid(True)
    return ax


def draw_point_trajectories(
    poses: Sequence[LegPose],
    point_names: Sequence[str] = ("B", "C", "H", "O"),
    ax=None,
):
    """Plot 2-D trajectories of selected solved linkage points."""
    if not poses:
        raise ValueError("poses must not be empty")
    plt = _plt()
    if ax is None:
        _, ax = plt.subplots()
    for point_name in point_names:
        points = [pose.point(point_name) for pose in poses]
        xs, ys = zip(*points)
        ax.plot(xs, ys, label=point_name)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.grid(True)
    ax.legend()
    return ax


def plot_point_coordinates(
    poses: Sequence[LegPose],
    point_name: str = "O",
    ax=None,
):
    """Plot point coordinates against the linkage input angle."""
    if not poses:
        raise ValueError("poses must not be empty")
    plt = _plt()
    if ax is None:
        _, ax = plt.subplots()
    angles = [math.degrees(pose.linkage_angle_rad) for pose in poses]
    points = [pose.point(point_name) for pose in poses]
    ax.plot(angles, [point[0] for point in points], label=f"{point_name}.x")
    ax.plot(angles, [point[1] for point in points], label=f"{point_name}.y")
    ax.set_xlabel("linkage angle q = angle(AB→DA) (deg)")
    ax.set_ylabel("coordinate (mm)")
    ax.grid(True)
    ax.legend()
    return ax


def plot_point_jacobian(
    geometry: LegGeometry,
    poses: Sequence[LegPose],
    point_name: str = "O",
    ax=None,
):
    """Plot analytic point Jacobian components in canonical robot coordinates."""
    if not poses:
        raise ValueError("poses must not be empty")
    plt = _plt()
    if ax is None:
        _, ax = plt.subplots()
    angles = [math.degrees(pose.linkage_angle_rad) for pose in poses]
    derivatives = [
        leg_jacobian(geometry, pose).point(point_name)
        for pose in poses
    ]
    ax.plot(
        angles,
        [value[0] for value in derivatives],
        label=f"d{point_name}.x/dq",
    )
    ax.plot(
        angles,
        [value[1] for value in derivatives],
        label=f"d{point_name}.y/dq",
    )
    ax.set_xlabel("linkage angle q (deg)")
    ax.set_ylabel("Jacobian component (mm/rad)")
    ax.grid(True)
    ax.legend()
    return ax
