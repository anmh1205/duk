"""Animated visualization for DUK leg motion.

This module contains presentation logic only. It never recomputes linkage
geometry itself: every animation frame is produced by the canonical kinematic
solver.

The default animation intentionally shows one mechanism pose at a time. Motion
history/trails are opt-in so the notebook behaves like an engineering animation
rather than a stack of overlaid snapshots.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

from duk_core.modeling.leg_linkage import (
    LegGeometry,
    LegPose,
    solve_leg_pose,
)


@dataclass(slots=True)
class LegAnimation:
    """Container for an animation and the solved poses that drive it."""

    animation: object
    figure: object
    poses: tuple[LegPose, ...]

    def to_jshtml(self) -> str:
        """Render the animation as an interactive Jupyter HTML player."""
        return self.animation.to_jshtml()  # type: ignore[attr-defined]


@dataclass(frozen=True, slots=True)
class LegAnimationOptions:
    """Presentation options for a DUK leg animation."""

    interval_ms: int = 33
    repeat: bool = True
    show_auxiliary: bool = True
    show_labels: bool = True
    show_wheel_trace: bool = False
    padding_mm: float = 25.0

    def __post_init__(self) -> None:
        if self.interval_ms <= 0:
            raise ValueError("interval_ms must be positive")
        if self.padding_mm < 0.0 or not math.isfinite(self.padding_mm):
            raise ValueError("padding_mm must be finite and non-negative")


def _matplotlib():
    """Import Matplotlib lazily so modeling remains usable without plotting."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Circle, Polygon
    except ImportError as exc:
        raise RuntimeError(
            "Install duk-core[research] to use animation helpers"
        ) from exc

    return plt, FuncAnimation, Circle, Polygon


def _motion_bounds(
    poses: Sequence[LegPose],
    geometry: LegGeometry,
    padding_mm: float,
) -> tuple[float, float, float, float]:
    """Return stable axis limits that contain every animation frame."""
    x_values: list[float] = []
    y_values: list[float] = []

    for pose in poses:
        for point_name in ("a", "b", "c", "d", "h", "o"):
            x, y = getattr(pose, point_name)
            x_values.append(x)
            y_values.append(y)

        # Include the full wheel envelope, not only its center.
        x_values.extend(
            (
                pose.o[0] - geometry.wheel_radius_mm,
                pose.o[0] + geometry.wheel_radius_mm,
            )
        )
        y_values.extend(
            (
                pose.o[1] - geometry.wheel_radius_mm,
                pose.o[1] + geometry.wheel_radius_mm,
            )
        )

    return (
        min(x_values) - padding_mm,
        max(x_values) + padding_mm,
        min(y_values) - padding_mm,
        max(y_values) + padding_mm,
    )


def create_leg_animation(
    geometry: LegGeometry,
    linkage_angles_rad: Sequence[float],
    *,
    options: LegAnimationOptions | None = None,
    ax=None,
):
    """Create a real-time Matplotlib animation of the DUK leg.

    linkage_angles_rad contains ordered values of the one-DOF linkage
    coordinate q. One pose is solved for each frame.

    The returned FuncAnimation can be shown in Jupyter with
    HTML(animation.to_jshtml()), which provides play/pause and scrub controls.

    Only the canonical solver computes A/B/C/D/H/O. This function updates
    artists from solved poses and does not duplicate robot mathematics.
    """
    if len(linkage_angles_rad) < 2:
        raise ValueError(
            "linkage_angles_rad must contain at least two frames"
        )

    options = options or LegAnimationOptions()
    plt, FuncAnimation, Circle, Polygon = _matplotlib()

    # Pre-solve every frame once. Playback then only updates artists, which
    # keeps motion smooth and avoids solving repeatedly inside the GUI loop.
    poses = tuple(
        solve_leg_pose(geometry, float(q))
        for q in linkage_angles_rad
    )

    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))

    x_min, x_max, y_min, y_max = _motion_bounds(
        poses,
        geometry,
        options.padding_mm,
    )
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")

    first = poses[0]

    # AB, CD and DA are ordinary links. BC is deliberately not drawn as a
    # separate bar because B-C-O is one rigid carrier body.
    (line_ab,) = ax.plot(
        (first.a[0], first.b[0]),
        (first.a[1], first.b[1]),
        "o-",
        linewidth=2.0,
        label="AB",
    )
    (line_cd,) = ax.plot(
        (first.c[0], first.d[0]),
        (first.c[1], first.d[1]),
        "o-",
        linewidth=2.0,
        label="CD",
    )
    (line_da,) = ax.plot(
        (first.d[0], first.a[0]),
        (first.d[1], first.a[1]),
        "o-",
        linewidth=2.0,
        label="DA",
    )

    carrier = Polygon(
        first.bco_triangle,
        closed=True,
        alpha=0.18,
        label="rigid BCO carrier",
    )
    ax.add_patch(carrier)

    # Separate outline keeps B/C/O visually explicit on top of the fill.
    (carrier_outline,) = ax.plot(
        [],
        [],
        "o-",
        linewidth=2.2,
    )

    (line_bh,) = ax.plot(
        [],
        [],
        "--",
        linewidth=1.2,
        label="BH",
    )
    (line_ho,) = ax.plot(
        [],
        [],
        "--",
        linewidth=1.2,
        label="OH",
    )
    if not options.show_auxiliary:
        line_bh.set_visible(False)
        line_ho.set_visible(False)

    wheel = Circle(
        first.o,
        geometry.wheel_radius_mm,
        fill=False,
        linewidth=1.6,
    )
    ax.add_patch(wheel)

    # Motion history is optional and disabled by default.
    (wheel_trace,) = ax.plot(
        [],
        [],
        linewidth=1.0,
        alpha=0.65,
        label="O trace",
    )
    wheel_trace.set_visible(options.show_wheel_trace)

    title = ax.set_title("")

    labels = {}
    if options.show_labels:
        for point_name in ("A", "B", "C", "D", "H", "O"):
            labels[point_name] = ax.annotate(
                point_name,
                (0.0, 0.0),
                xytext=(5, 5),
                textcoords="offset points",
            )

    ax.legend(loc="best")

    trace_x: list[float] = []
    trace_y: list[float] = []

    def update(frame_index: int):
        """Move every artist to the solved pose for one animation frame."""
        pose = poses[frame_index]

        line_ab.set_data(
            (pose.a[0], pose.b[0]),
            (pose.a[1], pose.b[1]),
        )
        line_cd.set_data(
            (pose.c[0], pose.d[0]),
            (pose.c[1], pose.d[1]),
        )
        line_da.set_data(
            (pose.d[0], pose.a[0]),
            (pose.d[1], pose.a[1]),
        )

        carrier.set_xy(pose.bco_triangle)
        carrier_loop = (
            pose.b,
            pose.c,
            pose.o,
            pose.b,
        )
        carrier_outline.set_data(
            tuple(point[0] for point in carrier_loop),
            tuple(point[1] for point in carrier_loop),
        )

        line_bh.set_data(
            (pose.h[0], pose.b[0]),
            (pose.h[1], pose.b[1]),
        )
        line_ho.set_data(
            (pose.h[0], pose.o[0]),
            (pose.h[1], pose.o[1]),
        )
        wheel.center = pose.o

        if options.show_wheel_trace:
            trace_x.append(pose.o[0])
            trace_y.append(pose.o[1])
            wheel_trace.set_data(trace_x, trace_y)

        if options.show_labels:
            for point_name, annotation in labels.items():
                annotation.xy = pose.point(point_name)

        title.set_text(
            "DUK leg motion — "
            f"q={math.degrees(pose.linkage_angle_rad):.2f}°"
        )

        artists = [
            line_ab,
            line_cd,
            line_da,
            carrier,
            carrier_outline,
            line_bh,
            line_ho,
            wheel,
            wheel_trace,
            title,
        ]
        artists.extend(labels.values())
        return tuple(artists)

    animation = FuncAnimation(
        ax.figure,
        update,
        frames=len(poses),
        interval=options.interval_ms,
        repeat=options.repeat,
        blit=False,
    )

    return LegAnimation(
        animation=animation,
        figure=ax.figure,
        poses=poses,
    )
