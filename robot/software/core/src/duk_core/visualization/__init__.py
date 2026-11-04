"""Optional plotting helpers for DUK model outputs.

Visualization depends on the modeling layer; model equations must not be copied
into plotting code.
"""

from .leg_linkage import (
    draw_leg_linkage,
    draw_point_trajectories,
    plot_point_coordinates,
    plot_point_jacobian,
)

__all__ = [
    "draw_leg_linkage",
    "draw_point_trajectories",
    "plot_point_coordinates",
    "plot_point_jacobian",
]
