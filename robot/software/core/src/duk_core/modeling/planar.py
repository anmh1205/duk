"""Generic planar serial-chain geometry.

This is a teaching/visualization example, not a validated DUK leg model.
Angles are relative joint angles in radians. Each link length is in the same
length unit as the output coordinates (meters recommended).
"""

from __future__ import annotations

import math
from collections.abc import Sequence


def forward_chain(
    lengths: Sequence[float], relative_angles: Sequence[float]
) -> tuple[tuple[float, float], ...]:
    """Return the origin followed by planar joint/end-effector coordinates.

    All angles are relative to the previous segment, measured counter-clockwise.
    For N links, the return value contains N+1 points, including (0, 0).

    Raises:
        ValueError: If lengths and angles disagree, are empty, or are nonfinite,
            or if a link length is not strictly positive.
    """
    if len(lengths) == 0 or len(lengths) != len(relative_angles):
        raise ValueError("lengths and relative_angles must be equally sized and non-empty")
    coords: list[tuple[float, float]] = [(0.0, 0.0)]
    x = y = theta = 0.0
    for length, angle in zip(lengths, relative_angles):
        if not math.isfinite(length) or length <= 0:
            raise ValueError("each link length must be finite and positive")
        if not math.isfinite(angle):
            raise ValueError("each joint angle must be finite")
        theta += angle
        x += length * math.cos(theta)
        y += length * math.sin(theta)
        coords.append((x, y))
    return tuple(coords)
