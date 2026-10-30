"""Optional Matplotlib display for generic planar chain coordinates."""

from __future__ import annotations

from collections.abc import Sequence


def draw_chain(points: Sequence[tuple[float, float]], ax=None):
    """Draw connected (x, y) points; return the Matplotlib Axes.

    Import Matplotlib only on demand, to keep the core runtime lightweight.
    """
    if not points:
        raise ValueError("points must not be empty")
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Install duk-core[research] to draw plots") from exc

    if ax is None:
        _, ax = plt.subplots()
    xs, ys = zip(*points)
    ax.plot(xs, ys, "o-")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (input length unit)")
    ax.set_ylabel("y (input length unit)")
    ax.grid(True)
    return ax
