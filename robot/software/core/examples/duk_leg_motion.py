"""Run the DUK leg as a live Matplotlib mechanism animation.

Example
-------
python robot/software/core/examples/duk_leg_motion.py

The motion profile in this example is deliberately kinematic. It stays inside
the safe geometric workspace and exists to inspect linkage motion; it is not a
motor command, controller trajectory, or dynamic simulation.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from duk_core.modeling import load_leg_geometry_json
from duk_core.visualization import (
    LegAnimationOptions,
    create_leg_animation,
)


def repository_geometry_path() -> Path:
    """Return the physical-geometry config from this repository checkout."""
    return (
        Path(__file__).resolve().parents[3]
        / "config"
        / "physical"
        / "duk_leg_geometry.json"
    )


def smooth_demo_motion(
    lower_rad: float,
    upper_rad: float,
    *,
    duration_s: float,
    fps: int,
) -> np.ndarray:
    """Generate a smooth periodic q(t) strictly inside the given limits."""
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    if fps <= 0:
        raise ValueError("fps must be positive")
    if lower_rad >= upper_rad:
        raise ValueError("lower_rad must be smaller than upper_rad")

    frame_count = max(2, round(duration_s * fps))
    time_s = np.linspace(
        0.0,
        duration_s,
        frame_count,
        endpoint=False,
    )

    midpoint = 0.5 * (lower_rad + upper_rad)

    # Use 42% of the half-to-half span so the demo remains comfortably away
    # from singular boundaries even if the safety margin is later reduced.
    amplitude = 0.42 * (upper_rad - lower_rad)

    return midpoint + amplitude * np.sin(
        2.0 * math.pi * time_s / duration_s
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Animate DUK single-leg kinematics in real time."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=6.0,
        help="seconds per animation cycle",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="animation frames per second",
    )
    parser.add_argument(
        "--margin-deg",
        type=float,
        default=5.0,
        help="margin removed from each geometric singular boundary",
    )
    parser.add_argument(
        "--trace",
        action="store_true",
        help="show the wheel-center path behind the moving mechanism",
    )
    args = parser.parse_args()

    geometry = load_leg_geometry_json(
        repository_geometry_path()
    )
    lower, upper = (
        geometry.geometric_linkage_angle_limits_rad()
    )

    margin = math.radians(args.margin_deg)
    safe_lower = lower + margin
    safe_upper = upper - margin
    if safe_lower >= safe_upper:
        raise ValueError(
            "margin is too large for the geometric workspace"
        )

    q_motion = smooth_demo_motion(
        safe_lower,
        safe_upper,
        duration_s=args.duration,
        fps=args.fps,
    )

    result = create_leg_animation(
        geometry,
        q_motion,
        options=LegAnimationOptions(
            interval_ms=round(1000 / args.fps),
            repeat=True,
            show_auxiliary=True,
            show_labels=True,
            show_wheel_trace=args.trace,
        ),
    )

    # Keep the animation object alive while the GUI event loop is running.
    import matplotlib.pyplot as plt

    _animation = result.animation
    plt.show()


if __name__ == "__main__":
    main()
