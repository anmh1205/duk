"""Tests for the real-time DUK leg animation layer."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from duk_core.modeling import load_leg_geometry_json

CONFIG_PATH = (
    Path(__file__).resolve().parents[3]
    / "config"
    / "physical"
    / "duk_leg_geometry.json"
)


def test_animation_builds_interactive_html_when_matplotlib_is_available() -> None:
    """Exercise the same JSHTML path used by the motion notebook."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from duk_core.visualization import (
        LegAnimationOptions,
        create_leg_animation,
    )

    geometry = load_leg_geometry_json(CONFIG_PATH)
    q_values = tuple(
        math.radians(value)
        for value in (75.0, 80.0, 85.0)
    )

    result = create_leg_animation(
        geometry,
        q_values,
        options=LegAnimationOptions(
            interval_ms=50,
            repeat=False,
            show_auxiliary=True,
            show_labels=True,
            show_wheel_trace=False,
        ),
    )

    try:
        assert len(result.poses) == len(q_values)
        for local, world in zip(result.poses, result.display_poses):
            assert local.a == (0, 0)
            assert world.o[1] - geometry.wheel_radius_mm == pytest.approx(0)
        assert [
            pose.linkage_angle_rad
            for pose in result.poses
        ] == pytest.approx(q_values)

        html = result.to_jshtml()
        assert "<script" in html
        assert "Animation" in html or "animation" in html
        # JSHTML renders the actual artists, not just the stored coordinates.
        wheel = next(
            patch
            for patch in result.figure.axes[0].patches
            if isinstance(patch, Circle)
        )
        assert wheel.center[1] - wheel.radius == pytest.approx(0)
    finally:
        plt.close(result.figure)
