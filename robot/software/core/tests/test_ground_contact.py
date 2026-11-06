"""World contact without changing the canonical mechanism or moving wheels alone."""

import math
from pathlib import Path

import pytest
from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_core.modeling.ground_contact import place_biped_on_ground, place_leg_on_ground

GEOMETRY = load_leg_geometry_json(
    Path(__file__).resolve().parents[3] / "config/physical/duk_leg_geometry.json"
)


def test_single_leg_world_contact_moves_body_and_keeps_local_origin():
    heights = []
    for angle in (75, 85, 95, 110, 125):
        local = solve_leg_pose(GEOMETRY, math.radians(angle))
        world = place_leg_on_ground(GEOMETRY, local)
        assert local.a == (0, 0)
        assert world.o[1] == pytest.approx(GEOMETRY.wheel_radius_mm)
        heights.append(world.a[1])
        for a, b in (("A", "B"), ("B", "C"), ("B", "O"), ("C", "O")):
            assert math.dist(world.point(a), world.point(b)) == pytest.approx(
                math.dist(local.point(a), local.point(b))
            )
    assert max(heights) - min(heights) > 10


@pytest.mark.parametrize("left_deg", [75, 95, 125])
@pytest.mark.parametrize("right_deg", [75, 95, 125])
def test_both_finite_tires_touch_without_penetrating_or_deforming(left_deg, right_deg):
    left, right = (
        solve_leg_pose(GEOMETRY, math.radians(q)) for q in (left_deg, right_deg)
    )
    frame = place_biped_on_ground(
        GEOMETRY, left, right, leg_spacing_mm=290, wheel_width_mm=36
    )
    assert abs(frame.roll_rad) < math.pi / 2
    for side, pose in (("left", left), ("right", right)):
        assert frame.ground_gap_mm(pose, side) == pytest.approx(0, abs=1e-10)
        wheel = frame.leg_point(pose, side, "O")
        # Independent support-point construction on both cap edges/rims.
        lowest = float("inf")
        for cap in (-18, 18):
            for index in range(24):
                theta = 2 * math.pi * index / 24
                local = (
                    pose.o[0] + 75 * math.cos(theta),
                    (145 if side == "left" else -145) + cap,
                    pose.o[1] + 75 * math.sin(theta),
                )
                lowest = min(lowest, frame.transform_point(local)[2])
        assert lowest == pytest.approx(0, abs=1e-10)
        assert wheel[2] >= 0
        for a, b in (("A", "B"), ("A", "D"), ("B", "C"), ("B", "O"), ("C", "O")):
            assert math.dist(
                frame.leg_point(pose, side, a), frame.leg_point(pose, side, b)
            ) == pytest.approx(math.dist(pose.point(a), pose.point(b)), abs=1e-10)
    assert math.dist(
        frame.leg_point(left, "left", "A"), frame.leg_point(right, "right", "A")
    ) == pytest.approx(290)
    if left_deg == right_deg:
        assert frame.roll_rad == 0
        assert frame.body_height_mm == pytest.approx(75 - left.o[1])


def test_ground_placement_rejects_invalid_layout():
    pose = solve_leg_pose(GEOMETRY, math.radians(75))
    for spacing, width in ((0, 36), (290, -1), (float("nan"), 36)):
        with pytest.raises(ValueError):
            place_biped_on_ground(
                GEOMETRY, pose, pose, leg_spacing_mm=spacing, wheel_width_mm=width
            )
