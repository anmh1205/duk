"""Independent URDF forward transforms check the displayed closed-loop geometry."""

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_description.kinematic_description import (
    build_kinematic_description,
    joint_positions,
    preview_grounding,
)

ROOT = next(
    p for p in Path(__file__).resolve().parents if (p / "compose.yaml").exists()
)
CONFIG = ROOT / "robot/config/physical/duk_leg_geometry.json"


def multiply(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)
    ]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0, 0, 0, 1],
    ]


def point(matrix, value):
    return tuple(
        sum(matrix[i][j] * value[j] for j in range(3)) + matrix[i][3] for i in range(3)
    )


def urdf_transforms(robot, positions, base_transform):
    result = {"base_link": base_transform}
    joints = list(robot.findall("joint"))
    while joints:
        count = len(joints)
        for joint in list(joints):
            parent, child = (
                joint.find("parent").get("link"),
                joint.find("child").get("link"),
            )
            if parent not in result:
                continue
            origin = joint.find("origin")
            xyz = (
                tuple(map(float, origin.get("xyz", "0 0 0").split()))
                if origin is not None
                else (0, 0, 0)
            )
            rpy = (
                tuple(map(float, origin.get("rpy", "0 0 0").split()))
                if origin is not None
                else (0, 0, 0)
            )
            matrix = multiply(result[parent], transform(xyz, rpy))
            if joint.get("type") == "continuous":
                assert joint.find("axis").get("xyz") == "0 -1 0"
                matrix = multiply(
                    matrix, transform(rpy=(0, -positions[joint.get("name")], 0))
                )
            result[child] = matrix
            joints.remove(joint)
        assert len(joints) < count, "URDF must form one connected tree"
    return result


@pytest.mark.parametrize("angles", [(75, 75), (78, 95), (105, 80), (120, 110)])
def test_displayed_links_wheel_and_cut_joint_close_at_each_pose(angles):
    geometry = load_leg_geometry_json(CONFIG)
    urdf, _ = build_kinematic_description(CONFIG)
    robot = ET.fromstring(urdf)
    positions = joint_positions(geometry, *(math.radians(a) for a in angles))
    poses = [solve_leg_pose(geometry, math.radians(a)) for a in angles]
    ground = preview_grounding(geometry, *poses)
    matrices = urdf_transforms(
        robot,
        positions,
        transform((0, 0, ground.body_height_mm / 1000), (ground.roll_rad, 0, 0)),
    )
    assert len(positions) == 8
    for side, pose in zip(("left", "right"), poses):
        expected = {
            name: tuple(v / 1000 for v in ground.leg_point(pose, side, name))
            for name in ("A", "B", "C", "D", "O")
        }
        assert point(matrices[side + "_ab_frame"], (0, 0, 0)) == pytest.approx(
            expected["A"], abs=1e-11
        )
        assert point(matrices[side + "_carrier_frame"], (0, 0, 0)) == pytest.approx(
            expected["B"], abs=1e-11
        )
        assert point(
            matrices[side + "_carrier_frame"], (geometry.bc_mm / 1000, 0, 0)
        ) == pytest.approx(expected["C"], abs=1e-11)
        assert point(
            matrices[side + "_cd_frame"], (geometry.cd_mm / 1000, 0, 0)
        ) == pytest.approx(expected["C"], abs=1e-11)
        assert point(matrices[side + "_wheel_frame"], (0, 0, 0)) == pytest.approx(
            expected["O"], abs=1e-11
        )
        assert ground.ground_gap_mm(pose, side) == pytest.approx(0, abs=1e-10)
        # Verify the actual visual cylinder endpoints, not just moving frame names.
        for part, endpoints in (
            ("AB", ("A", "B")),
            ("CD", ("C", "D")),
            ("carrier_BC", ("B", "C")),
            ("carrier_BO", ("B", "O")),
            ("carrier_CO", ("C", "O")),
        ):
            name = side + "_" + part
            visual = robot.find(f"link[@name='{name}']/visual")
            origin = visual.find("origin")
            matrix = multiply(
                matrices[name],
                transform(
                    tuple(map(float, origin.get("xyz").split())),
                    tuple(map(float, origin.get("rpy").split())),
                ),
            )
            length = float(visual.find("geometry/cylinder").get("length"))
            actual = [point(matrix, (0, 0, sign * length / 2)) for sign in (-1, 1)]
            for endpoint in endpoints:
                assert min(math.dist(p, expected[endpoint]) for p in actual) < 1e-10
