"""Generate matching URDF/SDF snapshots without duplicating leg equations.

Planar +x maps to ROS +x; planar +y maps to ROS +z. The two leg planes
are translated along ROS y, never mirrored or rotated after solving.
Only physical leg dimensions are authoritative. Body, link thicknesses,
wheel width and colors are illustrative. No mass/inertia is invented.
"""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_core.modeling.ground_contact import place_leg_on_ground

LEG_PLANE_SPACING_M = 0.29
WHEEL_WIDTH_M = 0.036


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def build_preview(geometry_path, q_deg=75.0):
    geometry = load_leg_geometry_json(geometry_path)
    pose = solve_leg_pose(geometry, math.radians(q_deg))
    mount_height = place_leg_on_ground(geometry, pose).body_height_mm / 1000
    robot = ET.Element("robot", name="duk_preview")
    ET.SubElement(robot, "link", name="world")
    ET.SubElement(robot, "link", name="base_link")
    joint = ET.SubElement(robot, "joint", name="preview_mount", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="base_link")
    ET.SubElement(joint, "origin", xyz=numbers((0, 0, mount_height)), rpy="0 0 0")
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name="duk_preview")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = numbers((0, 0, mount_height, 0, 0, 0))
    sdf_link = ET.SubElement(model, "link", name="base_link")

    def primitive(name, shape, dimensions, xyz, rpy, color):
        # RViz assigns one material per link. Separate welded preview parts keep
        # their colors while remaining a static snapshot of the same mechanism.
        part = ET.SubElement(robot, "link", name=name)
        weld = ET.SubElement(robot, "joint", name=name + "_weld", type="fixed")
        ET.SubElement(weld, "parent", link="base_link")
        ET.SubElement(weld, "child", link=name)
        visual = ET.SubElement(part, "visual", name=name)
        ET.SubElement(visual, "origin", xyz=numbers(xyz), rpy=numbers(rpy))
        urdf_geometry = ET.SubElement(visual, "geometry")
        node = ET.SubElement(urdf_geometry, shape)
        if shape == "box":
            node.set("size", numbers(dimensions))
        elif shape == "cylinder":
            node.set("radius", str(dimensions[0]))
            node.set("length", str(dimensions[1]))
        material = ET.SubElement(visual, "material", name=name + "_material")
        ET.SubElement(material, "color", rgba=numbers(color))
        for kind in ("visual", "collision"):
            item = ET.SubElement(sdf_link, kind, name=name)
            ET.SubElement(item, "pose").text = numbers((*xyz, *rpy))
            sg = ET.SubElement(item, "geometry")
            sn = ET.SubElement(sg, shape)
            if shape == "box":
                ET.SubElement(sn, "size").text = numbers(dimensions)
            else:
                ET.SubElement(sn, "radius").text = str(dimensions[0])
                ET.SubElement(sn, "length").text = str(dimensions[1])
            if kind == "visual":
                mat = ET.SubElement(item, "material")
                ET.SubElement(mat, "ambient").text = numbers(color)
                ET.SubElement(mat, "diffuse").text = numbers(color)
                ET.SubElement(item, "cast_shadows").text = "false"

    primitive(
        "illustrative_body",
        "box",
        (0.20, 0.24, 0.14),
        (-0.035, 0, 0.035),
        (0, 0, 0),
        (0.10, 0.16, 0.24, 1),
    )
    primitive(
        "body_top",
        "box",
        (0.17, 0.21, 0.006),
        (-0.035, 0, 0.108),
        (0, 0, 0),
        (0.95, 0.52, 0.12, 1),
    )
    # A front panel indicates body orientation without external textures/meshes.
    primitive(
        "front_panel",
        "box",
        (0.004, 0.15, 0.055),
        (0.067, 0, 0.045),
        (0, 0, 0),
        (0.12, 0.75, 0.85, 1),
    )
    leg_points = {}
    for side, y in (
        ("left", LEG_PLANE_SPACING_M / 2),
        ("right", -LEG_PLANE_SPACING_M / 2),
    ):
        points = {
            name: (pose.point(name)[0] / 1000, y, pose.point(name)[1] / 1000)
            for name in ("A", "B", "C", "D", "H", "O")
        }
        leg_points[side] = {
            name: [p[0], p[1], p[2] + mount_height] for name, p in points.items()
        }

        def bar(name, start, end, radius, color):
            a, b = points[start], points[end]
            delta = tuple(v - u for u, v in zip(a, b))
            length = math.sqrt(sum(v * v for v in delta))
            center = tuple((u + v) / 2 for u, v in zip(a, b))
            # Cylinder's local z follows the solved x/z segment.
            primitive(
                side + "_" + name,
                "cylinder",
                (radius, length),
                center,
                (0, math.atan2(delta[0], delta[2]), 0),
                color,
            )

        bar("AB", "A", "B", 0.006, (0.95, 0.43, 0.10, 1))
        bar("CD", "C", "D", 0.006, (0.95, 0.43, 0.10, 1))
        bar("AD_mount", "A", "D", 0.008, (0.50, 0.55, 0.60, 1))
        for start, end in (("B", "C"), ("B", "O"), ("C", "O")):
            bar("carrier_" + start + end, start, end, 0.005, (0.08, 0.65, 0.78, 1))
        for name in ("A", "B", "C", "D"):
            primitive(
                side + "_pin_" + name,
                "cylinder",
                (0.009, 0.025),
                points[name],
                (math.pi / 2, 0, 0),
                (0.72, 0.76, 0.80, 1),
            )
        primitive(
            side + "_wheel",
            "cylinder",
            (geometry.wheel_radius_mm / 1000, WHEEL_WIDTH_M),
            points["O"],
            (math.pi / 2, 0, 0),
            (0.045, 0.05, 0.06, 1),
        )
        primitive(
            side + "_hub",
            "cylinder",
            (0.025, 0.039),
            points["O"],
            (math.pi / 2, 0, 0),
            (0.95, 0.52, 0.12, 1),
        )

    metadata = {
        "purpose": "Static geometry/rendering smoke test; not balance, gait or contact dynamics validation",
        "geometry_source": str(Path(geometry_path).resolve()),
        "q_deg": q_deg,
        "body_size_m": [0.20, 0.24, 0.14],
        "leg_plane_spacing_m": LEG_PLANE_SPACING_M,
        "wheel_width_m": WHEEL_WIDTH_M,
        "points_world_m": leg_points,
        "wheel_radius_m": geometry.wheel_radius_mm / 1000,
        "mount_height_m": mount_height,
    }
    return ET.tostring(robot, encoding="unicode"), sdf, metadata


def write_preview(geometry_path, output_dir, q_deg=75.0):
    urdf, sdf, metadata = build_preview(geometry_path, q_deg)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "robot.urdf").write_text(urdf + "\n")
    ET.indent(sdf)
    ET.ElementTree(sdf).write(
        output_dir / "robot.sdf", encoding="unicode", xml_declaration=True
    )
    (output_dir / "geometry.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return urdf, sdf, metadata
