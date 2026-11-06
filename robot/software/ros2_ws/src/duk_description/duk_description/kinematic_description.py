"""RViz spanning tree of the four-bar; passive angles come from canonical FK.

The C closure joint is deliberately cut in the URDF tree. It is closed
geometrically by coupled joint states, not by a physics constraint or controller.
"""

import math
from xml.etree import ElementTree as ET

from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_core.modeling.ground_contact import place_biped_on_ground

from .preview import LEG_PLANE_SPACING_M, WHEEL_WIDTH_M, build_preview, numbers


def preview_grounding(geometry, left_pose, right_pose):
    """One rigid body placement; preview spacing/width remain illustrative."""
    return place_biped_on_ground(
        geometry,
        left_pose,
        right_pose,
        leg_spacing_mm=LEG_PLANE_SPACING_M * 1000,
        wheel_width_mm=WHEEL_WIDTH_M * 1000,
    )


def joint_positions(geometry, left_q, right_q):
    """All four relative URDF angles per side, from two independent leg q's."""
    joints = {}
    for side, q in (("left", left_q), ("right", right_q)):
        pose = solve_leg_pose(geometry, q)
        ab = math.atan2(pose.b[1], pose.b[0])
        carrier = math.atan2(pose.c[1] - pose.b[1], pose.c[0] - pose.b[0])
        dc = math.atan2(pose.c[1] - pose.d[1], pose.c[0] - pose.d[0])
        joints.update(
            {
                side + "_ab_angle": ab,
                side + "_carrier_relative": carrier - ab,
                side + "_cd_angle": dc,
                # Absolute wheel angle is held at zero; rolling is not imposed.
                side + "_wheel_relative": -carrier,
            }
        )
    return joints


def build_kinematic_description(geometry_path):
    """Reuse preview visuals, converting their origins into moving link frames."""
    geometry = load_leg_geometry_json(geometry_path)
    reference_q = math.radians(75)
    pose = solve_leg_pose(geometry, reference_q)
    urdf, _, metadata = build_preview(geometry_path, 75)
    robot = ET.fromstring(urdf)
    robot.set("name", "duk_kinematic_bench")
    # robot_state_publisher owns the internal tree; the bench broadcasts the
    # moving world->base_link transform obtained from the two contact constraints.
    robot.remove(robot.find("joint[@name='preview_mount']"))
    robot.remove(robot.find("link[@name='world']"))
    height = preview_grounding(geometry, pose, pose).body_height_mm / 1000
    ab = math.atan2(pose.b[1], pose.b[0])
    carrier = math.atan2(pose.c[1] - pose.b[1], pose.c[0] - pose.b[0])
    dc = math.atan2(pose.c[1] - pose.d[1], pose.c[0] - pose.d[0])

    def moving_joint(name, parent, child, xyz):
        ET.SubElement(robot, "link", name=child)
        joint = ET.SubElement(robot, "joint", name=name, type="continuous")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=numbers(xyz), rpy="0 0 0")
        # CCW in (x,z) is rotation about negative ROS y.
        ET.SubElement(joint, "axis", xyz="0 -1 0")

    half_spacing = metadata["leg_plane_spacing_m"] / 2
    for side, y in (("left", half_spacing), ("right", -half_spacing)):
        ab_frame, cd_frame = side + "_ab_frame", side + "_cd_frame"
        carrier_frame, wheel_frame = side + "_carrier_frame", side + "_wheel_frame"
        moving_joint(side + "_ab_angle", "base_link", ab_frame, (0, y, 0))
        moving_joint(
            side + "_carrier_relative",
            ab_frame,
            carrier_frame,
            (geometry.ab_mm / 1000, 0, 0),
        )
        moving_joint(
            side + "_cd_angle",
            "base_link",
            cd_frame,
            (pose.d[0] / 1000, y, pose.d[1] / 1000),
        )
        sign = 1 if geometry.bco_body.o_side_of_bc == "left" else -1
        moving_joint(
            side + "_wheel_relative",
            carrier_frame,
            wheel_frame,
            (-geometry.bh_mm / 1000, 0, sign * geometry.oh_mm / 1000),
        )
        for joint in list(robot.findall("joint")):
            name = joint.get("name")
            if not name.startswith(side + "_") or not name.endswith("_weld"):
                continue
            part_name = joint.find("child").get("link")
            if part_name == side + "_AB":
                frame, anchor, angle = ab_frame, pose.a, ab
            elif part_name == side + "_CD":
                frame, anchor, angle = cd_frame, pose.d, dc
            elif "_carrier_" in part_name or part_name.endswith(("_pin_B", "_pin_C")):
                frame, anchor, angle = carrier_frame, pose.b, carrier
            elif part_name.endswith(("_wheel", "_hub")):
                frame, anchor, angle = wheel_frame, pose.o, 0
            else:
                continue
            visual = robot.find(f"link[@name='{part_name}']/visual")
            origin = visual.find("origin")
            x, py, z = map(float, origin.get("xyz").split())
            roll, pitch, yaw = map(float, origin.get("rpy").split())
            dx, dz = x - anchor[0] / 1000, z - anchor[1] / 1000
            origin.set(
                "xyz",
                numbers(
                    (
                        math.cos(angle) * dx + math.sin(angle) * dz,
                        py - y,
                        -math.sin(angle) * dx + math.cos(angle) * dz,
                    )
                ),
            )
            origin.set("rpy", numbers((roll, pitch + angle, yaw)))
            joint.find("parent").set("link", frame)
        marker_name = side + "_wheel_marker"
        marker = ET.SubElement(robot, "link", name=marker_name)
        weld = ET.SubElement(robot, "joint", name=marker_name + "_weld", type="fixed")
        ET.SubElement(weld, "parent", link=wheel_frame)
        ET.SubElement(weld, "child", link=marker_name)
        visual = ET.SubElement(marker, "visual", name=marker_name)
        radius = geometry.wheel_radius_mm / 1000
        ET.SubElement(
            visual,
            "origin",
            xyz=numbers((radius * 0.55, 0.021 if y > 0 else -0.021, 0)),
            rpy="0 0 0",
        )
        ET.SubElement(
            ET.SubElement(visual, "geometry"),
            "box",
            size=numbers((radius * 0.7, 0.002, 0.008)),
        )
        material = ET.SubElement(visual, "material", name=marker_name)
        ET.SubElement(material, "color", rgba="0.95 0.95 0.95 1")
    return ET.tostring(robot, encoding="unicode"), height
