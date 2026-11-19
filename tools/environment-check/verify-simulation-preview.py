#!/usr/bin/env python3
"""Read-only ROS integration smoke test; run inside the simulation container."""

import argparse
import math
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import rclpy
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node as GazeboNode
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_core.modeling.leg_linkage import LegGeometry

ROOT = Path(__file__).resolve().parents[2]
GEOMETRY_TOLERANCE_M = 1e-6


def pose_matrix(pose: Pose) -> np.ndarray:
    matrix = np.eye(4)
    p, q = pose.position, pose.orientation
    matrix[:3, 3] = [p.x, p.y, p.z]
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return matrix


def scene_geometry_error(
    scene: Scene, geometry: LegGeometry, q_deg: float, lateral_mounts_m: list[float]
) -> float:
    """Compare actual scene cylinder endpoints, wheel centers and radii in metres."""
    model = next(m for m in scene.model if m.name == "duk_preview")
    link = next(link for link in model.link if link.name == "base_link")
    base = pose_matrix(model.pose) @ pose_matrix(link.pose)
    visuals = {v.name: v for v in link.visual}
    pose = solve_leg_pose(geometry, math.radians(q_deg))
    radius = geometry.wheel_radius_mm / 1000
    height = radius - pose.o[1] / 1000
    errors = []
    for side, y in zip(("left", "right"), lateral_mounts_m, strict=True):
        expected = {}
        for point in "ABCDO":
            x, z = pose.point(point)
            expected[point] = np.array([x / 1000, y, height + z / 1000])
        for name, segment in (
            ("AB", "AB"),
            ("CD", "CD"),
            ("AD_mount", "AD"),
            ("carrier_BC", "BC"),
            ("carrier_BO", "BO"),
            ("carrier_CO", "CO"),
            ("wheel", None),
        ):
            visual = visuals[f"{side}_{name}"]
            cylinder = visual.geometry.cylinder
            world = base @ pose_matrix(visual.pose)
            center = world[:3, 3]
            if segment is None:
                errors.extend(
                    [
                        np.linalg.norm(center - expected["O"]),
                        abs(cylinder.radius - radius),
                    ]
                )
            else:
                half = world[:3, 2] * cylinder.length / 2
                errors.extend(
                    [
                        np.linalg.norm(center - half - expected[segment[0]]),
                        np.linalg.norm(center + half - expected[segment[1]]),
                    ]
                )
    return float(np.max(errors))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle-deg", type=float, default=75.0)
    parser.add_argument(
        "--geometry",
        type=Path,
        default=ROOT / "robot/config/physical/duk_leg_geometry.json",
    )
    args = parser.parse_args()
    geometry = load_leg_geometry_json(args.geometry)
    rclpy.init()
    node = rclpy.create_node("duk_preview_smoke_test")
    received = {"clock": [], "description": None, "tf": False}
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def clock(message):
        value = message.clock.sec + message.clock.nanosec / 1e9
        if not received["clock"] or value > received["clock"][-1]:
            received["clock"].append(value)

    def description(message):
        robot = ET.fromstring(message.data)
        names = {v.get("name") for v in robot.findall("link/visual")}
        expected = {
            f"{side}_{part}"
            for side in ("left", "right")
            for part in (
                "AB",
                "CD",
                "carrier_BC",
                "carrier_BO",
                "carrier_CO",
                "wheel",
                "pin_A",
            )
        }
        if robot.get("name") == "duk_preview" and expected <= names:
            received["description"] = len(names)
            # Leg-plane spacing belongs to the preview description. The
            # canonical planar linkage supplies the independent x/z solution.
            received["mounts_y"] = [
                float(
                    robot.find(f"link/visual[@name='{side}_pin_A']/origin")
                    .get("xyz")
                    .split()[1]
                )
                for side in ("left", "right")
            ]

    def transforms(message):
        received["tf"] = received["tf"] or any(
            t.header.frame_id == "world" and t.child_frame_id == "base_link"
            for t in message.transforms
        )

    # rclpy retains subscriptions on the node until it is destroyed.
    _subscriptions = [
        node.create_subscription(Clock, "/clock", clock, 10),
        node.create_subscription(String, "/robot_description", description, durable),
        node.create_subscription(TFMessage, "/tf_static", transforms, durable),
    ]
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (
                len(received["clock"]) >= 2
                and received["description"]
                and received["tf"]
            ):
                transport = GazeboNode()
                ok, scene = transport.request(
                    "/world/duk_preview/scene/info", Empty(), Empty, Scene, 5000
                )
                if not ok:
                    raise SystemExit("Gazebo did not return the preview scene")
                error = scene_geometry_error(
                    scene, geometry, args.angle_deg, received["mounts_y"]
                )
                if not error <= GEOMETRY_TOLERANCE_M:
                    raise SystemExit(f"Gazebo geometry differs by {error:.9f} m")
                print(
                    f"Preview OK: {received['description']} visual parts; world -> base_link TF; simulation clock advances; "
                    f"maximum Gazebo geometry error {error:.12f} m (tolerance {GEOMETRY_TOLERANCE_M:.6f} m)"
                )
                break
        else:
            raise SystemExit(
                f"Preview integration failed: clock_samples={len(received['clock'])}, robot_parts={received['description']}, tf={received['tf']}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
