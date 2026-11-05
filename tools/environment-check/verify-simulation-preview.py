#!/usr/bin/env python3
"""Read-only ROS integration smoke test; run inside the simulation container."""

import time
from xml.etree import ElementTree as ET

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
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
            for part in ("AB", "CD", "carrier_BC", "carrier_BO", "carrier_CO", "wheel")
        }
        if robot.get("name") == "duk_preview" and expected <= names:
            received["description"] = len(names)

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
                print(
                    f"Preview OK: {received['description']} visual parts; world -> base_link TF; simulation clock advances"
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
