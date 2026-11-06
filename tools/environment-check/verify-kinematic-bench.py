#!/usr/bin/env python3
"""Exercise only the virtual bench: manual/invalid/auto/pause and actual ROS TF."""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def transform_point(transform, point):
    q = transform.transform.rotation
    v = (q.x, q.y, q.z)

    def cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    t = tuple(2 * x for x in cross(v, point))
    second = cross(v, t)
    origin = transform.transform.translation
    return tuple(
        p + q.w * dt + c + offset
        for p, dt, c, offset in zip(point, t, second, (origin.x, origin.y, origin.z))
    )


def main():
    rclpy.init()
    node = rclpy.create_node("duk_kinematic_verification")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data = {}

    def receive(message):
        data.clear()
        data["stamp"] = message.header.stamp
        for status in message.status:
            data[status.name.split("/")[-1]] = {
                v.key: json.loads(v.value) for v in status.values
            }

    subscription = node.create_subscription(
        DiagnosticArray, "/duk/kinematics/diagnostics", receive, 10
    )
    client = node.create_client(
        SetParametersAtomically, "/duk/kinematics/bench/set_parameters_atomically"
    )

    def wait_for(predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError("Timed out waiting for virtual bench state/TF")

    def controls(**values):
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError("Virtual bench parameter service did not respond")
        return future.result().result

    try:
        wait_for(
            lambda: client.service_is_ready() and "left" in data and "right" in data
        )
        assert controls(left_q_deg=78.0, right_q_deg=95.0, playing=False).successful
        wait_for(
            lambda: (
                abs(data["left"]["q_deg"] - 78) < 1e-8
                and abs(data["right"]["q_deg"] - 95) < 1e-8
            )
        )
        geometry = load_leg_geometry_json(
            "/workspace/robot/config/physical/duk_leg_geometry.json"
        )

        def check_tf():
            try:
                stamp = Time.from_msg(data["stamp"])
                base = buffer.lookup_transform("world", "base_link", stamp)
                height = base.transform.translation.z
                for side, y in (("left", 0.145), ("right", -0.145)):
                    pose = solve_leg_pose(geometry, math.radians(data[side]["q_deg"]))
                    carrier = buffer.lookup_transform(
                        "world", side + "_carrier_frame", stamp
                    )
                    cd = buffer.lookup_transform("world", side + "_cd_frame", stamp)
                    wheel = buffer.lookup_transform(
                        "world", side + "_wheel_frame", stamp
                    )
                    c_carrier = transform_point(carrier, (geometry.bc_mm / 1000, 0, 0))
                    c_cd = transform_point(cd, (geometry.cd_mm / 1000, 0, 0))
                    expected_c = (pose.c[0] / 1000, y, height + pose.c[1] / 1000)
                    expected_o = (pose.o[0] / 1000, y, height + pose.o[1] / 1000)
                    assert math.dist(c_carrier, c_cd) < 1e-9, (
                        "Rendered loop does not close at C"
                    )
                    assert math.dist(c_cd, expected_c) < 1e-9
                    assert (
                        math.dist(transform_point(wheel, (0, 0, 0)), expected_o) < 1e-9
                    )
                return True
            except TransformException:
                return False

        wait_for(check_tf)
        assert not controls(left_q_deg=0.0, right_q_deg=85.0, playing=False).successful
        wait_for(
            lambda: (
                abs(data["left"]["q_deg"] - 78) < 1e-8
                and abs(data["right"]["q_deg"] - 95) < 1e-8
            )
        )
        assert controls(playing=True, speed=1.0).successful
        wait_for(
            lambda: data["left"]["playing"] and abs(data["left"]["q_deg"] - 78) > 0.2
        )
        for side in ("left", "right"):
            assert data[side]["closure_error_mm"] < 1e-8
            assert data[side]["carrier_error_mm"] < 1e-8
            assert data[side]["carrier_area_error_mm2"] < 1e-7
            assert abs(data[side]["wheel_vz_mm_s"]) > 0
        assert controls(playing=False).successful
        wait_for(lambda: not data["left"]["playing"])
        frozen = (data["left"]["q_deg"], data["right"]["q_deg"])
        initial_time = data["left"]["profile_time_s"]
        wait_for(lambda: data["left"]["profile_time_s"] > initial_time + 0.4)
        assert frozen == (data["left"]["q_deg"], data["right"]["q_deg"])
        assert data["left"]["wheel_vz_mm_s"] == data["right"]["wheel_vz_mm_s"] == 0
        assert not controls(speed=3.0).successful
        wait_for(check_tf)
        assert controls(
            left_q_deg=75.0, right_q_deg=75.0, playing=False, speed=1.0
        ).successful
        print(
            "Kinematic bench OK: independent poses; actual TF closes at C; invalid request atomic; auto motion; pause; speed validation"
        )
    finally:
        node.destroy_subscription(subscription)
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
