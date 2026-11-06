"""ROS adapter for the ground-contact kinematic preview, with an optional Qt panel."""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from duk_core.modeling import load_leg_geometry_json
from duk_core.modeling.kinematic_bench import KinematicBench
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .kinematic_description import joint_positions, preview_grounding


class KinematicNode(Node):
    def __init__(self):
        super().__init__("bench", namespace="duk/kinematics")
        path = self.declare_parameter(
            "geometry", "/workspace/robot/config/physical/duk_leg_geometry.json"
        ).value
        self.geometry = load_leg_geometry_json(path)
        self.bench = KinematicBench(self.geometry)
        self.base_tf = TransformBroadcaster(self)
        self.declare_parameter("left_q_deg", 75.0)
        self.declare_parameter("right_q_deg", 75.0)
        self.declare_parameter("playing", False)
        self.declare_parameter("speed", 1.0)
        self.joints = self.create_publisher(JointState, "joint_states", 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.last_time = time.monotonic()
        self.add_on_set_parameters_callback(self.set_controls)
        self.create_timer(0.05, self.tick)

    @property
    def ground(self):
        return preview_grounding(self.geometry, *(s.pose for s in self.bench.samples))

    def set_controls(self, parameters):
        values = {p.name: p.value for p in parameters}
        allowed = {"left_q_deg", "right_q_deg", "playing", "speed"}
        if set(values) - allowed:
            return SetParametersResult(
                successful=False, reason="Only bench controls are mutable"
            )
        try:
            # Validate the entire request before changing any state.
            angles = tuple(
                math.radians(values.get(name, math.degrees(s.pose.linkage_angle_rad)))
                for name, s in zip(("left_q_deg", "right_q_deg"), self.bench.samples)
            )
            for q in angles:
                self.bench.validate_angle(q)
            speed = values.get("speed", self.bench.speed)
            if (
                isinstance(speed, bool)
                or not isinstance(speed, (int, float))
                or not math.isfinite(speed)
                or not 0.25 <= speed <= 2
            ):
                raise ValueError("Speed must be a finite number in [0.25, 2]")
            if "playing" in values and not isinstance(values["playing"], bool):
                raise ValueError("Playing must be true or false")
            if (
                self.bench.playing
                and ("left_q_deg" in values or "right_q_deg" in values)
                and values.get("playing", True)
            ):
                raise ValueError("Pause before changing manual angles")
            if "left_q_deg" in values or "right_q_deg" in values:
                self.bench.set_angles(*angles)
            self.bench.set_speed(speed)
            if "playing" in values:
                self.bench.set_playing(values["playing"])
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def tick(self):
        now = time.monotonic()
        self.bench.advance(now - self.last_time)
        self.last_time = now
        positions = joint_positions(
            self.geometry, *(s.pose.linkage_angle_rad for s in self.bench.samples)
        )
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        ground = self.ground
        base = TransformStamped()
        base.header.stamp = message.header.stamp
        base.header.frame_id = "world"
        base.child_frame_id = "base_link"
        base.transform.translation.z = ground.body_height_mm / 1000
        base.transform.rotation.x = math.sin(ground.roll_rad / 2)
        base.transform.rotation.w = math.cos(ground.roll_rad / 2)
        self.base_tf.sendTransform(base)
        message.name = list(positions)
        message.position = list(positions.values())
        self.joints.publish(message)
        diagnostics = DiagnosticArray()
        diagnostics.header = message.header
        for side, sample in zip(("left", "right"), self.bench.samples):
            world_o = ground.leg_point(sample.pose, side, "O")
            status = DiagnosticStatus(
                name="kinematics/" + side, hardware_id="virtual_bench"
            )
            status.level = (
                DiagnosticStatus.WARN
                if sample.boundary_margin_rad < 2 * self.bench.reserve_rad
                else DiagnosticStatus.OK
            )
            status.message = (
                "Near geometric boundary"
                if status.level
                else "Kinematics OK; no dynamics"
            )
            signals = {
                "q_deg": math.degrees(sample.pose.linkage_angle_rad),
                "q_dot_rad_s": sample.q_dot_rad_s,
                "q_ddot_rad_s2": sample.q_ddot_rad_s2,
                "wheel_x_mm": sample.pose.o[0],
                "wheel_z_local_mm": sample.pose.o[1],
                "wheel_z_world_mm": world_o[2],
                "ground_gap_mm": ground.ground_gap_mm(sample.pose, side),
                "body_height_mm": ground.body_height_mm,
                "body_roll_deg": math.degrees(ground.roll_rad),
                "wheel_vx_body_mm_s": sample.wheel_velocity_mm_s[0],
                "wheel_vz_body_mm_s": sample.wheel_velocity_mm_s[1],
                "wheel_ax_body_mm_s2": sample.wheel_acceleration_mm_s2[0],
                "wheel_az_body_mm_s2": sample.wheel_acceleration_mm_s2[1],
                "closure_error_mm": sample.closure_error_mm,
                "carrier_error_mm": sample.carrier_error_mm,
                "carrier_area_error_mm2": sample.carrier_area_error_mm2,
                "boundary_margin_deg": math.degrees(sample.boundary_margin_rad),
                "loop_sine": sample.loop_sine,
                "playing": self.bench.playing,
                "profile_time_s": self.bench.wall_time_s,
            }
            status.values = [
                KeyValue(key=key, value=json.dumps(value))
                for key, value in signals.items()
            ]
            diagnostics.status.append(status)
        self.diagnostics.publish(diagnostics)


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = KinematicNode()
    try:
        if options.headless:
            rclpy.spin(node)
        else:
            from .kinematic_panel import run_panel

            run_panel(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
