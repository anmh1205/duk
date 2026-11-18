"""Publish read-only simulation replay poses and diagnostics to RViz."""

from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from duk_core.modeling.leg_linkage import LegGeometry, LinkageError

from .kinematic_description import joint_positions

NAMESPACE = "/duk/workbench"
FRAME_PREFIX = "duk_workbench/"


@dataclass(frozen=True)
class ReplayWheel:
    center: list[float] | None
    contact: list[float] | None
    force: list[float] | None
    detail: str
    hip_detail: str


@dataclass(frozen=True)
class ReplayFrame:
    time_s: float
    position: list[float]
    orientation_xyzw: list[float]
    joints: list[float]
    wheels: tuple[ReplayWheel, ReplayWheel]
    center_of_mass: list[float] | None
    detail: str


def _point(values) -> Point:
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class ReplayNode(Node):
    def __init__(self, geometry: LegGeometry) -> None:
        super().__init__("replay", namespace=NAMESPACE)
        self.geometry = geometry
        self.transforms = TransformBroadcaster(self)
        self.joints = self.create_publisher(JointState, "joint_states", 10)
        self.time = self.create_publisher(Float64, "sample_time", 10)
        self.markers = self.create_publisher(MarkerArray, "markers", 10)

    def send(self, frame: ReplayFrame | None) -> bool:
        stamp = self.get_clock().now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]
        if frame is None:
            self.markers.publish(MarkerArray(markers=markers))
            return False

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = FRAME_PREFIX + "base_link"
        translation, rotation = (
            transform.transform.translation,
            transform.transform.rotation,
        )
        translation.x, translation.y, translation.z = map(float, frame.position)
        rotation.x, rotation.y, rotation.z, rotation.w = map(
            float, frame.orientation_xyzw
        )

        def label(key, position, message, coordinate="world"):
            markers.append(
                self.marker(
                    stamp,
                    key,
                    Marker.TEXT_VIEW_FACING,
                    coordinate,
                    position,
                    text=message,
                    scale=(0.0, 0.0, 0.022),
                )
            )

        # A terminal body pose can be valid even when the cut linkage tree
        # cannot be reconstructed outside its geometry domain.
        complete = True
        try:
            angles = joint_positions(
                self.geometry, *frame.joints[:2], wheel_spin_relative=frame.joints[2:]
            )
        except LinkageError:
            complete = False
        else:
            joint = JointState()
            joint.header.stamp = stamp
            joint.name, joint.position = list(angles), list(angles.values())
            self.transforms.sendTransform(transform)
            self.joints.publish(joint)
        self.time.publish(Float64(data=frame.time_s))
        label(
            1,
            [frame.position[0], frame.position[1], frame.position[2] + 0.17],
            frame.detail,
        )
        if frame.center_of_mass is not None:
            markers.append(
                self.marker(
                    stamp,
                    2,
                    Marker.SPHERE,
                    "world",
                    frame.center_of_mass,
                    scale=(0.023,) * 3,
                )
            )
        if complete:
            for index, wheel in enumerate(frame.wheels):
                side = ("left", "right")[index]
                label(
                    3 + index,
                    [0.0, 0.0, 0.045],
                    wheel.hip_detail,
                    FRAME_PREFIX + side + "_ab_frame",
                )
                if wheel.center is not None:
                    x, y, z = wheel.center
                    label(5 + index, [x, y, z + 0.10], wheel.detail)
                if (
                    wheel.contact is not None
                    and wheel.force is not None
                    and any(wheel.force)
                ):
                    arrow = self.marker(
                        stamp,
                        7 + index,
                        Marker.ARROW,
                        "world",
                        [0, 0, 0],
                        scale=(0.007, 0.018, 0.025),
                    )
                    arrow.points = [
                        _point(wheel.contact),
                        _point(
                            [x + 0.004 * f for x, f in zip(wheel.contact, wheel.force)]
                        ),
                    ]
                    markers.append(arrow)
        self.markers.publish(MarkerArray(markers=markers))
        return complete

    @staticmethod
    def marker(
        stamp: Any,
        key: int,
        kind: int,
        frame: str,
        position: list,
        *,
        scale: tuple,
        text: str = "",
    ) -> Marker:
        marker = Marker(id=key, ns="simulation", type=kind, action=Marker.ADD)
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.pose.orientation.w = 1.0
        marker.pose.position = _point(position)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.1,
            0.32,
            0.48,
            1.0,
        )
        marker.text = text
        return marker
