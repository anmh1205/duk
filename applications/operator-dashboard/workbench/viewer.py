"""Connect workbench playback to the geometry-derived ROS adapter."""

from __future__ import annotations

import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix

from duk_description.kinematic_description import build_kinematic_description
from duk_description.simulation_replay import (
    FRAME_PREFIX,
    NAMESPACE,
    ReplayFrame,
    ReplayNode,
    ReplayWheel,
)

from .inputs import GEOMETRY, ROOT, Inputs
from .records import RunRecord, finite_number

CONFIG = ROOT / "robot/software/ros2_ws/src/duk_description/config/workbench.rviz"


def replay_frame(run: RunRecord, index: int) -> ReplayFrame | None:
    def value(name):
        signal = run.signals.get(name)
        return signal.values[index] if signal else None

    def vector(name, length):
        data = value(name)
        if (
            isinstance(data, list)
            and len(data) == length
            and all(map(finite_number, data))
        ):
            return data
        return None

    position = vector("position", 3)
    orientation = vector("orientation_xyzw", 4)
    joints = vector("state_joints", 4)
    if position is None or orientation is None or joints is None:
        return None

    wheels = []
    for side in ("left", "right"):
        normal = value(side + "_normal_force")
        tangent = value(side + "_tangential_force")
        force = None
        if finite_number(normal):
            horizontal = (
                [tangent, 0.0]
                if finite_number(tangent)
                else vector(side + "_tangential_force", 2)
            )
            if horizontal is not None:
                force = [*horizontal, normal]
        wheels.append(
            ReplayWheel(
                vector(side + "_contact_point", 3),
                force,
            )
        )
    return ReplayFrame(
        run.time_s[index],
        position,
        orientation,
        joints,
        (wheels[0], wheels[1]),
        vector("center_of_mass", 3),
    )


class Viewer:
    def __init__(self, inputs: Inputs, *, headless: bool = False) -> None:
        body = inputs.fixture["planar"]
        mounts = [
            (body[f"a_{side}_x_m"], y, body[f"a_{side}_z_m"])
            for side, y in zip(
                ("L", "R"), inputs.fixture["spatial"]["lateral_mounts_m"]
            )
        ]
        urdf, _ = build_kinematic_description(GEOMETRY, mounts_xyz_m=mounts)
        self.context_owner = not rclpy.ok()
        self.processes: list[subprocess.Popen] = []
        self.node = None
        try:
            if self.context_owner:
                rclpy.init(args=[])
            self.node = ReplayNode(inputs.geometry)
            self.start(
                "robot_state_publisher",
                "--ros-args",
                "-r",
                f"__ns:={NAMESPACE}",
                "-p",
                f"robot_description:={urdf}",
                "-p",
                f"frame_prefix:={FRAME_PREFIX}",
                "-p",
                "ignore_timestamp:=true",
            )
            if not headless:
                self.start("rviz2", "-d", str(CONFIG))
        except (LookupError, OSError, RuntimeError):
            self.close()
            raise

    def start(self, package: str, *args: str) -> None:
        path = Path(get_package_prefix(package)) / "lib" / package / package
        self.processes.append(subprocess.Popen([str(path), *args]))

    def send(self, run: RunRecord | None, index: int) -> bool:
        if any(process.poll() is not None for process in self.processes):
            raise RuntimeError("Tiến trình RViz hoặc robot_state_publisher đã đóng.")
        rclpy.spin_once(self.node, timeout_sec=0)
        return self.node.send(replay_frame(run, index) if run else None)

    def close(self) -> None:
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.processes.clear()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context_owner:
            self.context_owner = False
            if rclpy.ok():
                rclpy.shutdown()
