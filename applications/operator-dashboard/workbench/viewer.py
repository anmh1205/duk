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
from .records import RunRecord, decimal

CONFIG = ROOT / "robot/software/ros2_ws/src/duk_description/config/workbench.rviz"


def replay_frame(run: RunRecord, index: int) -> ReplayFrame | None:
    def value(name):
        signal = run.signals.get(name)
        return signal.values[index] if signal else None

    if value("position") is None or value("state_joints") is None:
        return None
    wheels = []
    for side in ("left", "right"):
        normal = value(side + "_normal_force")
        tangent = value(side + "_tangential_force")
        force = None
        if normal is not None and tangent is not None:
            force = [
                *(tangent if isinstance(tangent, list) else [tangent, 0.0]),
                normal,
            ]
        mode = value(side + "_contact_mode")
        wheel_effort = value(side + "_wheel_applied")
        hip_effort = value(side + "_hip_applied")
        wheel_title, hip_title = "τ bánh", "τ hông"
        if wheel_effort is None:
            wheel_effort = value(side + "_wheel_preceding_effort")
            if wheel_effort is not None:
                wheel_title += " bước trước"
        if hip_effort is None:
            hip_effort = value(side + "_hip_preceding_effort")
            if hip_effort is not None:
                hip_title += " bước trước"
            else:
                hip_effort = value(side + "_hip_static_effort")
        label = "Trái" if side == "left" else "Phải"
        wheels.append(
            ReplayWheel(
                value(side + "_wheel_center"),
                value(side + "_contact_point"),
                force,
                f"{label} · {mode or '—'} · {decimal(normal, 2)} N\n{wheel_title} {decimal(wheel_effort, 3)} N·m",
                f"{hip_title} {decimal(hip_effort, 3)} N·m",
            )
        )
    time = run.time_s[index]
    return ReplayFrame(
        time,
        value("position"),
        value("orientation_xyzw"),
        value("state_joints"),
        (wheels[0], wheels[1]),
        value("center_of_mass"),
        f"{run.study} · {decimal(time)} s",
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
        except (OSError, RuntimeError):
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
        if self.node is not None:
            self.node.destroy_node()
        if self.context_owner and rclpy.ok():
            rclpy.shutdown()
