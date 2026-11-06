"""Ground-contact RViz preview. The loop is enforced by FK, not by robot dynamics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from duk_description.kinematic_description import build_kinematic_description
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    geometry = LaunchConfiguration("geometry").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    urdf, _ = build_kinematic_description(geometry)
    share = Path(get_package_share_directory("duk_description"))
    bench = Node(
        package="duk_description",
        executable="kinematic_bench",
        parameters=[{"geometry": geometry}],
        arguments=[] if gui else ["--headless"],
        output="screen",
    )
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="duk/kinematics",
            parameters=[
                {
                    "robot_description": ParameterValue(urdf, value_type=str),
                    "use_sim_time": False,
                    "publish_frequency": 20.0,
                }
            ],
            output="screen",
        ),
        bench,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bench, on_exit=[Shutdown(reason="Kinematic bench closed")]
            )
        ),
    ]
    if gui:
        viewer = Node(
            package="rviz2",
            executable="rviz2",
            name="kinematic_rviz",
            arguments=["-d", str(share / "config/kinematics.rviz")],
            output="screen",
        )
        actions.extend(
            [
                viewer,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=viewer, on_exit=[Shutdown(reason="RViz closed")]
                    )
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry",
                default_value="/workspace/robot/config/physical/duk_leg_geometry.json",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            OpaqueFunction(function=start),
        ]
    )
