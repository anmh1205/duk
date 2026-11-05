"""Static two-leg preview. Uses existing geometry; makes no dynamics claims."""

from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from duk_description.preview import write_preview
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    geometry = Path(LaunchConfiguration("geometry").perform(context))
    q_deg = float(LaunchConfiguration("q_deg").perform(context))
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    share = Path(get_package_share_directory("duk_gazebo"))
    output = Path(LaunchConfiguration("output_dir").perform(context))
    urdf, sdf, _ = write_preview(geometry, output, q_deg)
    world = ET.parse(share / "worlds/vmware-preview.sdf")
    world.getroot().find("world").append(sdf.find("model"))
    ET.indent(world)
    world_path = output / "world.sdf"
    world.write(world_path, encoding="unicode", xml_declaration=True)
    command = ["gz", "sim", "-r", "-v", "3", str(world_path)]
    if gui:
        command += [
            "--render-engine",
            "ogre",
            "--gui-config",
            str(share / "config/vmware.gui.config"),
        ]
    else:
        command += ["-s"]
    gazebo = ExecuteProcess(cmd=command, output="screen")
    actions = [
        gazebo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo, on_exit=[Shutdown(reason="Gazebo exited")]
            )
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": ParameterValue(urdf, value_type=str),
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ]
    if rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "config/preview.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry",
                default_value="/workspace/robot/config/physical/duk_leg_geometry.json",
            ),
            DeclareLaunchArgument(
                "output_dir", default_value="/workspace/data/generated/vmware-preview"
            ),
            DeclareLaunchArgument("q_deg", default_value="75"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=start),
        ]
    )
