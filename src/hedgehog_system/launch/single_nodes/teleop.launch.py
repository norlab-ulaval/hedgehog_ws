from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    joy_teleop_config = os.path.join(get_package_share_directory("hedgehog_system"), "config", "teleop.yaml")

    joy_teleop_la = DeclareLaunchArgument(
        "joy_teleop_config",
        default_value=joy_teleop_config,
        description="joy_teleop node launch arguments",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[LaunchConfiguration("joy_teleop_config")],
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop_node",
        parameters=[LaunchConfiguration("joy_teleop_config")],
        remappings=[
            ("/joy", "/modified_joy"),
        ],
    )

    joy_axis_merging_node = Node(
        package="teleop_axis_merging", executable="teleop_axis_merging_node", name="teleop_axis_merging_node"
    )

    return LaunchDescription([joy_teleop_la, joy_node, joy_teleop_node, joy_axis_merging_node])