from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    ackermann_mux_config = os.path.join(
        get_package_share_directory("hedgehog_system"),
        "config",
        "ackermann_mux.yaml",
    )

    ackermann_mux_la = DeclareLaunchArgument(
        "ackermann_mux_config",
        default_value=ackermann_mux_config,
        description="Ackermann mux node launch arguments",
    )

    ackermann_mux_node = Node(
        package="ackermann_mux",
        executable="ackermann_mux",
        name="ackermann_mux_node",
        parameters=[LaunchConfiguration("ackermann_mux_config")],
        remappings=[("ackermann_cmd_out", "ackermann_cmd")],
    )

    return LaunchDescription([ackermann_mux_la, ackermann_mux_node])