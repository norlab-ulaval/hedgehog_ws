from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    vesc_config = os.path.join(get_package_share_directory("hedgehog_system"), "config", "vesc.yaml")

    vesc_la = DeclareLaunchArgument(
        "vesc_config",
        default_value=vesc_config,
    )

    vesc_driver_node = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )

    ackermann_to_vesc_node = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )

    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )

    return LaunchDescription([vesc_la, ackermann_to_vesc_node, vesc_to_odom_node, vesc_driver_node])
