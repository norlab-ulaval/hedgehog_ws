import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    share_folder = get_package_share_directory('hedgehog_system')
    config_file = os.path.join(share_folder, "config", "imu_and_wheel_odom.yaml")

    imu_and_wheel_odom_node = Node(
        package="norlab_imu_tools",
        executable="imu_and_wheel_odom_node",
        name="imu_and_wheel_odom_node",
        output="screen",
        respawn=True,
        parameters=[config_file],
        remappings=[
            ("imu_topic", "/vn100/data"),
            ("wheel_odom_topic", "/vesc_odom")
        ]
    )

    return LaunchDescription([
        imu_and_wheel_odom_node
    ])