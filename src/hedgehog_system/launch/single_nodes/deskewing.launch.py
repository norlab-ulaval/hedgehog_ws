import os, yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    share_folder = get_package_share_directory("hedgehog_system")
    config_file = os.path.join(share_folder, "config", "deskewing.yaml")

    pcl_deskew_node = Node(
        package="pointcloud_motion_deskew",
        executable="pointcloud2_deskew_node",
        name="pcl_deskew_node",
        parameters=[config_file],
        remappings=[
            ("input_point_cloud", "rslidar_points"),
            ("output_point_cloud", "rslidar_points_deskewed"),
        ],
    )

    return LaunchDescription([pcl_deskew_node])
