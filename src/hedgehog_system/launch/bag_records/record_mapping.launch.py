from datetime import datetime
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os


def generate_launch_description():
    topics = ["rslidar_points", "sensors/core", "commands/servo/position", "vectornav/imu"]

    filename = datetime.now().strftime("mapping-%Y-%m-%d_%H-%M-%S")
    path = f"/home/hedgehog/bags/{filename}"

    command = ["ros2", "bag", "record", "-o", path]
    command.extend(topics)

    return LaunchDescription([ExecuteProcess(name="rosbag_record", cmd=command, output="screen")])
