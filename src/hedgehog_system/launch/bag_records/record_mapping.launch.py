from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
)


def generate_launch_description():
    topics = ["rslidar_points", "sensors/core", "commands/servo/position", "vn100/data"]

    filename = datetime.now().strftime("mapping-%Y-%m-%d_%H-%M-%S")
    path = f"/home/hedgehog/bags/{filename}"

    command = ["ros2", "bag", "record", "-s", "mcap", "-o", path]
    command.extend(topics)

    return LaunchDescription(
        [ExecuteProcess(name="rosbag_record", cmd=command, output="screen")]
    )
