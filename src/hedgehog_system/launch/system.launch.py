from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os


def generate_launch_description():
    base_path = get_package_share_directory("hedgehog_system")
    launch_folder = os.path.join(base_path, "launch", "single_nodes")

    la_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "lidar.launch.py")])
    )

    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "vesc.launch.py")])
    )

    vn100_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "vn100.launch.py")])
    )

    ackermann_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "ackermann_mux.launch.py")])
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "teleop.launch.py")]),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_folder, "description.launch.py")])
    )

    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml")
        )
    )

    return LaunchDescription(
        [
            la_sim_time,
            lidar_launch,
            vesc_launch,
            vn100_launch,
            ackermann_mux_launch,
            teleop_launch,
            description_launch,
            foxglove_launch,
        ]
    )
