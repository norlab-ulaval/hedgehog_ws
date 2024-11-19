from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    ekf_config = os.path.join(get_package_share_directory("hedgehog_system"), "config", "ekf.yaml")

    ekf_la = DeclareLaunchArgument("ekf_config", default_value=ekf_config)

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[LaunchConfiguration("ekf_config"), LaunchConfiguration("use_sim_time")],
    )

    # bias_observer_node = Node(
    #     package="norlab_imu_tools",
    #     executable="imu_bias_observer",
    #     name="imu_bias_observer_node",
    #     parameters=[{"target_observation_samples": 2000}],
    #     remappings=[
    #         ("imu_topic_in", "/sensors/imu/raw"),
    #         ("bias_topic_out", "/sensors/imu/bias"),
    #     ],
    # )

    # bias_compensator_node = Node(
    #     package="norlab_imu_tools",
    #     executable="imu_bias_compensator_node",
    #     name="imu_bias_compensator_node",
    #     remappings=[
    #         ("imu_topic_in", "/sensors/imu/raw"),
    #         ("bias_topic_in", "/sensors/imu/bias"),
    #         ("imu_topic_out", "/sensors/imu/unbiased"),
    #     ],
    # )

    return LaunchDescription([ekf_la, ekf_node])
