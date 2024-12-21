import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('vn100_ns')
    namespace_launch_arg = DeclareLaunchArgument(
        'vn100_ns',
        default_value='vn100'
    )

    share_folder = get_package_share_directory('hedgehog_system')
    config_file = os.path.join(share_folder, "config", "vn100.yaml")

    vectornav_driver = Node(
        package="vectornav",
        executable="vectornav",
        name="driver",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("/vectornav/raw/attitude", "raw/attitude"),
            ("/vectornav/raw/common", "raw/common"),
            ("/vectornav/raw/gps", "raw/gps"),
            ("/vectornav/raw/gps2", "raw/gps2"),
            ("/vectornav/raw/imu", "raw/imu"),
            ("/vectornav/raw/ins", "raw/ins"),
            ("/vectornav/raw/time", "raw/time"),
        ],
    )

    vectornav_decoder = Node(
        package="vectornav",
        executable="vn_sensor_msgs",
        name="decoder",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("/vectornav/raw/attitude", "raw/attitude"),
            ("/vectornav/raw/common", "raw/common"),
            ("/vectornav/raw/gps", "raw/gps"),
            ("/vectornav/raw/gps2", "raw/gps2"),
            ("/vectornav/raw/imu", "raw/imu"),
            ("/vectornav/raw/ins", "raw/ins"),
            ("/vectornav/raw/time", "raw/time"),

            ("vectornav/gnss", "gnss"),
            ("vectornav/imu", "imu/original"),
            ("vectornav/imu_uncompensated", "data_raw"),
            ("vectornav/magnetic", "magnetic"),
            ("vectornav/pose", "pose"),
            ("vectornav/pressure", "pressure"),
            ("vectornav/temperature", "temperature"),
            ("vectornav/time_gps", "time_gps"),
            ("vectornav/time_pps", "time_pps"),
            ("vectornav/time_startup", "time_startup"),
            ("vectornav/time_syncin", "time_syncin"),
            ("vectornav/velocity_body", "velocity_body"),
        ],
    )

    bias_compensator_node = Node(
        package="norlab_imu_tools",
        executable="imu_bias_compensator_node",
        name="bias_compensator",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("imu_topic_in", "data_raw"),
            ("bias_topic_in", "bias"),
            ("imu_topic_out", "data_unbiased")
        ]
    )

    filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="madgwick_filter",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("imu/data_raw", "data_unbiased"),
            ("imu/mag", "mag"),
            ("imu/data", "data")
        ]
    )

    bias_observer_node = Node(
        package="norlab_imu_tools",
        executable="imu_bias_observer",
        name="bias_observer",
        namespace=namespace,
        output="screen",
        parameters=[config_file],
        remappings=[
            ("imu_topic_in", "data_raw"),
            ("bias_topic_out", "bias")
        ]
    )

    return LaunchDescription([
        namespace_launch_arg,
        vectornav_driver,
        vectornav_decoder,
        bias_compensator_node,
        filter_madgwick_node,
        bias_observer_node
    ])