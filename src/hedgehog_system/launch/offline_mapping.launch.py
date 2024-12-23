import os
import sys
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    bag_file = "/home/nicolaslauzon/ws/uni/bags/groundTruthNicoExp1"
    # bag_file = "/home/nicolaslauzon/ws/uni/bags/fastNicoExp1"
    # bag_file = "/home/nicolaslauzon/ws/uni/bags/map_slow_21"
    bag_command = [
        "ros2",
        "bag",
        "play",
        bag_file,
        "--clock",
        "--rate",
        "0.2",
        "--start-offset",
        "22",
    ]

    base_path = get_package_share_directory("hedgehog_system")
    launch_folder = os.path.join(base_path, "launch", "single_nodes")

    # kill_system()

    la_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    # TFs
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(launch_folder, "description.launch.py")]
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    # Foxglove Bridge
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        )
    )

    # VESC odom
    vesc_config = os.path.join(base_path, "config", "vesc.yaml")
    vesc_la = DeclareLaunchArgument(
        "vesc_config",
        default_value=vesc_config,
    )
    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        parameters=[
            LaunchConfiguration("vesc_config"),
            {"use_sim_time": True},
        ],
    )

    # IMU and wheel odom
    # imu_and_wheel_odom_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(launch_folder, "imu_and_wheel_odom.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    # )

    # IMU odom
    # imu_odom_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(launch_folder, "imu_odom.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    # )

    # # EKF
    # ekf_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(launch_folder, "ekf.launch.py")]),
    #     launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    # )

    # Mapper
    mapper_config = os.path.join(base_path, "config", "offline_mapper.yaml")
    mapper_node = Node(
        package="norlab_icp_mapper_ros",
        executable="mapper_node",
        name="mapper_node",
        parameters=[
            {
                "odom_frame": "odom",
                "robot_frame": "base_link",
                "mapping_config": mapper_config,
                "initial_map_file_name": "",
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                # 'initial_robot_pose': '[[1, 0, 0], [0, 1, 0], [0, 0, 1]]',
                "final_map_file_name": "map.vtk",
                "final_trajectory_file_name": "trajectory.vtk",
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 10.0,
                "is_mapping": True,
                "is_online": False,
                "is_3D": True,
                "save_map_cells_on_hard_drive": True,
                "publish_tfs_between_registrations": False,
                "use_sim_time": True,
            },
        ],
        remappings=[
            ("points_in", "rslidar_points"),
        ],
    )

    pcl_deskew_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(launch_folder, "deskewing.launch.py")]
        )
    )

    return LaunchDescription(
        [
            la_sim_time,
            description_launch,
            foxglove_launch,
            vesc_la,
            # pcl_deskew_launch,
            vesc_to_odom_node,
            # imu_and_wheel_odom_launch,
            # imu_odom_launch,
            # ekf_launch,
            mapper_node,
            ExecuteProcess(
                name="rosbag_play",
                cmd=bag_command,
                output="screen",
            ),
        ]
    )


def kill_system():
    exit_code_systemctl = os.system("sudo systemctl stop hedgehog.service")

    ALREADY_STOPPED = 1280
    if exit_code_systemctl != 0 and exit_code_systemctl != ALREADY_STOPPED:
        print("Failed to stop hedgehog_system. Make sure to provide sudo access.")
        sys.exit(1)

    killscreen_code = os.system("killall screen")
    ALREADY_KILLED = 256
    if killscreen_code != 0 and killscreen_code != ALREADY_KILLED:
        print("Failed to kill screen. Make sure to provide sudo access.")
        sys.exit(1)
