import os
import sys
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    base_path = get_package_share_directory("hedgehog_system")
    launch_folder = os.path.join(base_path, "launch", "single_nodes")

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

    # Mapper
    mapper_config = os.path.join(base_path, "config", "online_mapper.yaml")
    mapper_node = Node(
        package="norlab_icp_mapper_ros",
        executable="mapper_node",
        name="mapper_node",
        parameters=[
            {
                "odom_frame": "odom",
                "robot_frame": "base_link",
                "mapping_config": mapper_config,
                "initial_map_file_name": "/home/hedgehog/map_couloir.vtk",
                "initial_robot_pose": "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]",
                "final_map_file_name": "map.vtk",
                "final_trajectory_file_name": "trajectory.vtk",
                "map_publish_rate": 10.0,
                "map_tf_publish_rate": 10.0,
                "max_idle_time": 10.0,
                "is_mapping": False,
                "is_online": True,
                "is_3D": True,
                "save_map_cells_on_hard_drive": True,
                "publish_tfs_between_registrations": True,
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("points_in", "rslidar_points"),
        ],
    )

    return LaunchDescription(
        [
            vesc_la,
            vesc_to_odom_node,
            mapper_node,
        ]
    )
