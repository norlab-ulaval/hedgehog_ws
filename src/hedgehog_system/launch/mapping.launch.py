import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='norlab_icp_mapper_ros',
            executable='mapper_node',
            name='mapper_node',
            parameters=[{
                'odom_frame': 'odom',
                'robot_frame': 'base_link',
                'mapping_config': os.path.join(
                    get_package_share_directory('hedgehog_system'),
                    'config',
                    'mapper.yaml'
                ),
                'initial_map_file_name': '',
                'initial_robot_pose': '[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]',
                # 'initial_robot_pose': '[[1, 0, 0], [0, 1, 0], [0, 0, 1]]',
                'final_map_file_name': 'map.vtk',
                'final_trajectory_file_name': 'trajectory.vtk',
                'map_publish_rate': 10.0,
                'map_tf_publish_rate': 10.0,
                'max_idle_time': 10.0,
                'is_mapping': True,
                'is_online': True,
                'is_3D': True,
                'save_map_cells_on_hard_drive': True,
                'publish_tfs_between_registrations': True,
            }],
            remappings=[
                ('points_in', 'rslidar_points'),
            ]
        )
    ])