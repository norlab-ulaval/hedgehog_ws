from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_description_urdf = os.path.join(
        get_package_share_directory('hedgehog_system'),
        'urdf',
        'hedgehog_description.urdf'
    )

    robot_description_la = DeclareLaunchArgument(
        'robot_description_urdf',
        default_value=robot_description_urdf,
        description='Path to the robot description urdf file'
    )

    with open(robot_description_urdf, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )
    
    return LaunchDescription([
        robot_description_la, 
        robot_state_publisher_node
    ])