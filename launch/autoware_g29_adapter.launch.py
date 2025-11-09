from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('autoware_g29_adapter')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'autoware_g29_adapter.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    # Create the adapter node
    adapter_node = Node(
        package='autoware_g29_adapter',
        executable='autoware_g29_adapter_node',
        name='autoware_g29_adapter',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add remappings here if needed
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        adapter_node
    ])