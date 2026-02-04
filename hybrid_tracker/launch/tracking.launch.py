"""
Launch file for hybrid tracker system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for hybrid tracker"""
    
    # Package directory
    pkg_share = FindPackageShare('hybrid_tracker')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'tracker_params.yaml']),
        description='Path to configuration YAML file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo)'
    )
    
    # Hybrid tracker node
    hybrid_tracker_node = Node(
        package='hybrid_tracker',
        executable='hybrid_tracker_node',
        name='hybrid_tracker_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        hybrid_tracker_node,
    ])
