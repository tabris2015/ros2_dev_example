# Copyright YEAR Jose Laruta
"""Launch the TOPIC node in either language with its parameter file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    lang = DeclareLaunchArgument(
        'lang',
        default_value='py',
        choices=['py', 'cpp'],
        description='Run the Python or the C++ implementation',
    )
    params = PathJoinSubstitution(
        [FindPackageShare('PACKAGE_NAME'), 'config', 'params.yaml'],
    )
    return LaunchDescription([
        lang,
        Node(
            package='PACKAGE_NAME',
            executable=['TOPIC_', LaunchConfiguration('lang')],
            name='TOPIC',
            parameters=[params],
            output='screen',
        ),
    ])
