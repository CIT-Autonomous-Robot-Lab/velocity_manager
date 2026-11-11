#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('velocity_manager')
    default_config = os.path.join(pkg_share, 'config', 'velocity_manager.yaml')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Full path to the velocity_manager parameters YAML',
    )

    velocity_manager_node = Node(
        package='velocity_manager',
        executable='velocity_manager_node',
        name='velocity_manager',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        declare_config_file,
        velocity_manager_node,
    ])
