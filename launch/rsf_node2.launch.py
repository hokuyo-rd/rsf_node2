"""Starts the RSF-X001 driver node with the parameters from config/."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_yaml = os.path.join(
        get_package_share_directory('rsf_node2'), 'config', 'rsf_node2.yaml')

    return LaunchDescription([
        Node(
            package='rsf_node2',
            executable='rsf_node',
            name='rsf_node',
            parameters=[config_yaml],
            output='screen',
        ),
    ])
