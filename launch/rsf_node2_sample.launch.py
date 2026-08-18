"""Starts the RSF-X001 driver node together with a preconfigured RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('rsf_node2'), 'config')
    config_yaml = os.path.join(config_dir, 'rsf_node2.yaml')
    rviz_config = os.path.join(config_dir, 'rviz.rviz')

    return LaunchDescription([
        Node(
            package='rsf_node2',
            executable='rsf_node',
            name='rsf_node',
            parameters=[config_yaml],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
