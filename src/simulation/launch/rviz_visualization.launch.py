#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    # Get the path to the rviz config file
    try:
        rviz_config_path = os.path.join(
            get_package_share_directory('simulation'),
            'rviz',
            'drone_visualization.rviz'
        )
    except:
        # Fallback to relative path during development
        rviz_config_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'rviz',
            'drone_visualization.rviz'
        )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to RViz config file'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node
    ])