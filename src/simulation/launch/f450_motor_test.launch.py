#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for F450 drone motor testing in SITL simulation.
    
    This launch file:
    1. Starts the full SITL simulation
    2. Waits for initialization
    3. Runs the motor test
    """
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='f450_empty',
        description='World file name for simulation'
    )
    
    # Include the main SITL launch file
    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('simulation'),  # This might need adjustment
                'launch',
                'f450_sitl.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': 'false',  # Keep GUI for motor test visualization
        }.items()
    )
    
    # Motor test node (started after SITL and MAVROS2 are ready)
    motor_test_node = TimerAction(
        period=20.0,  # 20 second delay to ensure everything is connected
        actions=[
            Node(
                package='drone_control',
                executable='motor_test_mavros2',
                name='motor_test_mavros2',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        
        # Launch processes
        sitl_launch,
        motor_test_node,
    ])