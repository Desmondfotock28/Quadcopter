#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for F450 drone SITL simulation with MAVROS2 integration.
    
    This launch file starts:
    1. PX4 SITL with Gazebo
    2. MAVROS2 node for ROS2 bridge
    3. Optional QGroundControl connection
    """
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='f450_empty',
        description='World file name (without .world extension)'
    )
    
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='f450_drone',
        description='Vehicle model to use'
    )
    
    x_pos_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position to spawn the vehicle'
    )
    
    y_pos_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position to spawn the vehicle'
    )
    
    z_pos_arg = DeclareLaunchArgument(
        'z',
        default_value='0.2',
        description='Z position to spawn the vehicle'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # PX4 SITL process
    px4_sitl = ExecuteProcess(
        cmd=[
            'make', 'px4_sitl_default',
            PythonExpression([
                "'gazebo_", LaunchConfiguration('vehicle'), "'" if LaunchConfiguration('vehicle') != 'iris' 
                else "'gazebo'"
            ])
        ],
        cwd='/workspace/PX4-Autopilot',
        output='screen',
        environment={
            'PX4_SIM_WORLD': LaunchConfiguration('world'),
            'GAZEBO_MODEL_PATH': '/workspace/simulation/models:/workspace/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models',
            'GAZEBO_RESOURCE_PATH': '/workspace/simulation/worlds',
            'HEADLESS': PythonExpression([
                "'1'" if LaunchConfiguration('headless') == 'true' else "'0'"
            ]),
        }
    )
    
    # MAVROS2 node (started after a delay to allow PX4 SITL to initialize)
    mavros_node = TimerAction(
        period=10.0,  # 10 second delay
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[
                    {'fcu_url': 'udp://:14540@127.0.0.1:14557'},
                    {'gcs_url': ''},
                    {'target_system_id': 1},
                    {'target_component_id': 1},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=[
                    # Add any topic remappings here if needed
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        vehicle_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        headless_arg,
        use_sim_time_arg,
        
        # Launch processes
        px4_sitl,
        mavros_node,
    ])