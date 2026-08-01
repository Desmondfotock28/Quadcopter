import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('nmpc_px4_ros2_bringup')
    ref_traj_config = os.path.join(bringup_share, 'config', 'ref_traj_params.yaml')
    nmpc_config = os.path.join(bringup_share, 'config', 'nmpc_params.yaml')

    enable_reference_publisher = LaunchConfiguration('enable_reference_publisher')
    enable_nmpc_controller = LaunchConfiguration('enable_nmpc_controller')
    use_rviz = LaunchConfiguration('use_rviz')
    ref_traj = LaunchConfiguration('ref_traj')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_reference_publisher',
            default_value='false',
            description='Start reference trajectory publishing. Default is false for safety.',
        ),
        DeclareLaunchArgument(
            'enable_nmpc_controller',
            default_value='false',
            description='Start the NMPC direct-actuator controller. Default is false for safety.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz on systems with a display.',
        ),
        DeclareLaunchArgument(
            'ref_traj',
            default_value='static',
            description='Reference trajectory name from nmpc_px4_ros2_utils/config/traj.',
        ),
        Node(
            package='nmpc_px4_ros2_utils',
            executable='odom_repub_node',
            name='odom_repub_node',
            output='screen',
        ),
        Node(
            package='nmpc_px4_ros2_utils',
            executable='ref_traj_pub_node',
            name='ref_traj_pub_node',
            parameters=[ref_traj_config, {'ref_traj': ref_traj}],
            output='screen',
            condition=IfCondition(enable_reference_publisher),
        ),
        Node(
            package='nmpc_px4_ros2',
            executable='nmpc_flight_mode',
            name='nmpc_flight_mode',
            parameters=[
                nmpc_config,
                {
                    'hardware_safety_gate': True,
                    'require_safety_switch': True,
                    'require_preflight_checks': True,
                },
            ],
            output='screen',
            condition=IfCondition(enable_nmpc_controller),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'config.rviz')],
            condition=IfCondition(use_rviz),
        ),
    ])
