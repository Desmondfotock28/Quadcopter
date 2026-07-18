# Copyright 2023 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Doron Hirshberg */
import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys

# Ensure launch_utils can be found
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

def generate_launch_description():
    # 1. Get model from command line arguments
    params = dict([aa for aa in [aa.split(':=') for aa in sys.argv] if len(aa) == 2])
    
    if 'model' not in params:
        print('USAGE: ros2 launch realsense2_description view_model.launch.py model:=test_d415_camera.urdf.xacro')
        return launch.LaunchDescription()

    rviz_config_dir = os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', params['model'])
    
    # 2. Convert Xacro to URDF string
    # Ensure nominal extrinsics is true to generate the optical frames needed for NMPC
    urdf_content = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})

    # 3. Robot State Publisher Node
    model_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True  # CRITICAL: Match Gazebo clock
        }]
    )

    # 4. RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}] # CRITICAL: Match Gazebo clock
    )

    return launch.LaunchDescription([
        model_node,
        rviz_node
    ])
