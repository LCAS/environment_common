# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Specify map.yaml reference
    map_yaml_file = LaunchConfiguration('map')
    ld.add_action(DeclareLaunchArgument('map',
        description='Full path to map yaml file to load',
        default_value='/home/james/ros2_ws/src/environment_template/config/metric/map/map_autogen.yaml'))

    # Declare namespace
    namespace = LaunchConfiguration('namespace')
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))

    # Declare Paramaters
    param_substitutions = {
        'use_sim_time': False,
        'yaml_filename': map_yaml_file}

    params_file = LaunchConfiguration('params_file')
    ld.add_action(DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'))

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)



    # Add the actions to launch the map server node
    ld.add_action(GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[configured_params],
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': ['map_server']}])
        ]
    ))

    return ld
