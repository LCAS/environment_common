# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def either(folder, file1, file2):
    return f"{folder}/{file1}" if os.path.exists(f"{folder}/{file1}") else f"{folder}/{file2}"

def generate_launch_description():
    ENV = join(get_package_share_directory('environment_template'), 'config')
    LD = LaunchDescription()

    # Define Map References
    map_input = LaunchConfiguration('map')
    metric_default = either(join(ENV, 'metric', 'map'), 'map.yaml', 'map_autogen.yaml')
    LD.add_action(DeclareLaunchArgument('map', default_value=metric_default))

    tmap_input = LaunchConfiguration('tmap')
    tmap_default = either(join(ENV, 'topological'), 'network.tmap2.yaml', 'network_autogen.tmap2.yaml')
    LD.add_action(DeclareLaunchArgument('tmap', default_value=tmap_default))

    gazebo_input = LaunchConfiguration('world')
    gazebo_default = either(join(ENV, 'world'), 'gazebo.world', 'gazebo_autogen.world')
    LD.add_action(DeclareLaunchArgument('world', default_value=gazebo_default))


    # Declare namespace
    #namespace = LaunchConfiguration('namespace')
    #LD.add_action(DeclareLaunchArgument('namespace', default_value=namespace, description='Top-level namespace'))

    # Define map node
    """
    LD.add_action(GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
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
    """

    ## Topological Map Server
    LD.add_action(Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topomap2_server',
        arguments=[tmap_default]
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='topological_transform_publisher'
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topomap_marker.py',
        name='topomap_marker'
    ))

    ## Execute all Components
    return LD


