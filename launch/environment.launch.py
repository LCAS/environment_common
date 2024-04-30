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

from launch_ros.actions import Node, SetParameter

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command


def either(folder, file1, file2):
    return f"{folder}/{file1}" if os.path.exists(f"{folder}/{file1}") else f"{folder}/{file2}"

def generate_launch_description():
    ENV = join(get_package_share_directory('environment_template'), 'config')
    LD = LaunchDescription()

    # Define default values for parameters
    metric_default_value = either(join(ENV, 'metric', 'map'), 'map.yaml', 'map_autogen.yaml')
    tmap_default_value = either(join(ENV, 'topological'), 'network.tmap2.yaml', 'network_autogen.tmap2.yaml')
    gazebo_default_value = either(join(ENV, 'world'), 'gazebo.world', 'gazebo_autogen.world')

    # Declare the launch arguments
    LD.add_action(DeclareLaunchArgument('map', default_value=metric_default_value))
    LD.add_action(DeclareLaunchArgument('tmap', default_value=tmap_default_value))
    LD.add_action(DeclareLaunchArgument('world', default_value=gazebo_default_value))

    # Define references for the arg inputs to be saved
    map_input = LaunchConfiguration('map')
    tmap_input = LaunchConfiguration('tmap')
    gazebo_input = LaunchConfiguration('world')

    # Declare namespace
    #namespace = LaunchConfiguration('namespace')
    #LD.add_action(DeclareLaunchArgument('namespace', default_value=namespace, description='Top-level namespace'))

    # Display status
    print('\n'*5)
    print('--'*10)
    print('Default map:', metric_default_value)
    print('Default tmap:', tmap_default_value)
    print('Default gazebo:', gazebo_default_value)
    print('--'*10)
    print('Loading map:', map_input)
    print('Loading tmap:', tmap_input)
    print('Loading gazebo:', gazebo_input)
    print('--'*10)
    print('\n'*5)

    ## Topological Map Server
    LD.add_action(Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topomap2_server',
        arguments=[tmap_input]
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='topological_transform_publisher'
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topomap_marker2.py',
        name='topomap_marker2'
    ))

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


    ## RViz2
    LD.add_action(Node(
        package='rviz2',
        executable='rviz2'
    ))

    ## Execute all Components
    return LD


