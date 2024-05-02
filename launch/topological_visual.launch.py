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


def generate_launch_description():
    LD = LaunchDescription()

    # Define Map References
    topic_input = LaunchConfiguration('topic')
    LD.add_action(DeclareLaunchArgument('topic', default_value='/topological_map_2'))

    ## Topological Map Server
    LD.add_action(Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='topological_transform_publisher'
    ))
    LD.add_action(Node(
        package='topological_navigation',
        executable='topomap_marker2.py',
        name='topomap_marker2',
        remappings=[
            ('/topological_map_2', topic_input)
        ]
    ))


    LD.add_action(Node(
        package='rviz2',
        executable='rviz2'
    ))

    ## Execute all Components
    return LD


