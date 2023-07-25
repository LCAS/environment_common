# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ENV = get_package_share_directory('environment_template')

    # Map
    map = os.path.join(ENV, 'config', 'metric', 'map', 'map.png')
    pkg = get_package_share_directory('nav2_map_server')
    # Map Server
    file = os.path.join(pkg, 'launch', 'map_server.launch.py')
    src = PythonLaunchDescriptionSource(file)
    args = {'map': map}.items()
    map_server_cmd = IncludeLaunchDescription(src, launch_arguments={'map': map}.items())


    # Topological Map
    tmap = os.path.join(ENV, 'config', 'metric', 'map', 'map.png')
    pkg = get_package_share_directory('topological_navigation')
    # Map Server
    file = os.path.join(pkg, 'launch', 'tmap_server.launch.py')
    src = PythonLaunchDescriptionSource(file)
    args = {'tmap': tmap}.items()
    tmap_manager_cmd = IncludeLaunchDescription(src, launch_arguments={'tmap': tmap}.items())


    # Gazebo
    world = os.path.join(ENV, 'config', 'world', 'gazebo.world')
    pkg = get_package_share_directory('gazebo_ros')
    # Gazebo Server
    file = os.path.join(pkg, 'launch', 'gzserver.launch.py')
    src = PythonLaunchDescriptionSource(file)
    args = {'world': world}.items()
    gzserver_cmd = IncludeLaunchDescription(src, launch_arguments=args)
    # Gazebo Client
    file = os.path.join(pkg, 'launch', 'gzclient.launch.py')
    src = PythonLaunchDescriptionSource(file)
    gzclient_cmd = IncludeLaunchDescription(src)


    # Create a node manager
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(tmap_manager_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
