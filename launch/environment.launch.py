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


    # Other Launch Args
    #launch_list += [DeclareLaunchArgument('params_file', default_value=param_dir)]
    #launch_list += [DeclareLaunchArgument('use_sim_time', default_value='false')]



    # Declare namespace
    namespace = LaunchConfiguration('namespace')
    LD.add_action(DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'))

    # Define map node
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




    ## Metric Map Server
    #launch_list += [Node(
    #    package='nav2_map_server',
    #    executable='map_server',
    #    name='map_server',
    #    arguments=[topological_ref]
    #)]
    #Save map using: `ros2 run nav2_map_server map_saver_cli -f ./map_autogen.png`


    #nav2_launch_file_dir = join(get_package_share_directory('nav2_bringup'), 'launch')
    #launch_list += [IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
    #    launch_arguments={
    #        'map': map_dir,
    #        'use_sim_time': use_sim_time,
    #        'params_file': param_dir
    #    }.items()
    #)

    # https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/tb3_simulation_launch.py
    #bringup_dir = get_package_share_directory('nav2_bringup')
    #launch_dir = os.path.join(bringup_dir, 'launch')
    #launch_list += [IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
    #    launch_arguments={'namespace': namespace,
    #                      'use_namespace': use_namespace,
    #                      'slam': slam,
    #                      'map': map_yaml_file,
    #                      'use_sim_time': use_sim_time,
    #                      'params_file': params_file,
    #                      'autostart': autostart,
    #                      'use_composition': use_composition,
    #                      'use_respawn': use_respawn}.items()
    #)]

    # https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py
    #launch_list += [
    #    IncludeLaunchDescription(
    #        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
    #        launch_arguments={'namespace': namespace,
    #                          'map': map_yaml_file,
    #                          'use_sim_time': use_sim_time,
    #                          'autostart': autostart,
    #                          'params_file': params_file,
    #                          'use_composition': use_composition,
    #                          'use_respawn': use_respawn,
    #                          'container_name': 'nav2_container'}.items()),
    #]



    #ros2 launch nav2_bringup localization_launch.py map:=$METRIC_FILE
    #^^^^ this one works, just run rviz first


    # Map Server
    #launch_list += [GroupAction(
    #    actions=[
    #        SetParameter('use_sim_time', 'false'),
    #        Node(
    #            package='nav2_map_server',
    #            executable='map_server',
    #            name='map_server',
    #            output='screen',
    #            respawn=False,
    #            respawn_delay=2.0,
    #            parameters=[ #configured_params,
    #                        {'yaml_filename': metric_ref}],
    #            arguments=['--ros-args', '--log-level', 'info']),
    #        Node(
    #            package='nav2_lifecycle_manager',
    #            executable='lifecycle_manager',
    #            name='lifecycle_manager_localization',
    #            output='screen',
    #            arguments=['--ros-args', '--log-level', 'info'],
    #            parameters=[{'autostart': True},
    #                        {'node_names': ['map_server']}])
    #    ]
    #)]



    #pkg = get_package_share_directory('nav2_map_server')
    #map_server_cmd = IncludeLaunchDescription(
    #  src = PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'map_server.launch.py')),
    #  launch_arguments = {'map': map}.items()
    #)






    ## Topological Map Server
    """
    launch_list += [Node(
        package='topological_navigation',
        executable='map_manager2.py',
        name='topomap2_server',
        arguments=[topological_ref]
    )]
    launch_list += [Node(
        package='topological_navigation',
        executable='topological_transform_publisher.py',
        name='topological_transform_publisher'
    )]
    """

    ## Execute all Components
    return LD












    """
    Node(
      package
      executable
      name
      namespace
      exec_name
      parameters
      remappings
      ros_arguments
      arguments
    )
    """






#
#
#    # Topological Map
#    pkg = get_package_share_directory('topological_navigation')
#    # Map Server
#    file = os.path.join(pkg, 'launch', 'tmap_server.launch.py')
#    src = PythonLaunchDescriptionSource(file)
#    args = {'tmap': tmap}.items()
#    tmap_manager_cmd = IncludeLaunchDescription(src, launch_arguments=args)
#
#
#    # Gazebo
#    pkg = get_package_share_directory('gazebo_ros')
#    # Gazebo Server
#    file = os.path.join(pkg, 'launch', 'gzserver.launch.py')
#    src = PythonLaunchDescriptionSource(file)
#    args = {'world': world}.items()
#    gzserver_cmd = IncludeLaunchDescription(src, launch_arguments=args)
#    # Gazebo Client
#    file = os.path.join(pkg, 'launch', 'gzclient.launch.py')
#    src = PythonLaunchDescriptionSource(file)
#    gzclient_cmd = IncludeLaunchDescription(src)


    # Create a node manager

    # Add the commands to the launch description
#    ld.add_action(map_server_cmd)
#    ld.add_action(tmap_manager_cmd)
#    ld.add_action(gzserver_cmd)
#    ld.add_action(gzclient_cmd)

    return LD

