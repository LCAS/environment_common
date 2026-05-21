# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from nav2_common.launch import RewrittenYaml


def declare3(arg_name, description, envvar='', default=None):
    """ Declare the default arg value from environment variable, or default value.
    """
    env_value = os.getenv(envvar) if envvar else None
    default_value = env_value if env_value else default
    return DeclareLaunchArgument(
        arg_name,
        default_value=default_value,
        description=description
    )

def either(basic, autogen):
    return basic if os.path.exists(basic) else autogen

def generate_launch_description():
    CONF = os.path.join(get_package_share_directory('environment_common'), 'config')
    ENV = os.path.join(get_package_share_directory('environment_template'), 'config')
    LD = LaunchDescription()

    ################################################
    # Define default values for map args
    ################################################

    # METRIC MAP
    desc=f'Full path to the metric map yaml file.'
    dir = os.path.join(ENV, 'metric', 'map')
    basic = os.path.join(dir, 'map.yaml')
    autogen = os.path.join(dir, 'map_autogen.yaml')
    metric_default_value = either(basic, autogen)
    LD.add_action(declare3('map', desc, envvar='COSTMAP_YAML_FILE', default=metric_default_value))
    map_input = LaunchConfiguration('map')

    # NOGO METRIC MAP
    desc=f'Full path to the no-go metric map yaml file.'
    dir = os.path.join(ENV, 'metric', 'nogo')
    basic = os.path.join(dir, 'map.yaml')
    autogen = os.path.join(dir, 'map_autogen.yaml')
    nogo_metric_default_value = either(basic, autogen)
    LD.add_action(declare3('nogomap', desc, envvar='NOGO_COSTMAP_YAML_FILE', default=nogo_metric_default_value))
    nogomap_input = LaunchConfiguration('nogomap')

    # TOPOLOGICAL MAP
    dir = os.path.join(ENV, 'topological')
    basic = os.path.join(dir, 'network.tmap2.yaml')
    autogen = os.path.join(dir, 'network_autogen.tmap2.yaml')
    tmap_default_value = either(basic, autogen)
    LD.add_action(declare3('tmap', desc, envvar='TMAP_FILE', default=tmap_default_value))
    tmap_input = LaunchConfiguration('tmap')

    # GAZEBO WORLD
    dir = os.path.join(ENV, 'world')
    basic = os.path.join(dir, 'gazebo.world')
    autogen = os.path.join(dir, 'gazebo_autogen.world')
    gazebo_default_value = either(basic, autogen)
    LD.add_action(declare3('world', desc, envvar='GAZEBO_WORLD_FILE', default=gazebo_default_value))
    gazebo_input = LaunchConfiguration('world')

    # Display status
    print('\n'*5)
    print('--'*10)
    print('Metric:')
    print('Default:', metric_default_value)
    print('Loading:', map_input)
    print('\n'*2)
    print('Nogo Metric:')
    print('Default:', nogo_metric_default_value)
    print('Loading:', nogomap_input)
    print('\n'*2)
    print('TMap:')
    print('Default:', tmap_default_value)
    print('Loading:', tmap_input)
    print('\n'*2)
    print('Gazebo:')
    print('Default:', gazebo_default_value)
    print('Loading:', gazebo_input)
    print('--'*10)
    print('\n'*5)


    ################################################
    # Define default values for parameters
    ################################################


    # RVIZ PATH
    rviz_default_value = os.path.join(CONF, 'topomap_marker.rviz')
    LD.add_action(DeclareLaunchArgument('rviz', default_value=rviz_default_value))
    rviz_input = LaunchConfiguration('rviz')
    LD.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    use_rviz_input = LaunchConfiguration('use_rviz')
    LD.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    use_sim_time_input = LaunchConfiguration('use_sim_time')
    sim_time_params = [{'use_sim_time': use_sim_time_input}]

    # Declare the param file
    param_file = os.path.join(CONF, 'params_map_server.yaml')
    LD.add_action(DeclareLaunchArgument('params_file', default_value=param_file))
    param_input = LaunchConfiguration('params_file')
    configured_map_params = RewrittenYaml(
        source_file=param_input,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time_input,
            'yaml_filename': map_input
        },
        convert_types=True
    )
    configured_nogomap_params = RewrittenYaml(
        source_file=param_input,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time_input,
            'yaml_filename': nogomap_input
        },
        convert_types=True
    )
    lifecycle_nodes = []

    ## Topological Map Server
    try:
        pkg = get_package_share_directory('topological_navigation')
    except:
        pkg = None
    if pkg:
        LD.add_action(Node(
            package='topological_navigation',
            executable='map_manager2.py',
            name='topomap2_server',
            parameters=sim_time_params,
            arguments=[tmap_input]
        ))
        LD.add_action(Node(
            package='topological_navigation',
            executable='topological_transform_publisher.py',
            name='topological_transform_publisher',
            parameters=sim_time_params
        ))
        LD.add_action(Node(
            package='topological_navigation',
            executable='topomap_marker2.py',
            name='topomap_marker2',
            parameters=sim_time_params
        ))



    ## Costmap Map Server
    try:
        pkg = get_package_share_directory('nav2_map_server')
    except:
        pkg = None
    if pkg:

        # Normal environment map
        LD.add_action(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_map_params]
        ))
        lifecycle_nodes.append('map_server')


        # Keepout / no-go mask map server
        LD.add_action(Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            output='screen',
            parameters=[configured_nogomap_params]
        ))
        lifecycle_nodes.append('keepout_mask_server')


        # Publishes metadata telling Nav2 how to interpret the mask
        LD.add_action(Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[RewrittenYaml(
                source_file=param_input,
                root_key='',
                param_rewrites={
                    'use_sim_time': use_sim_time_input
                },
                convert_types=True
            )]
        ))
        lifecycle_nodes.append('costmap_filter_info_server')



    ## Lifecycle Manager
    try:
        pkg = get_package_share_directory('nav2_lifecycle_manager')
    except:
        pkg = None
    if pkg:
        LD.add_action(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_input,
                'autostart': True,
                'node_names': lifecycle_nodes
            }]
        ))



    ## Gazebo World Server
    try:
        pkg = get_package_share_directory('gazebo_ros')
    except:
        pkg = None
    if pkg:
        launch_file = os.path.join(pkg, 'launch', 'gazebo.launch.py')
        LD.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file]),
                launch_arguments={'world': gazebo_input}.items(),
                condition=IfCondition(
                    PythonExpression(["(lambda f: f != '' and __import__('os').path.exists(f))('",LaunchConfiguration('world'),"')"])
                )
            )
        )


    ## RViz2
    LD.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_input, '--ros-args', '-p', ['use_sim_time:=', use_sim_time_input]],
        condition=IfCondition(use_rviz_input)
    ))


    ## Execute all Components
    return LD

