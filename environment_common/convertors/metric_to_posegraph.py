import os

yml = {
    'use_sim_time': 'false', 
    'mode': 'localization', 
    'map_file_name': '/path/to/your/map.yaml', 
    'map_start_pose': [0.0, 0.0, 0.0] }

param_path = yml.path
#ros2 run slam_toolbox async_slam_toolbox_node params:=param_path

#ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{filename: '/path/to/output_map.posegraph'}"
