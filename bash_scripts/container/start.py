#!/usr/bin/env bash
# ------------------------------------------------------------
# Custom environment for TopFleets agent‑container
# ------------------------------------------------------------

##1
echo -e "\n\n\nRunnng apt update\n"
sudo apt-get update

#2
echo -e "\n\n\nSourcing /opt/ros/setup.bash\n"
source /opt/ros/humble/setup.bash


#3
echo -e "\n\n\nSet working directory to /home/ros/ros2_ws\n"
[ -d "$HOME/ros2_ws" ] && cd "$HOME/ros2_ws"


##4
echo -e "\n\n\nRunning ROSDep update and install\n\n"
rosdep update
rosdep install --from-paths src --ignore-src -r -y


#5
echo -e "\n\n\nBuilding Colcon Workspace at /home/ros/ros2_ws\n\n"
[ -d "$HOME/ros2_ws/build" ] && sudo rm -r $HOME/ros2_ws/build
[ -d "$HOME/ros2_ws/install" ] && sudo rm -r $HOME/ros2_ws/install
[ -d "$HOME/ros2_ws/log" ] && sudo rm -r $HOME/ros2_ws/log
colcon build --packages-select environment_template environment_common


#6
echo -e "\n\n\nSourcing built workspace\n\n"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"


#7
echo -e "\n\n\nCustomise ros logger\n\n"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{severity}: {message}"
export RCUTILS_COLORIZED_OUTPUT=1


#8
echo -e "\n\n\nLaunching TopFleets Connection Script\n\n"
ros2 run environment_common fill_gaps.py

