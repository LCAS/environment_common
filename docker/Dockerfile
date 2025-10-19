# Start from full ROS2 Humble desktop image
FROM osrf/ros:humble-desktop-full

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV SHELL=/bin/bash

# Create workspace
RUN mkdir -p /home/ros/ros2_ws/src
WORKDIR /home/ros/ros2_ws

# Install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update || true

# Ensure entrypoint is ROS-aware
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["bash", "-c"]
CMD ["source /opt/ros/$ROS_DISTRO/setup.bash && exec bash"]
