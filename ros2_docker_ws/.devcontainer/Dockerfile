# Use the official ROS 2 Humble image
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install useful tools and packages
RUN apt update && apt install -y \
    openssh-client \
    git \
    curl \
    build-essential \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-pip \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    && apt clean

# Set default shell
SHELL ["/bin/bash", "-c"]

# Working directory can be left as /workspace or unset; VS Code opens workspaceFolder
WORKDIR /workspace/ros2_docker_ws/ros2_ws

# Automatically source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc