# Use the official ROS 2 Humble image
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# Locale (ROS2 requirement)
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS2 Humble sources
RUN apt update && apt install -y curl gnupg lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Desktop Full + common dev tools
RUN apt update && apt install -y \
    ros-humble-desktop-full \
    ros-dev-tools \
    python3-colcon-common-extensions \
    openssh-client \
    git \
    build-essential \
    python3-pip \
    sudo \
    && apt clean

    
# Create non-root user with passwordless sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set default shell
SHELL ["/bin/bash", "-c"]

# Working directory can be left as /workspace or unset; VS Code opens workspaceFolder
WORKDIR /workspace/ros2_docker_ws/ros2_ws

# Automatically source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc