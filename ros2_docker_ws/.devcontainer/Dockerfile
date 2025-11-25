# Use the official ROS 2 Humble base
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install required packages
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg \
    lsb-release \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    git \
    build-essential \
    python3-pip \
    openssh-client \
    sudo \
    vim && \
    apt clean

# Locale setup
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch shell to bash
SHELL ["/bin/bash", "-c"]

# Ensure ALL users source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set default user (important for VS Code)
USER $USERNAME
# Working directory can be left as /workspace or unset; VS Code opens workspaceFolder
WORKDIR /workspace/ros2_docker_ws/ros2_ws

