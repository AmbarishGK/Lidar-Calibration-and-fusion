# Start with ROS 2 Humble as base
FROM osrf/ros:humble-desktop

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install necessary system dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace for MOLA
WORKDIR /root/mola_ws
RUN mkdir -p /root/mola_ws/src

# Install ROS 2 MOLA packages
RUN apt-get update && apt-get install -y \
    ros-humble-mola \
    ros-humble-mola-state-estimation \
    ros-humble-mola-lidar-odometry \
    ros-humble-mola-test-datasets \
    ros-humble-mvsim \
    && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source ROS 2 and keep the container running
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
