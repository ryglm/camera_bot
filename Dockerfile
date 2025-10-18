# syntax=docker/dockerfile:1.7
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

# Base tooling + ROS pkgs required by this repo and README flows
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl tini \
    build-essential cmake python3-colcon-common-extensions \
    # core runtime libs used by this package and README commands
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs ros-jazzy-sensor-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
    # Gazebo Sim + ROS-GZ (server, GUI, bridge)
    ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
    # QoL
    nano less \
  && rm -rf /var/lib/apt/lists/*

# SLAM 
ARG WITH_RTABMAP=1
RUN if [ "$WITH_RTABMAP" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rtabmap-ros && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

# Create workspace
WORKDIR /ws

COPY package.xml CMakeLists.txt /ws/src/camera_bot/

RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p /ws/src && \
    colcon build --event-handlers console_cohesion+ --packages-select camera_bot || true

COPY . /ws/src/camera_bot

# Build package
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-clean-cache --symlink-install --packages-select camera_bot

# Source the environemtn
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/install/setup.bash" >> /etc/bash.bashrc

# Entry
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/usr/bin/tini","--","/entrypoint.sh"]
CMD ["bash"]

