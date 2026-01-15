FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ---- dependencias base ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmodbus-dev pkg-config \
    build-essential \
    git \
    nano \
    wget \
    curl \
    iputils-ping \
    net-tools \
    iproute2 \
    lsb-release \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-pip \
    ros-jazzy-ur-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-controller-manager \
    && rm -rf /var/lib/apt/lists/*

# ---- MoveIt2 (en Jazzy puede variar qué meta-paquete existe) ----
# Intentamos instalar moveit; si tu repo usa MoveIt desde source, puedes omitir esto.
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-common \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    || true \
    && rm -rf /var/lib/apt/lists/*

# ---- rosdep ----
RUN rosdep init || true
RUN rosdep update

WORKDIR /workspace_ur5

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
