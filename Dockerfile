FROM nvcr.io/nvidia/pytorch:23.07-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- System deps ---
RUN apt update && apt install -y \
    locales \
    curl gnupg2 lsb-release software-properties-common \
    build-essential cmake ninja-build git pkg-config \
    python3-dev python3-pip \
    usbutils \
    libgl1 libglib2.0-0 \
    && locale-gen en_US en_US.UTF-8

RUN pip install --upgrade pip

# --- ROS 2 Humble (Ubuntu 22.04 / jammy) ---
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-realsense2-camera \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-sensor-msgs-py \
    python3-colcon-common-extensions \
    python3-rosdep

RUN rosdep init || true && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# --- Pin numpy for ROS2 + GraspGen ---
RUN pip install "numpy==1.26.4"

# --- Common python deps (no torch touches) ---
RUN pip install \
    opencv-python-headless==4.7.0.72 \
    imageio \
    hydra-core omegaconf \
    trimesh==4.5.3 \
    meshcat \
    scikit-learn scipy matplotlib \
    addict yourdfpy==0.0.56

# --- Workspace ---
WORKDIR /workspace
