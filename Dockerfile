# =============================================================================
# Argos ROS 2 Humble Development Container
# =============================================================================
# This Dockerfile builds a full ROS 2 Humble development environment with
# all the tools needed for the Argos SAR quadruped project.
# =============================================================================

FROM osrf/ros:humble-desktop

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# --- System packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    gdb \
    git \
    wget \
    curl \
    nano \
    vim \
    # Python
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    # ROS 2 packages — navigation + SLAM
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-robot-localization \
    ros-humble-rviz2 \
    # ROS 2 packages — RealSense
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    # ROS 2 packages — RPLidar
    ros-humble-rplidar-ros \
    ros-humble-laser-filters \
    ros-humble-pointcloud-to-laserscan \
    # ROS 2 packages — image / perception
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-view \
    ros-humble-rqt-image-view \
    # ROS 2 packages — diagnostics
    ros-humble-diagnostic-updater \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-rqt-publisher \
    ros-humble-micro-ros-agent \
    # Misc tools
    apt-utils \
    iputils-ping \
    net-tools \
    iproute2 \
    ssh \
    rsync \
    usbutils \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# --- Python packages ---
RUN pip3 install --no-cache-dir \
    transforms3d \
    numpy \
    scipy \
    matplotlib \
    opencv-python-headless \
    pyrealsense2

# --- Shell prompt ---
RUN echo "PS1='\[\033[1;36m\][argos-dev]\[\033[0m\] \u@\h:\w\$ '" >> /etc/bash.bashrc

# --- Default ROS 2 source ---
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# --- Workspace ---
WORKDIR /workspace

CMD ["bash"]
