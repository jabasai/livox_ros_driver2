FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

# Install system and ROS2 dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libpcl-dev \
    libapr1-dev \
    libaprutil1-dev \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rcl-interfaces \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rosbag2 \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

RUN \
    echo "Installing livox_sdk2..." && \
    REPO_DIR="/tmp/livox_sdk2_repo" && \
    mkdir -p "$REPO_DIR" && \
    git clone --branch v1.2.5 --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$REPO_DIR/Livox-SDK2" && \
    cd "$REPO_DIR/Livox-SDK2" && \
    mkdir -p build && \
    cd build && \
    rm -rf * && \
    cmake ..  && \
    make -j$(nproc) && \
    make install && \
    echo "livox_sdk2 installed successfully." && \
    cd "/" && \
    rm -rf "$REPO_DIR" && \
    ldconfig


# Source ROS2 in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set up workspace directory
WORKDIR /ros2_ws
