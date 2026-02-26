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

# Source ROS2 in every new shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set up workspace directory
WORKDIR /ros2_ws
