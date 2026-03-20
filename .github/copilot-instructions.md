# Livox ROS Driver 2 — Copilot Instructions

## Project Overview

This is a ROS2 device driver for Livox 3D LiDAR sensors (MID360, HAP). It targets **ROS2 Humble or later**. The codebase also retains partial ROS1 compatibility gated behind `#ifdef BUILDING_ROS1` preprocessor guards. But only every work on the ROS2 code.

## Build and Test

```bash
# From the workspace root (/ros2_ws)
colcon build

```

## Architecture

- **`src/driver_node.*`** — ROS2 node entry point, lifecycle management
- **`src/lddc.*`** — Lidar Data Distribution Center; routes point cloud and IMU data to ROS publishers
- **`src/lds_lidar.*` / `src/lds.*`** — Lidar Device Scheduler; manages sensor connections and data pipelines
- **`src/comm/`** — Low-level communication utilities: queues, semaphores, caching, and the central `PubHandler`
- **`src/call_back/`** — SDK callback handlers for incoming lidar/IMU frames
- **`src/parse_cfg_file/`** — JSON config file parsing (uses RapidJSON from `3rdparty/`)
- **`src/include/`** — Shared headers; `ros_headers.h` abstracts ROS1/ROS2 includes
- **`config/`** — Per-sensor JSON configuration files
- **`launch/`** — ROS2 Python launch files

## Code Conventions

- **Language**: C++14, namespace `livox_ros`
- **Naming**: `PascalCase` for classes, `snake_case` for functions and member variables
- **Header guards**: `#ifndef LIVOX_<MODULE>_H` / `#define LIVOX_<MODULE>_H`
- **ROS abstraction**: Use the wrappers in `src/include/ros_headers.h` rather than including ROS headers directly; guard any ROS1-specific code with `#ifdef BUILDING_ROS1`
- **JSON parsing**: Use the bundled RapidJSON library in `3rdparty/rapidjson/`
- **License header**: All new source files must include the MIT license block present in existing files

## Dependencies

Key runtime deps: `rclcpp`, `rclcpp_components`, `sensor_msgs`, `pcl_conversions`, `livox_lidar_sdk`.  
Do **not** add new third-party libraries without updating both `CMakeLists.txt` and `package.xml`.
