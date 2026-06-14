# Livox ROS Driver 2 — Copilot Instructions

## Project Overview

This is a ROS2 device driver for Livox 3D LiDAR sensors (MID360, HAP). It targets **ROS2 Humble or later**. The codebase also retains partial ROS1 compatibility gated behind `#ifdef BUILDING_ROS1` preprocessor guards. But only ever work on the ROS2 code.

The driver supports running a **single node that manages multiple lidars** (the recommended approach for dual MID360 setups): one SDK instance handles both lidars, eliminating cross-instance SDK race conditions. See `launch/dual_mid360_launch.py`.

## Build and Test

```bash
# From the workspace root (/ros2_ws)
colcon build

```

## Architecture

- **`src/driver_node.*`** — ROS2 node entry point, lifecycle management; includes watchdog timer and Race-3 reconnect timer
- **`src/lddc.*`** — Lidar Data Distribution Center; routes point cloud and IMU data to ROS publishers
- **`src/lds_lidar.*` / `src/lds.*`** — Lidar Device Scheduler; manages sensor connections and data pipelines
- **`src/comm/`** — Low-level communication utilities: queues, semaphores, caching, and the central `PubHandler` (with per-handle allowlist)
- **`src/call_back/`** — SDK callback handlers for incoming lidar/IMU frames
- **`src/parse_cfg_file/`** — JSON config file parsing (uses RapidJSON from `3rdparty/`)
- **`src/include/`** — Shared headers; `ros_headers.h` abstracts ROS1/ROS2 includes; `livox_log.h` provides node-free logging macros
- **`config/`** — Per-sensor JSON configuration files
- **`launch/`** — ROS2 Python launch files; `dual_mid360_launch.py` launches ONE node for BOTH MID360 lidars
- **`src/tools/`** — Standalone CLI tools: `livox_scan` (discover lidars) and `livox_set_ip` (configure lidar IPs)

## Code Conventions

- **Language**: C++14, namespace `livox_ros`
- **Naming**: `PascalCase` for classes, `snake_case` for functions and member variables
- **Header guards**: `#ifndef LIVOX_<MODULE>_H` / `#define LIVOX_<MODULE>_H`
- **ROS abstraction**: Use the wrappers in `src/include/ros_headers.h` rather than including ROS headers directly; guard any ROS1-specific code with `#ifdef BUILDING_ROS1`
- **Logging**: Use `LIVOX_INFO` / `LIVOX_WARN` / `LIVOX_ERROR` / `LIVOX_DEBUG` macros from `src/include/livox_log.h` for node-free logging; they route to `RCLCPP_*` in ROS2 and `ROS_*` in ROS1
- **Thread safety**: `connect_state` in `LidarDevice` is `std::atomic<LidarConnectState>`; use `memory_order_acquire` / `memory_order_release` at all access sites
- **JSON parsing**: Use the bundled RapidJSON library in `3rdparty/rapidjson/`
- **License header**: All new source files must include the MIT license block present in existing files

## Dependencies

Key runtime deps: `rclcpp`, `rclcpp_components`, `sensor_msgs`, `pcl_conversions`, `livox_lidar_sdk`.  
Do **not** add new third-party libraries without updating both `CMakeLists.txt` and `package.xml`.
