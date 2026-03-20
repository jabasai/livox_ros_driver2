// MIT License — see project root for full license text.
//
// livox_log.h
// -----------
// Lightweight, node-free logging macros for internal driver code that runs
// outside of a ROS node context (callbacks, data-processing threads, etc.).
//
// Usage (printf-style format string):
//   LIVOX_INFO("lidar %s ready", ip_str.c_str());
//   LIVOX_WARN("retry #%d for %s", n, ip_str.c_str());
//
// ROS2 : routes to RCLCPP_* with a named "livox_driver" logger.
// ROS1 : routes to the standard ROS_* macros.
// Other: falls back to fprintf(stderr/stdout).

#ifndef LIVOX_LOG_H_
#define LIVOX_LOG_H_

#ifdef BUILDING_ROS2

#include <rclcpp/rclcpp.hpp>
// Retrieve a cached named logger so the overhead is negligible.
namespace livox_ros { namespace detail {
inline rclcpp::Logger get_logger() {
  return rclcpp::get_logger("livox_driver");
}
}} // namespace livox_ros::detail

#define LIVOX_DEBUG(...) RCLCPP_DEBUG(livox_ros::detail::get_logger(), __VA_ARGS__)
#define LIVOX_INFO(...)  RCLCPP_INFO (livox_ros::detail::get_logger(), __VA_ARGS__)
#define LIVOX_WARN(...)  RCLCPP_WARN (livox_ros::detail::get_logger(), __VA_ARGS__)
#define LIVOX_ERROR(...) RCLCPP_ERROR(livox_ros::detail::get_logger(), __VA_ARGS__)

#elif defined BUILDING_ROS1

#include <ros/ros.h>
#define LIVOX_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define LIVOX_INFO(...)  ROS_INFO(__VA_ARGS__)
#define LIVOX_WARN(...)  ROS_WARN(__VA_ARGS__)
#define LIVOX_ERROR(...) ROS_ERROR(__VA_ARGS__)

#else  // non-ROS build (tools, unit tests)

#include <cstdio>
#define LIVOX_DEBUG(...) do {} while (0)
#define LIVOX_INFO(...)  do { fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); } while (0)
#define LIVOX_WARN(...)  do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#define LIVOX_ERROR(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while (0)

#endif  // BUILDING_ROS2 / BUILDING_ROS1

#endif  // LIVOX_LOG_H_
