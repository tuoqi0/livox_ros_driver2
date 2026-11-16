
// ROS 2 message and threading dependencies used by the driver.

#ifndef ROS2_HEADERS_H_
#define ROS2_HEADERS_H_

#include <thread>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#define DRIVER_DEBUG(node, ...) RCLCPP_DEBUG((node).get_logger(), __VA_ARGS__)
#define DRIVER_INFO(node, ...) RCLCPP_INFO((node).get_logger(), __VA_ARGS__)
#define DRIVER_WARN(node, ...) RCLCPP_WARN((node).get_logger(), __VA_ARGS__)
#define DRIVER_ERROR(node, ...) RCLCPP_ERROR((node).get_logger(), __VA_ARGS__)
#define DRIVER_FATAL(node, ...) RCLCPP_FATAL((node).get_logger(), __VA_ARGS__)

#define DRIVER_DEBUG_EXTRA(node, EXTRA, ...) RCLCPP_DEBUG_##EXTRA((node).get_logger(), __VA_ARGS__)
#define DRIVER_INFO_EXTRA(node, EXTRA, ...) RCLCPP_INFO_##EXTRA((node).get_logger(), __VA_ARGS__)
#define DRIVER_WARN_EXTRA(node, EXTRA, ...) RCLCPP_WARN_##EXTRA((node).get_logger(), __VA_ARGS__)
#define DRIVER_ERROR_EXTRA(node, EXTRA, ...) RCLCPP_ERROR_##EXTRA((node).get_logger(), __VA_ARGS__)
#define DRIVER_FATAL_EXTRA(node, EXTRA, ...) RCLCPP_FATAL_##EXTRA((node).get_logger(), __VA_ARGS__)

#endif // ROS2_HEADERS_H_
