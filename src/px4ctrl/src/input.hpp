#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "px4ctrl/msg/position_command.hpp"
#include "px4ctrl/msg/position_command_trajectory.hpp"

struct OdomData
{
  nav_msgs::msg::Odometry msg;
  rclcpp::Time stamp;
  bool received{false};

  void feed(const nav_msgs::msg::Odometry::SharedPtr &m);
};

struct ImuData
{
  sensor_msgs::msg::Imu msg;
  rclcpp::Time stamp;
  bool received{false};

  void feed(const sensor_msgs::msg::Imu::SharedPtr &m);
};

struct CommandData
{
  px4ctrl::msg::PositionCommand msg;
  rclcpp::Time stamp;
  bool received{false};

  void feed(const px4ctrl::msg::PositionCommand::SharedPtr &m);
  bool ready() const;
};

struct TrajectoryData
{
  px4ctrl::msg::PositionCommandTrajectory msg;
  rclcpp::Time stamp;
  bool received{false};

  void feed(const px4ctrl::msg::PositionCommandTrajectory::SharedPtr &m);
  bool ready() const;
};

struct GoalPoseData
{
  geometry_msgs::msg::PoseStamped msg;
  rclcpp::Time stamp;
  bool received{false};

  void feed(const geometry_msgs::msg::PoseStamped::SharedPtr &m);
  bool ready() const;
};
