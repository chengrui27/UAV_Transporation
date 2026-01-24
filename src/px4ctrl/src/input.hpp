#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "px4ctrl/msg/position_command.hpp"

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
