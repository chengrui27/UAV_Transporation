#include "input.hpp"

void OdomData::feed(const nav_msgs::msg::Odometry::SharedPtr &m)
{
  msg = *m;
  stamp = rclcpp::Time(msg.header.stamp);
  received = true;
}

void ImuData::feed(const sensor_msgs::msg::Imu::SharedPtr &m)
{
  msg = *m;
  stamp = rclcpp::Time(msg.header.stamp);
  received = true;
}

void CommandData::feed(const px4ctrl::msg::PositionCommand::SharedPtr &m)
{
  msg = *m;
  stamp = rclcpp::Time(msg.header.stamp);
  received = true;
}

bool CommandData::ready() const
{
  return received;
}
