#include "params.hpp"

void Params::load(rclcpp::Node &node)
{
  feedforward_thrust = node.declare_parameter<bool>("feedforward_thrust", feedforward_thrust);
  publish_enabled = node.declare_parameter<bool>("publish_enabled", publish_enabled);
  use_bodyrate = node.declare_parameter<bool>("use_bodyrate", use_bodyrate);
  use_goal_pose = node.declare_parameter<bool>("use_goal_pose", use_goal_pose);
  
  ctrl_rate = node.declare_parameter<double>("ctrl_rate", ctrl_rate);
  mass = node.declare_parameter<double>("mass", mass);
  gra = node.declare_parameter<double>("gra", gra);

  kp_x = node.declare_parameter<double>("kp_x", kp_x);
  kp_y = node.declare_parameter<double>("kp_y", kp_y);
  kp_z = node.declare_parameter<double>("kp_z", kp_z);
  kv_x = node.declare_parameter<double>("kv_x", kv_x);
  kv_y = node.declare_parameter<double>("kv_y", kv_y);
  kv_z = node.declare_parameter<double>("kv_z", kv_z);
  kvi_x = node.declare_parameter<double>("kvi_x", kvi_x);
  kvi_y = node.declare_parameter<double>("kvi_y", kvi_y);
  kvi_z = node.declare_parameter<double>("kvi_z", kvi_z);
  ka = node.declare_parameter<double>("ka", ka);
  kT = node.declare_parameter<double>("kT", kT);

  katt_xy = node.declare_parameter<double>("katt_xy", katt_xy);
  katt_z = node.declare_parameter<double>("katt_z", katt_z);

  max_tilt_deg = node.declare_parameter<double>("max_tilt_deg", max_tilt_deg);
  hover_thrust = node.declare_parameter<double>("hover_thrust", hover_thrust);
  thrust_min = node.declare_parameter<double>("thrust_min", thrust_min);
  thrust_max = node.declare_parameter<double>("thrust_max", thrust_max);
}
