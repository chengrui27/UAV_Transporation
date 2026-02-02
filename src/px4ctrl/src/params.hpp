#pragma once

#include <rclcpp/rclcpp.hpp>

struct Params
{
  bool feedforward_thrust{false};
  bool publish_enabled{true};
  bool use_bodyrate{false};
  bool use_goal_pose{false};
  
  double ctrl_rate{100.0};
  double mass{1.535};
  double gra{9.81};

  double kp_x{1.0};
  double kp_y{1.0};
  double kp_z{1.0};
  double kv_x{1.5};
  double kv_y{1.5};
  double kv_z{1.0};
  double kvi_x{0.0};
  double kvi_y{0.0};
  double kvi_z{0.0};
  double ka{1.0};
  double kT{1.0};

  double katt_xy{1.0};
  double katt_z{1.0};

  double max_tilt_deg{35.0};
  double hover_thrust{0.708};
  double thrust_min{0.05};
  double thrust_max{0.95};

  void load(rclcpp::Node &node);
};
