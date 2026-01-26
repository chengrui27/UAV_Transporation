#pragma once

#include <rclcpp/rclcpp.hpp>

struct Params
{
  bool feedforward_thrust{true};
  bool publish_enabled{true};
  bool use_bodyrate{false};
  
  double ctrl_rate{100.0};
  double mass{1.535};
  double gra{9.81};

  double kp_xy{1.0};
  double kp_z{1.0};
  double kv_xy{1.5};
  double kv_z{1.5};
  double kvi_xy{0.1};
  double kvi_z{0.1};
  double ka{1.0};
  double kT{1.0};

  double katt_xy{6.0};
  double katt_z{4.0};

  double max_tilt_deg{35.0};
  double hover_thrust{0.638};
  double thrust_min{0.05};
  double thrust_max{0.95};

  void load(rclcpp::Node &node);
};
