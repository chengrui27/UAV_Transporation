#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "params.hpp"
#include "input.hpp"

struct DesiredState
{
  Eigen::Vector3d p{0.0, 0.0, 0.0};
  Eigen::Vector3d v{0.0, 0.0, 0.0};
  Eigen::Vector3d a{0.0, 0.0, 0.0};
  Eigen::Vector3d T{0.0, 0.0, 0.0};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct ControllerOutput
{
  Eigen::Quaterniond q{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d bodyrates{0.0, 0.0, 0.0};
  double thrust{0.0};
};

class Controller
{
public:
  void set_params(const Params &p);
  void update(const DesiredState &des, const OdomData &odom, const ImuData &imu, ControllerOutput &out);

private:
  const Params *params_{nullptr};
  Eigen::Vector3d int_e_v{0.0, 0.0, 0.0};
};
