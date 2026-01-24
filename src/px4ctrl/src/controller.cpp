#include "controller.hpp"

#include <cmath>
#include <vector>

namespace
{
/* 
 * vee map for so(3), maps a skew-symmetric matrix to a vector
 */
Eigen::Vector3d vee(const Eigen::Matrix3d &m)
{
  return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

/*
 * use yaw angle to get rotation matrix around z axis
 */
Eigen::Matrix3d rotz(double yaw)
{
  Eigen::Matrix3d r;
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  r << c, -s, 0.0,
       s,  c, 0.0,
       0.0, 0.0, 1.0;
  return r;
}

/*
 * convert rotation matrix to yaw-pitch-roll euler angles
 */
Eigen::Vector3d R_to_ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d ypr;
  ypr(0) = std::atan2(R(1, 0), R(0, 0));   // yaw
  ypr(1) = std::asin(-R(2, 0));            // pitch
  ypr(2) = std::atan2(R(2, 1), R(2, 2));   // roll
  return ypr;
}
} // use namespace to make these functions private to this file

/*
 * make read-only ptr to params
 */
void Controller::set_params(const Params &p)
{
  params_ = &p;
  int_e_v.setZero();
}

/*
 * use desired state and feedforward thrust to compute control output(attitude and thrust)
 */
void Controller::update(const DesiredState &des, const OdomData &odom, const ImuData &imu, ControllerOutput &out)
{
  (void)imu; // imu is not used in this controller
  out = ControllerOutput();
  if (!params_){
    return;
  }

  // reset integral if desired velocity is non-zero(uav is under active tracking)
  if(des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0){
    int_e_v.setZero();
  }

  const auto &p_msg = odom.msg.pose.pose.position;
  const auto &v_msg = odom.msg.twist.twist.linear;
  const auto &q_msg = odom.msg.pose.pose.orientation;

  Eigen::Vector3d p(p_msg.x, p_msg.y, p_msg.z);
  Eigen::Vector3d v(v_msg.x, v_msg.y, v_msg.z);
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  Eigen::Matrix3d R = q.toRotationMatrix(); 

  Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kv = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Ki = Eigen::Matrix3d::Zero();
  Kp(0, 0) = params_->kp_xy;
  Kp(1, 1) = params_->kp_xy;
  Kp(2, 2) = params_->kp_z;
  Kv(0, 0) = params_->kv_xy;
  Kv(1, 1) = params_->kv_xy;
  Kv(2, 2) = params_->kv_z;
  Ki(0, 0) = params_->kvi_xy;
  Ki(1, 1) = params_->kvi_xy;
  Ki(2, 2) = params_->kvi_z;

  // Step1: use quaternion to get current yaw
  double yaw_curr = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  Eigen::Matrix3d wRc = rotz(yaw_curr);  // world to current yaw frame
  Eigen::Matrix3d cRw = wRc.transpose();  // current yaw frame to world

  // Step2: compute position PD control and velocity PD control in current yaw frame
  Eigen::Vector3d e_p = des.p - p;
  Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;

  Eigen::Vector3d e_v = u_p + des.v - v;
  Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;  // proportional term of velocity control
  for(int i=0; i<3; i++){
    if(std::fabs(e_v[i]) < 0.2){
      int_e_v[i] += e_v[i] / params_->ctrl_rate;
    }  
  }
  const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
  Eigen::Vector3d u_v_i = wRc * Ki* cRw * int_e_v;  // integral term of velocity control
  for(int i=0; i<3; i++){
    if(u_v_i[i] > integration_output_limits[i]){
      u_v_i[i] = integration_output_limits[i];
    }else if(u_v_i[i] < -integration_output_limits[i]){
      u_v_i[i] = -integration_output_limits[i];
    }
  }
  Eigen::Vector3d u_v = u_v_p + u_v_i;  
  
  // Step3: add feedforward force or acceleration to compute desired force
  Eigen::Vector3d F_des;
  if(params_->feedforward_thrust){
    F_des = params_->kT * des.T + params_->mass * u_v;
  }else{
    Eigen::Vector3d a_cmd = u_v + params_->ka * des.a;
    F_des = params_->mass * (a_cmd + Eigen::Vector3d(0.0, 0.0, params_->gra));
  }

  // Step4: constrain desired force within limits
  double min_fz = 0.5 * params_->mass * params_->gra;
  double max_fz = 2.0 * params_->mass * params_->gra;
  if (F_des.z() < min_fz){
    F_des.z() = min_fz;
  }else if (F_des.z() > max_fz){
    F_des.z() = max_fz;
  }

  double max_tilt = std::tan(params_->max_tilt_deg * M_PI / 180.0);
  double max_xy = max_tilt * F_des.z();
  if (std::fabs(F_des.x()) > max_xy){
    F_des.x() = (F_des.x() > 0.0 ? 1.0 : -1.0) * max_xy;
  }
  if (std::fabs(F_des.y()) > max_xy){
    F_des.y() = (F_des.y() > 0.0 ? 1.0 : -1.0) * max_xy;
  }

  // Step5: compute desired attitude and thrust
  Eigen::Vector3d z_b_des = F_des / F_des.norm();

  Eigen::Vector3d y_c_des(-std::sin(des.yaw), std::cos(des.yaw), 0.0);
  Eigen::Vector3d x_b_des = y_c_des.cross(z_b_des);
  Eigen::Vector3d y_b_des = z_b_des.cross(x_b_des);

  // avoid singularity
  Eigen::Matrix3d R_des1;
  R_des1 << x_b_des, y_b_des, z_b_des;
  Eigen::Matrix3d R_des2;
  R_des2 << -x_b_des, -y_b_des, z_b_des;

  Eigen::Vector3d e1 = R_to_ypr(R_des1.transpose() * R);
  Eigen::Vector3d e2 = R_to_ypr(R_des2.transpose() * R);

  Eigen::Matrix3d R_des;

  if (e1.norm() < e2.norm()){
    R_des = R_des1;
  }else{
    R_des = R_des2;
  }

  if(params_->use_bodyrate){
    Eigen::Matrix3d R_err = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);  // define attitude error
    Eigen::Vector3d e_R = vee(R_err);

    out.bodyrates.x() = -params_->katt_xy * e_R.x();
    out.bodyrates.y() = -params_->katt_xy * e_R.y();
    out.bodyrates.z() = -params_->katt_z * e_R.z() + des.yaw_rate;
  }else{
    out.q = Eigen::Quaterniond(R_des);
  }

  Eigen::Vector3d z_b_curr = R.col(2);
  double u1 = F_des.dot(z_b_curr);  // map total thrust along current body z axis
  double thrust = (u1 / (params_->mass * params_->gra)) * params_->hover_thrust;
  if (thrust < params_->thrust_min)
  {
    thrust = params_->thrust_min;
  }
  if (thrust > params_->thrust_max)
  {
    thrust = params_->thrust_max;
  }

  out.thrust = thrust;
}
