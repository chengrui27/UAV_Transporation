#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <functional>
#include <cmath>

#include "params.hpp"
#include "input.hpp"
#include "controller.hpp"

using std::placeholders::_1;

class Px4CtrlNode : public rclcpp::Node
{
public:
  Px4CtrlNode() : Node("px4ctrl")
  {
    params_.load(*this);
    controller_.set_params(params_);

    ctrl_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);

    auto sensor_qos = rclcpp::SensorDataQoS();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", sensor_qos, std::bind(&Px4CtrlNode::odom_cb, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", sensor_qos, std::bind(&Px4CtrlNode::imu_cb, this, _1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, std::bind(&Px4CtrlNode::state_cb, this, _1));

    cmd_sub_ = create_subscription<px4ctrl::msg::PositionCommand>(
        "/cmd", 10, std::bind(&Px4CtrlNode::cmd_cb, this, _1));

    cmd_traj_sub_ = create_subscription<px4ctrl::msg::PositionCommandTrajectory>(
        "/cmd_traj", 10, std::bind(&Px4CtrlNode::cmd_traj_cb, this, _1));

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&Px4CtrlNode::goal_pose_cb, this, _1));

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    auto period = std::chrono::duration<double>(1.0 / params_.ctrl_rate);
    timer_ = create_wall_timer(period, std::bind(&Px4CtrlNode::on_timer, this));
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_.feed(msg); }
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) { imu_.feed(msg); }
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mavros_state_ = *msg;
    mavros_state_received_ = true;
  }
  void cmd_cb(const px4ctrl::msg::PositionCommand::SharedPtr msg) { cmd_.feed(msg); }
  void cmd_traj_cb(const px4ctrl::msg::PositionCommandTrajectory::SharedPtr msg) { traj_.feed(msg); }
  void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { goal_pose_.feed(msg); }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  /*
   * Fill desired state from trajectory based on current time
   */
  bool fillDesiredFromTrajectory(DesiredState &des)
  {
    if (!traj_.ready())
    {
      return false;
    }

    double dt = traj_.msg.dt;
    double t = (odom_.stamp.seconds() - traj_.stamp.seconds()) + (odom_.stamp.nanoseconds() - traj_.stamp.nanoseconds()) / 1000000000.0;
    t = t / 2.0;  // ros::Timer is two times faster than real time
    if (t < 0.0)
    {
      t = 0.0;
    }

    size_t idx = 0;
    double idx_f = 0.0;
    double prop = 0.0;
    if (dt > 1e-6)
    {
      idx_f = t / dt;
      idx = static_cast<size_t>(idx_f);
      prop = idx_f - static_cast<double>(idx);
      idx = idx + 1;
      RCLCPP_INFO(get_logger(), "traj idx = %zu, traj idx_f = %f, timer_cnt = %d", idx, idx_f, timer_cnt_);
    }
    if (idx >= traj_.msg.points.size())
    {
      idx = traj_.msg.points.size() - 1;
      const auto &pt = traj_.msg.points[idx];
      des.p.x() = pt.position.x;
      des.p.y() = pt.position.y;
      des.p.z() = pt.position.z;

      des.v.x() = 0.0;
      des.v.y() = 0.0;
      des.v.z() = 0.0;

      des.a.x() = 0.0;
      des.a.y() = 0.0;
      des.a.z() = 0.0;

      des.T.x() = 0.0;
      des.T.y() = 0.0;
      des.T.z() = pt.thrust.z;

      des.yaw = pt.yaw;
      des.yaw_rate = pt.yaw_rate;
    }
    else
    {
      const auto &pt = traj_.msg.points[idx];
      const auto &pt_last = traj_.msg.points[idx - 1];
      des.p.x() = pt.position.x * prop + pt_last.position.x * (1.0 - prop);
      des.p.y() = pt.position.y * prop + pt_last.position.y * (1.0 - prop);
      des.p.z() = pt.position.z * prop + pt_last.position.z * (1.0 - prop);

      des.v.x() = pt.velocity.x * prop + pt_last.velocity.x * (1.0 - prop);
      des.v.y() = pt.velocity.y * prop + pt_last.velocity.y * (1.0 - prop);
      des.v.z() = pt.velocity.z * prop + pt_last.velocity.z * (1.0 - prop);

      des.a.x() = pt.acceleration.x * prop + pt_last.acceleration.x * (1.0 - prop);
      des.a.y() = pt.acceleration.y * prop + pt_last.acceleration.y * (1.0 - prop);
      des.a.z() = pt.acceleration.z * prop + pt_last.acceleration.z * (1.0 - prop);

      des.T.x() = pt.thrust.x * prop + pt_last.thrust.x * (1.0 - prop);
      des.T.y() = pt.thrust.y * prop + pt_last.thrust.y * (1.0 - prop);
      des.T.z() = pt.thrust.z * prop + pt_last.thrust.z * (1.0 - prop);

      des.yaw = pt.yaw * prop + pt_last.yaw * (1.0 - prop);
      des.yaw_rate = pt.yaw_rate * prop + pt_last.yaw_rate * (1.0 - prop);
    }

    return true;
  }

  int timer_cnt_{0};
  void on_timer()
  {
    // 用于检查定时器调用频率
    // timer_cnt_++;
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "Px4CtrlNode on_timer called, cnt=%d",
    //     timer_cnt_);

    if (!odom_.received || !imu_.received)
    {
      return;
    }

    DesiredState des;
    if (params_.use_goal_pose)
    {
      if (!goal_pose_.ready())
      {
        return;
      }

      des.p.x() = goal_pose_.msg.pose.position.x;
      des.p.y() = goal_pose_.msg.pose.position.y;
      des.p.z() = 1.5;

      des.v.x() = 0.0;
      des.v.y() = 0.0;
      des.v.z() = 0.0;

      des.a.x() = 0.0;
      des.a.y() = 0.0;
      des.a.z() = 0.0;

      des.T.x() = 0.0;
      des.T.y() = 0.0;
      des.T.z() = 0.0;

      des.yaw = yaw_from_quaternion(goal_pose_.msg.pose.orientation);
      des.yaw_rate = 0.0;
    }
    else if (!fillDesiredFromTrajectory(des))
    {
      if (!cmd_.ready())
      {
        return;
      }

      des.p.x() = cmd_.msg.position.x;
      des.p.y() = cmd_.msg.position.y;
      des.p.z() = cmd_.msg.position.z;

      des.v.x() = cmd_.msg.velocity.x;
      des.v.y() = cmd_.msg.velocity.y;
      des.v.z() = cmd_.msg.velocity.z;

      des.a.x() = cmd_.msg.acceleration.x;
      des.a.y() = cmd_.msg.acceleration.y;
      des.a.z() = cmd_.msg.acceleration.z;

      des.T.x() = cmd_.msg.thrust.x;
      des.T.y() = cmd_.msg.thrust.y;
      des.T.z() = cmd_.msg.thrust.z;

      des.yaw = cmd_.msg.yaw;
      des.yaw_rate = cmd_.msg.yaw_rate;
    }

    ControllerOutput out;
    controller_.update(des, odom_, imu_, out);

    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     500,
    //     "ctrl out: des=[%.3f, %.3f, %.3f], cur=[%.3f, %.3f, %.3f], thrust=%.3f, bodyrate=[%.3f, %.3f, %.3f], q=[%.3f, %.3f, %.3f, %.3f]",
    //     des.p.x(),
    //     des.p.y(),
    //     des.p.z(),
    //     odom_.msg.pose.pose.position.x,
    //     odom_.msg.pose.pose.position.y,
    //     odom_.msg.pose.pose.position.z,
    //     out.thrust,
    //     out.bodyrates.x(),
    //     out.bodyrates.y(),
    //     out.bodyrates.z(),
    //     out.q.w(),
    //     out.q.x(),
    //     out.q.y(),
    //     out.q.z());

    if (!params_.publish_enabled)
    {
      return;
    }

    mavros_msgs::msg::AttitudeTarget msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";

    if (params_.use_bodyrate)
    {
      msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
      msg.body_rate.x = out.bodyrates.x();
      msg.body_rate.y = out.bodyrates.y();
      msg.body_rate.z = out.bodyrates.z();
      msg.thrust = out.thrust;
    }
    else
    {
      msg.type_mask =
          mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
          mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
          mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

      msg.orientation.x = out.q.x();
      msg.orientation.y = out.q.y();
      msg.orientation.z = out.q.z();
      msg.orientation.w = out.q.w();
      msg.thrust = out.thrust;
    }

    ctrl_pub_->publish(msg);
    published_setpoint_count_++;

    // 自动切换offboard模式
    constexpr size_t kOffboardSetpointCount = 100;
    if (!mavros_state_received_)
    {
      return;
    }
    if (mavros_state_.mode == "OFFBOARD")
    {
      return;
    }
    if (published_setpoint_count_ < kOffboardSetpointCount)
    {
      return;
    }
    if (!set_mode_client_->service_is_ready())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "SetMode service not available");
      return;
    }
    auto now_time = now();
    if ((now_time - last_offboard_request_).seconds() < 1.0)
    {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";
    set_mode_client_->async_send_request(request);
    last_offboard_request_ = now_time;
  }

private:
  Params params_;
  Controller controller_;

  OdomData odom_;
  ImuData imu_;
  CommandData cmd_;
  TrajectoryData traj_;
  GoalPoseData goal_pose_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<px4ctrl::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<px4ctrl::msg::PositionCommandTrajectory>::SharedPtr cmd_traj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State mavros_state_;
  bool mavros_state_received_{false};
  size_t published_setpoint_count_{0};
  rclcpp::Time last_offboard_request_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4CtrlNode>());
  rclcpp::shutdown();
  return 0;
}
