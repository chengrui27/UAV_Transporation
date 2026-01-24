#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <functional>

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

    cmd_sub_ = create_subscription<px4ctrl::msg::PositionCommand>(
        "/cmd", 10, std::bind(&Px4CtrlNode::cmd_cb, this, _1));

    auto period = std::chrono::duration<double>(1.0 / params_.ctrl_rate);
    timer_ = create_wall_timer(period, std::bind(&Px4CtrlNode::on_timer, this));
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_.feed(msg); }
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) { imu_.feed(msg); }
  void cmd_cb(const px4ctrl::msg::PositionCommand::SharedPtr msg) { cmd_.feed(msg); }

  void on_timer()
  {
    if (!odom_.received || !imu_.received || !cmd_.ready())
    {
      return;
    }

    DesiredState des;
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

    ControllerOutput out;
    controller_.update(des, odom_, imu_, out);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "ctrl out: thrust=%.3f, bodyrate=[%.3f, %.3f, %.3f], q=[%.3f, %.3f, %.3f, %.3f]",
        out.thrust,
        out.bodyrates.x(),
        out.bodyrates.y(),
        out.bodyrates.z(),
        out.q.w(),
        out.q.x(),
        out.q.y(),
        out.q.z());

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
  }

private:
  Params params_;
  Controller controller_;

  OdomData odom_;
  ImuData imu_;
  CommandData cmd_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4ctrl::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4CtrlNode>());
  rclcpp::shutdown();
  return 0;
}
