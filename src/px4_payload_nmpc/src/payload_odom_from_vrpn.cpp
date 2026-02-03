#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>

class PayloadOdomFromVrpn : public rclcpp::Node {
public:
    PayloadOdomFromVrpn() : Node("payload_odom_from_vrpn") {
        quad_name_ = this->declare_parameter<std::string>("quad_name", "t1");
        payload_name_ = this->declare_parameter<std::string>("payload_name", "h1");

        std::string quad_pose_topic = "/vrpn/" + quad_name_ + "/pose";
        std::string quad_twist_topic = "/vrpn/" + quad_name_ + "/twist";
        std::string payload_pose_topic = "/vrpn/" + payload_name_ + "/pose";
        std::string payload_twist_topic = "/vrpn/" + payload_name_ + "/twist";

        RCLCPP_INFO(this->get_logger(),
                    "Using VRPN topics:\n  quad:    %s (pose), %s (twist)\n  payload: %s (pose), %s (twist)",
                    quad_pose_topic.c_str(), quad_twist_topic.c_str(),
                    payload_pose_topic.c_str(), payload_twist_topic.c_str());

        quad_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            quad_pose_topic, 10,
            std::bind(&PayloadOdomFromVrpn::quadPoseCallback, this, std::placeholders::_1));

        quad_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            quad_twist_topic, 10,
            std::bind(&PayloadOdomFromVrpn::quadTwistCallback, this, std::placeholders::_1));

        payload_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            payload_pose_topic, 10,
            std::bind(&PayloadOdomFromVrpn::payloadPoseCallback, this, std::placeholders::_1));

        payload_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            payload_twist_topic, 10,
            std::bind(&PayloadOdomFromVrpn::payloadTwistCallback, this, std::placeholders::_1));

        payload_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/payload/odom", 10);

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            10ms, std::bind(&PayloadOdomFromVrpn::publishOdom, this));
    }

private:
    void quadPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        quad_position_ << msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z;

        quad_orientation_.w() = msg->pose.orientation.w;
        quad_orientation_.x() = msg->pose.orientation.x;
        quad_orientation_.y() = msg->pose.orientation.y;
        quad_orientation_.z() = msg->pose.orientation.z;

        last_stamp_ = msg->header.stamp;
        quad_pose_received_ = true;
    }

    void quadTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        quad_linear_velocity_ << msg->twist.linear.x,
                                 msg->twist.linear.y,
                                 msg->twist.linear.z;
        last_stamp_ = msg->header.stamp;
        quad_twist_received_ = true;
    }

    void payloadPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        payload_position_ << msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z;

        last_stamp_ = msg->header.stamp;
        payload_pose_received_ = true;
    }

    void payloadTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        payload_linear_velocity_ << msg->twist.linear.x,
                                   msg->twist.linear.y,
                                   msg->twist.linear.z;
        last_stamp_ = msg->header.stamp;
        payload_twist_received_ = true;
    }

    void publishOdom() {
        if (!(quad_pose_received_ && quad_twist_received_ &&
              payload_pose_received_ && payload_twist_received_)) {
            return;
        }

        Eigen::Matrix3d R_world_to_body = quad_orientation_.toRotationMatrix().transpose();

        Eigen::Vector3d rel_pos_world = payload_position_ - quad_position_;
        Eigen::Vector3d rel_pos_body = R_world_to_body * rel_pos_world;

        Eigen::Vector3d rel_vel_world = payload_linear_velocity_ - quad_linear_velocity_;
        Eigen::Vector3d rel_vel_body = R_world_to_body * rel_vel_world;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_stamp_;
        odom_msg.header.frame_id = "base_link";
        odom_msg.child_frame_id = "payload";

        odom_msg.pose.pose.position.x = rel_pos_body.x();
        odom_msg.pose.pose.position.y = rel_pos_body.y();
        odom_msg.pose.pose.position.z = rel_pos_body.z();

        odom_msg.pose.pose.orientation.w = 1.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;

        odom_msg.twist.twist.linear.x = rel_vel_body.x();
        odom_msg.twist.twist.linear.y = rel_vel_body.y();
        odom_msg.twist.twist.linear.z = rel_vel_body.z();

        payload_odom_pub_->publish(odom_msg);
    }

    std::string quad_name_;
    std::string payload_name_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr quad_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr quad_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr payload_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr payload_twist_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr payload_odom_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d quad_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quad_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d quad_linear_velocity_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d payload_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d payload_linear_velocity_ = Eigen::Vector3d::Zero();

    builtin_interfaces::msg::Time last_stamp_;

    bool quad_pose_received_ = false;
    bool quad_twist_received_ = false;
    bool payload_pose_received_ = false;
    bool payload_twist_received_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PayloadOdomFromVrpn>());
    rclcpp::shutdown();
    return 0;
}

