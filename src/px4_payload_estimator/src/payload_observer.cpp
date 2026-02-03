#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <cmath>

class PayloadObserver : public rclcpp::Node
{
public:
    PayloadObserver() : Node("payload_observer")
    {
        // Parameters
        this->declare_parameter("m_uav", 2.059);
        this->declare_parameter("l_cable", 1.0);
        this->declare_parameter("max_thrust", 40.6673);

        m_uav_ = this->get_parameter("m_uav").as_double();
        l_cable_ = this->get_parameter("l_cable").as_double();
        max_thrust_ = this->get_parameter("max_thrust").as_double();

        // Publishers
        // Use default QoS (Reliable) to ensure compatibility with standard subscribers
        rclcpp::QoS qos(10);

        pub_measured_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slung_load/measured_pose_only", qos);
        pub_observed_angles_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/slung_load/observed_angles", qos);
        pub_true_angles_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/slung_load/true_angles", qos);

        // Subscribers
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            std::bind(&PayloadObserver::imu_cb, this, std::placeholders::_1));
        sub_thrust_ = this->create_subscription<mavros_msgs::msg::VfrHud>(
            "/mavros/vfr_hud", sensor_qos,
            std::bind(&PayloadObserver::thrust_cb, this, std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/payload/odom", sensor_qos,
            std::bind(&PayloadObserver::odom_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Payload Observer Initialized");
    }

private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        double phi = std::atan2(std::sqrt(x * x + y * y), z);
        double theta = std::atan2(y, x);

        geometry_msgs::msg::PointStamped angle_msg;
        angle_msg.header = msg->header;
        angle_msg.point.x = phi;
        angle_msg.point.y = theta;
        angle_msg.point.z = 0.0;
        pub_true_angles_->publish(angle_msg);
    }

    void thrust_cb(const mavros_msgs::msg::VfrHud::SharedPtr msg)
    {
        thrust_ = msg->throttle * max_thrust_;
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        accel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        calculate_and_publish(msg->header);
    }

    void calculate_and_publish(const std_msgs::msg::Header &header)
    {
        double ax = accel_.x();
        double ay = accel_.y();
        double az = accel_.z();

        double accel_xy_norm = std::sqrt(ax * ax + ay * ay);
        double term_z = az - thrust_ / m_uav_;

        double z_phi = std::atan2(accel_xy_norm, term_z);
        double z_theta = std::atan2(ay, ax);

        // Publish observed angles
        geometry_msgs::msg::PointStamped angle_msg;
        angle_msg.header = header;
        angle_msg.point.x = z_phi;
        angle_msg.point.y = z_theta;
        angle_msg.point.z = 0.0;
        pub_observed_angles_->publish(angle_msg);

        // Calculate position
        Eigen::Vector3d n = get_vectors(z_phi, z_theta);
        Eigen::Vector3d p_body = l_cable_ * n;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = "base_link";

        pose_msg.pose.position.x = p_body.x();
        pose_msg.pose.position.y = p_body.y();
        pose_msg.pose.position.z = p_body.z();
        pose_msg.pose.orientation.w = 1.0;

        pub_measured_pose_->publish(pose_msg);
    }

    Eigen::Vector3d get_vectors(double phi, double theta)
    {
        double s_phi = std::sin(phi);
        double c_phi = std::cos(phi);
        double s_theta = std::sin(theta);
        double c_theta = std::cos(theta);

        return Eigen::Vector3d(s_phi * c_theta, s_phi * s_theta, c_phi);
    }

    // Member variables
    double m_uav_;
    double l_cable_;
    double max_thrust_;
    double thrust_ = 0.0;
    Eigen::Vector3d accel_ = Eigen::Vector3d::Zero();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_observed_angles_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_true_angles_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr sub_thrust_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PayloadObserver>());
    rclcpp::shutdown();
    return 0;
}
