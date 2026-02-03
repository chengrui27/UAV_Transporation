/**
 * @file offboard_control.cpp
 * @brief PX4 Offboard Control Node using MAVROS
 *
 * This node demonstrates how to control a PX4 drone in offboard mode using MAVROS with ROS2.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node"), offboard_setpoint_counter_(0)
    {
        // Configure QoS profile
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // Publishers
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        // Subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_profile,
            std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));

        // Service clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // Wait for services
        while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arming service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Arming service not available, waiting...");
        }

        while (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set_mode service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Set mode service not available, waiting...");
        }

        // Initialize pose setpoint
        pose_.pose.position.x = 1.0;
        pose_.pose.position.y = 1.0;
        pose_.pose.position.z = 4.0;  // 2 meters altitude

        // Create timers
        // High rate timer for publishing setpoints (100 Hz - required for offboard mode)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&OffboardControl::timer_callback, this));

        // State machine timer (2 Hz)
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&OffboardControl::state_machine_callback, this));

        RCLCPP_INFO(this->get_logger(), "Offboard control node initialized");
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void timer_callback()
    {
        // Initialize trajectory start time when armed
        if (current_state_.armed && !trajectory_started_) {
            trajectory_start_time_ = this->now();
            trajectory_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting circular trajectory (radius=%.2f m, angular_vel=%.2f rad/s)",
                       circle_radius_, circle_angular_velocity_);
        }

        // Update circular trajectory if started
        if (trajectory_started_) {
            double t = (this->now() - trajectory_start_time_).seconds();
            // update_circular_trajectory(t);
        }

        // Publish setpoints at high rate (required for offboard mode)
        pose_.header.stamp = this->now();
        pose_.header.frame_id = "map";
        local_pos_pub_->publish(pose_);
    }

    void state_machine_callback()
    {
        // Send a few setpoints before starting offboard mode
        if (offboard_setpoint_counter_ < 10) {
            offboard_setpoint_counter_++;
            return;
        }

        // Switch to offboard mode
        if (current_state_.mode != "OFFBOARD") {
            set_mode_async("OFFBOARD");
        } else {
            // Arm the vehicle
            if (!current_state_.armed) {
                arm_async(true);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Offboard mode active - Target: x=%.2f, y=%.2f, z=%.2f",
                    pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
            }
        }
    }

    void set_mode_async(const std::string &mode)
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        auto result_future = set_mode_client_->async_send_request(request,
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                auto result = future.get();
                if (result->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Mode set to %s", mode.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to set mode to %s", mode.c_str());
                }
            });
    }

    void arm_async(bool arm_state)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_state;

        auto result_future = arming_client_->async_send_request(request,
            [this, arm_state](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle %s", arm_state ? "armed" : "disarmed");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to %s vehicle", arm_state ? "arm" : "disarm");
                }
            });
    }

    void set_target_position(double x, double y, double z)
    {
        pose_.pose.position.x = x;
        pose_.pose.position.y = y;
        pose_.pose.position.z = z;
        RCLCPP_INFO(this->get_logger(), "New target position set: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    }

    /**
     * @brief Update pose with circular trajectory
     * @param t Current time in seconds
     */
    void update_circular_trajectory(double t)
    {
        // Calculate phase angle
        double theta = circle_angular_velocity_ * t;

        // Calculate position on circle (ENU frame)
        pose_.pose.position.x = circle_center_x_ + circle_radius_ * std::cos(theta);
        pose_.pose.position.y = circle_center_y_ + circle_radius_ * std::sin(theta);
        pose_.pose.position.z = circle_height_;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    // Service clients
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    // Variables
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped pose_;
    int offboard_setpoint_counter_;

    // Circular trajectory parameters (same as NMPC controller)
    const double circle_radius_ = 2.0;          // Circle radius (m)
    const double circle_angular_velocity_ = 1.0; // Angular velocity (rad/s)
    const double circle_center_x_ = 0.0;        // Circle center X (m, ENU)
    const double circle_center_y_ = 0.0;        // Circle center Y (m, ENU)
    const double circle_height_ = 4.0;          // Circle height (m)

    // Time tracking
    rclcpp::Time trajectory_start_time_;
    bool trajectory_started_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
