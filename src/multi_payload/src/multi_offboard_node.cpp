/**
 * @file multi_offboard_node.cpp
 * @brief Multi-UAV offboard position takeoff controller using MAVROS (ROS2)
 */

#include <array>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using namespace std::chrono_literals;

class MultiOffboardNode : public rclcpp::Node
{
public:
    static constexpr std::size_t kNumUavs = 3;

    MultiOffboardNode()
    : Node("multi_offboard_node")
    {
        uav_names_ = {"uav1", "uav2", "uav3"};
        target_x_ = {-1.0, 0.0, 0.0};
        target_y_ = {0.0, -1.0, 1.0};

        auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        auto qos_pose = rclcpp::SensorDataQoS();

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            has_state_[i] = false;
            has_pose_[i] = false;
            target_initialized_[i] = false;
            offboard_setpoint_counter_[i] = 0;

            const std::string ns = "/" + uav_names_[i];

            local_pos_pub_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                ns + "/setpoint_position/local", 10);

            state_sub_[i] = this->create_subscription<mavros_msgs::msg::State>(
                ns + "/state",
                qos_state,
                [this, i](const mavros_msgs::msg::State::SharedPtr msg)
                {
                    current_state_[i] = *msg;
                    has_state_[i] = true;
                });

            pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                ns + "/local_position/pose",
                qos_pose,
                [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    current_pose_[i] = *msg;
                    has_pose_[i] = true;

                    if (!target_initialized_[i]) {
                        target_pose_[i] = *msg;
                        target_pose_[i].pose.position.x = target_x_[i];
                        target_pose_[i].pose.position.y = target_y_[i];
                        target_pose_[i].pose.position.z = msg->pose.position.z + takeoff_height_;
                        target_initialized_[i] = true;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "[%s] Takeoff target initialized at x=%.2f y=%.2f z=%.2f",
                            uav_names_[i].c_str(),
                            target_pose_[i].pose.position.x,
                            target_pose_[i].pose.position.y,
                            target_pose_[i].pose.position.z);
                    }
                });

            arming_client_[i] = this->create_client<mavros_msgs::srv::CommandBool>(
                ns + "/cmd/arming");
            set_mode_client_[i] = this->create_client<mavros_msgs::srv::SetMode>(
                ns + "/set_mode");
        }

        // Timer for high-rate setpoint publishing (50 Hz)
        setpoint_timer_ = this->create_wall_timer(
            20ms,
            std::bind(&MultiOffboardNode::publish_setpoints, this));

        // Timer for offboard / arming state machine (2 Hz)
        state_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MultiOffboardNode::state_machine_step, this));

        RCLCPP_INFO(this->get_logger(), "MultiOffboardNode initialized for %zu UAVs", kNumUavs);
    }

private:
    void publish_setpoints()
    {
        const auto now = this->now();

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!target_initialized_[i]) {
                continue;
            }

            target_pose_[i].header.stamp = now;
            target_pose_[i].header.frame_id = "map";
            local_pos_pub_[i]->publish(target_pose_[i]);
        }
    }

    void state_machine_step()
    {
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!has_state_[i] || !target_initialized_[i]) {
                continue;
            }

            if (!arming_client_[i]->service_is_ready() ||
                !set_mode_client_[i]->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "[%s] Services not ready yet",
                    uav_names_[i].c_str());
                continue;
            }

            // Send some setpoints before switching to offboard
            if (offboard_setpoint_counter_[i] < offboard_setpoint_threshold_) {
                ++offboard_setpoint_counter_[i];
                continue;
            }

            // Ensure OFFBOARD mode
            if (current_state_[i].mode != "OFFBOARD") {
                set_mode_async(i, "OFFBOARD");
                continue;
            }

            // Arm once in offboard
            if (!current_state_[i].armed) {
                arm_async(i, true);
                continue;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[%s] OFFBOARD active, holding at x=%.2f y=%.2f z=%.2f",
                uav_names_[i].c_str(),
                target_pose_[i].pose.position.x,
                target_pose_[i].pose.position.y,
                target_pose_[i].pose.position.z);
        }
    }

    void set_mode_async(std::size_t index, const std::string &mode)
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        auto client = set_mode_client_[index];

        client->async_send_request(
            request,
            [this, index, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                auto result = future.get();
                if (result && result->mode_sent) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[%s] Mode set to %s",
                        uav_names_[index].c_str(),
                        mode.c_str());
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[%s] Failed to set mode to %s",
                        uav_names_[index].c_str(),
                        mode.c_str());
                }
            });
    }

    void arm_async(std::size_t index, bool arm_state)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_state;

        auto client = arming_client_[index];

        client->async_send_request(
            request,
            [this, index, arm_state](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
            {
                auto result = future.get();
                if (result && result->success) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[%s] Vehicle %s",
                        uav_names_[index].c_str(),
                        arm_state ? "armed" : "disarmed");
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[%s] Failed to %s vehicle",
                        uav_names_[index].c_str(),
                        arm_state ? "arm" : "disarm");
                }
            });
    }

    // Publishers
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, kNumUavs> local_pos_pub_;

    // Subscribers
    std::array<rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr, kNumUavs> state_sub_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, kNumUavs> pose_sub_;

    // Service clients
    std::array<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr, kNumUavs> arming_client_;
    std::array<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr, kNumUavs> set_mode_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    // State
    std::array<std::string, kNumUavs> uav_names_;
    std::array<double, kNumUavs> target_x_;
    std::array<double, kNumUavs> target_y_;
    std::array<mavros_msgs::msg::State, kNumUavs> current_state_;
    std::array<geometry_msgs::msg::PoseStamped, kNumUavs> current_pose_;
    std::array<geometry_msgs::msg::PoseStamped, kNumUavs> target_pose_;

    std::array<bool, kNumUavs> has_state_;
    std::array<bool, kNumUavs> has_pose_;
    std::array<bool, kNumUavs> target_initialized_;
    std::array<int, kNumUavs> offboard_setpoint_counter_;

    const double takeoff_height_ = 2.0;
    const int offboard_setpoint_threshold_ = 20;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiOffboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
