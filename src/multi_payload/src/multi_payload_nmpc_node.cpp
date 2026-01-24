/**
 * @file multi_payload_nmpc_node.cpp
 * @brief Multi-UAV payload NMPC controller using acados and MAVROS (ROS2)
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <Eigen/Dense>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_multi_payload.h"
}

using namespace std::chrono_literals;

class MultiPayloadNmpcNode : public rclcpp::Node
{
public:
    static constexpr std::size_t kNumUavs = 3;
    static constexpr int kLoadStateSize = 13;
    static constexpr int kCableStateSize = 6;
    static constexpr int kQuadStateSize = 11;
    static constexpr int kCableStateStart = kLoadStateSize;
    static constexpr int kQuadStateStart = kCableStateStart + static_cast<int>(kNumUavs) * kCableStateSize;

    MultiPayloadNmpcNode()
    : Node("multi_payload_nmpc")
    {
        this->declare_parameter("uav_names", std::vector<std::string>{"uav1", "uav2", "uav3"});
        this->declare_parameter("control_frequency", 100.0);
        this->declare_parameter("nmpc_frequency", 10.0);
        this->declare_parameter("takeoff_height", 2.0);
        this->declare_parameter("takeoff_tolerance", 0.2);
        this->declare_parameter("offboard_setpoint_threshold", 20);
        this->declare_parameter("takeoff_target_x", std::vector<double>{2.1, -0.3, -0.3});
        this->declare_parameter("takeoff_target_y", std::vector<double>{0.0, 1.9, -1.9});
        this->declare_parameter("takeoff_target_z", std::vector<double>{2.0, 2.0, 2.0});
        this->declare_parameter("cable_length", 1.5);
        this->declare_parameter("quad_mass", 1.535);
        this->declare_parameter("payload_mass", 1.0);
        this->declare_parameter("max_thrust", 23.59);
        this->declare_parameter("omega_filter_alpha", 0.3);
        this->declare_parameter("acados_print_level", 0);
        this->declare_parameter("acados_rti_log_residuals", 0);

        uav_names_ = this->get_parameter("uav_names").as_string_array();
        if (uav_names_.size() != kNumUavs) {
            RCLCPP_WARN(this->get_logger(),
                "uav_names size (%zu) != %zu, using default names",
                uav_names_.size(), kNumUavs);
            uav_names_ = {"uav1", "uav2", "uav3"};
        }

        control_frequency_ = this->get_parameter("control_frequency").as_double();
        nmpc_frequency_ = this->get_parameter("nmpc_frequency").as_double();
        takeoff_height_ = this->get_parameter("takeoff_height").as_double();
        takeoff_tolerance_ = this->get_parameter("takeoff_tolerance").as_double();
        offboard_setpoint_threshold_ = this->get_parameter("offboard_setpoint_threshold").as_int();
        takeoff_target_x_ = this->get_parameter("takeoff_target_x").as_double_array();
        takeoff_target_y_ = this->get_parameter("takeoff_target_y").as_double_array();
        takeoff_target_z_ = this->get_parameter("takeoff_target_z").as_double_array();
        cable_length_ = this->get_parameter("cable_length").as_double();
        quad_mass_ = this->get_parameter("quad_mass").as_double();
        payload_mass_ = this->get_parameter("payload_mass").as_double();
        max_thrust_ = this->get_parameter("max_thrust").as_double();
        omega_filter_alpha_ = this->get_parameter("omega_filter_alpha").as_double();
        acados_print_level_ = this->get_parameter("acados_print_level").as_int();
        acados_rti_log_residuals_ = this->get_parameter("acados_rti_log_residuals").as_int();
        gravity_ = 9.81;

        r_b_[0] = Eigen::Vector3d(0.6, 0.0, 0.0);
        r_b_[1] = Eigen::Vector3d(-0.3, 0.4, 0.0);
        r_b_[2] = Eigen::Vector3d(-0.3, -0.4, 0.0);

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            uav_pose_received_[i] = false;
            uav_velocity_received_[i] = false;
            uav_state_received_[i] = false;
            takeoff_target_initialized_[i] = false;
            offboard_setpoint_counter_[i] = 0;
            cable_prev_valid_[i] = false;
            cable_omega_filtered_[i].setZero();
            current_body_rate_[i].setZero();
            current_thrust_[i] = 0.0;
            current_attitude_[i] = Eigen::Quaterniond::Identity();
        }

        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        auto qos_sensor = rclcpp::SensorDataQoS();

        payload_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/payload/odom", 10,
            std::bind(&MultiPayloadNmpcNode::payloadOdomCallback, this, std::placeholders::_1));

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&MultiPayloadNmpcNode::goalPoseCallback, this, std::placeholders::_1));

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            const std::string ns = "/" + uav_names_[i];

            uav_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                ns + "/local_position/pose",
                qos_sensor,
                [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    uav_position_[i] << msg->pose.position.x,
                                         msg->pose.position.y,
                                         msg->pose.position.z;

                    uav_attitude_[i].w() = msg->pose.orientation.w;
                    uav_attitude_[i].x() = msg->pose.orientation.x;
                    uav_attitude_[i].y() = msg->pose.orientation.y;
                    uav_attitude_[i].z() = msg->pose.orientation.z;
                    uav_attitude_[i] = normalizeQuaternionSign(uav_attitude_[i]);

                    uav_pose_received_[i] = true;

                    if (!takeoff_target_initialized_[i]) {
                        double target_x = msg->pose.position.x;
                        double target_y = msg->pose.position.y;
                        double target_z = msg->pose.position.z + takeoff_height_;
                        if (takeoff_target_x_.size() == kNumUavs) {
                            target_x = takeoff_target_x_[i];
                        }
                        if (takeoff_target_y_.size() == kNumUavs) {
                            target_y = takeoff_target_y_[i];
                        }
                        if (takeoff_target_z_.size() == kNumUavs) {
                            target_z = takeoff_target_z_[i];
                        }

                        takeoff_target_[i].pose.position.x = target_x;
                        takeoff_target_[i].pose.position.y = target_y;
                        takeoff_target_[i].pose.position.z = target_z;
                        takeoff_target_[i].pose.orientation = msg->pose.orientation;
                        takeoff_target_[i].header.frame_id = "map";

                        takeoff_target_initialized_[i] = true;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "[%s] Takeoff target: x=%.2f y=%.2f z=%.2f",
                            uav_names_[i].c_str(),
                            takeoff_target_[i].pose.position.x,
                            takeoff_target_[i].pose.position.y,
                            takeoff_target_[i].pose.position.z);
                    }
                });

            uav_velocity_sub_[i] = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                ns + "/local_position/velocity_local",
                qos_sensor,
                [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
                {
                    uav_velocity_[i] << msg->twist.linear.x,
                                         msg->twist.linear.y,
                                         msg->twist.linear.z;
                    uav_velocity_received_[i] = true;
                });

            uav_state_sub_[i] = this->create_subscription<mavros_msgs::msg::State>(
                ns + "/state",
                qos_best_effort,
                [this, i](const mavros_msgs::msg::State::SharedPtr msg)
                {
                    uav_state_[i] = *msg;
                    uav_state_received_[i] = true;
                });

            setpoint_pos_pub_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                ns + "/setpoint_position/local", 10);

            attitude_pub_[i] = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
                ns + "/setpoint_raw/attitude", 10);

            arming_client_[i] = this->create_client<mavros_msgs::srv::CommandBool>(
                ns + "/cmd/arming");

            set_mode_client_[i] = this->create_client<mavros_msgs::srv::SetMode>(
                ns + "/set_mode");
        }

        payload_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/multi_payload/payload_predicted_path", 10);
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            const std::string topic = "/multi_payload/" + uav_names_[i] + "/predicted_path";
            uav_path_pub_[i] = this->create_publisher<nav_msgs::msg::Path>(topic, 10);
        }

        initializeAcadosSolver();

        auto timer_period = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / control_frequency_));
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&MultiPayloadNmpcNode::controlTimerCallback, this));

        auto solve_timer_period = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / nmpc_frequency_));
        solve_timer_ = this->create_wall_timer(
            solve_timer_period,
            std::bind(&MultiPayloadNmpcNode::solveTimerCallback, this));

        path_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MultiPayloadNmpcNode::publishPredictedPaths, this));

        state_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MultiPayloadNmpcNode::stateMachineTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Multi-payload NMPC node initialized");
    }

    ~MultiPayloadNmpcNode()
    {
        cleanupAcadosSolver();
    }

private:
    void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        payload_position_ << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z;
        payload_velocity_ << msg->twist.twist.linear.x,
                              msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z;
        payload_attitude_.w() = msg->pose.pose.orientation.w;
        payload_attitude_.x() = msg->pose.pose.orientation.x;
        payload_attitude_.y() = msg->pose.pose.orientation.y;
        payload_attitude_.z() = msg->pose.pose.orientation.z;
        payload_attitude_ = normalizeQuaternionSign(payload_attitude_);

        payload_angular_velocity_ << msg->twist.twist.angular.x,
                                     msg->twist.twist.angular.y,
                                     msg->twist.twist.angular.z;
        payload_received_ = true;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_payload_position_ << msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z;
        goal_payload_position_[2] = 0.5;  // keep constant altitude for now

        goal_payload_attitude_.w() = msg->pose.orientation.w;
        goal_payload_attitude_.x() = msg->pose.orientation.x;
        goal_payload_attitude_.y() = msg->pose.orientation.y;
        goal_payload_attitude_.z() = msg->pose.orientation.z;
        goal_payload_attitude_ = normalizeQuaternionSign(goal_payload_attitude_);
        goal_pose_received_ = true;

        RCLCPP_INFO(this->get_logger(),
            "New payload goal: [%.2f, %.2f, %.2f]",
            goal_payload_position_(0),
            goal_payload_position_(1),
            goal_payload_position_(2));

        if (solver_initialized_) {
            have_prediction_ = false;
            nmpc_active_ = false;
            force_trajectory_reset_ = true;
            multi_payload_acados_reset(acados_ocp_capsule_, 1);
            setQpWarmStart(0);
            RCLCPP_INFO(this->get_logger(), "NMPC reset after new goal");
        }
    }

    void controlTimerCallback()
    {
        if (!payload_received_) {
            return;
        }

        if (!allUavPoseReady()) {
            return;
        }

        if (!allUavVelocityReady()) {
            return;
        }

        if (!takeoff_complete_) {
            publishTakeoffSetpoints();
            return;
        }

        if (nmpc_active_) {
            publishAttitudeSetpoints();
        } else {
            publishTakeoffSetpoints();
        }
        // publishTakeoffSetpoints();
    }

    void solveTimerCallback()
    {
        if (!payload_received_) {
            return;
        }

        if (!allUavPoseReady()) {
            return;
        }

        if (!allUavVelocityReady()) {
            return;
        }

        if (!takeoff_complete_) {
            return;
        }

        computeCableStates();
        auto solve_start = std::chrono::high_resolution_clock::now();
        bool solver_ok = solveNmpc();
        auto solve_end = std::chrono::high_resolution_clock::now();
        double solve_time_ms = std::chrono::duration<double, std::milli>(
            solve_end - solve_start).count();
        if (solver_ok && !nmpc_active_) {
            nmpc_active_ = true;
            RCLCPP_INFO(this->get_logger(), "NMPC solve OK, switching to attitude control");
        }
        if (solver_ok) {
            solve_count_++;
            solve_success_count_++;
            avg_solve_time_ms_ =
                (avg_solve_time_ms_ * (solve_count_ - 1) + solve_time_ms) / solve_count_;
        } else {
            solve_count_++;
            avg_solve_time_ms_ =
                (avg_solve_time_ms_ * (solve_count_ - 1) + solve_time_ms) / solve_count_;
        }

        // if (solve_count_ % 20 == 0) {
        //     double success_rate = 100.0 * solve_success_count_ / solve_count_;
        //     RCLCPP_INFO(this->get_logger(),
        //         "NMPC Stats: Success rate: %.1f%%, Avg solve time: %.2f ms",
        //         success_rate, avg_solve_time_ms_);
        // }
    }

    void stateMachineTimerCallback()
    {
        if (!payload_received_ || !allUavPoseReady() || !allUavStateReady()) {
            return;
        }

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!takeoff_target_initialized_[i]) {
                return;
            }
        }

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!arming_client_[i]->service_is_ready() ||
                !set_mode_client_[i]->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "[%s] Services not ready yet",
                    uav_names_[i].c_str());
                return;
            }
        }

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (offboard_setpoint_counter_[i] < offboard_setpoint_threshold_) {
                continue;
            }

            if (uav_state_[i].mode != "OFFBOARD") {
                setModeAsync(i, "OFFBOARD");
                continue;
            }

            if (!uav_state_[i].armed) {
                armAsync(i, true);
                continue;
            }
        }

        if (!takeoff_complete_ && allUavsReadyForNmpc()) {
            takeoff_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff complete, start NMPC solving");
        }

    }

    bool allUavPoseReady() const
    {
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!uav_pose_received_[i]) {
                return false;
            }
        }
        return true;
    }

    bool allUavVelocityReady() const
    {
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!uav_velocity_received_[i]) {
                return false;
            }
        }
        return true;
    }

    bool allUavStateReady() const
    {
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!uav_state_received_[i]) {
                return false;
            }
        }
        return true;
    }

    bool allUavsReadyForNmpc() const
    {
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            const auto &target = takeoff_target_[i].pose.position;
            double dx = uav_position_[i].x() - target.x;
            double dy = uav_position_[i].y() - target.y;
            double dz = uav_position_[i].z() - target.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > takeoff_tolerance_) {
                return false;
            }
            if (uav_state_[i].mode != "OFFBOARD" || !uav_state_[i].armed) {
                return false;
            }
        }
        return true;
    }

    void computeCableStates()
    {
        Eigen::Matrix3d R_L = payload_attitude_.normalized().toRotationMatrix();
        rclcpp::Time now = this->now();
        double nominal_dt = 1.0 / std::max(1.0, nmpc_frequency_);

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            Eigen::Vector3d attach = payload_position_ + R_L * r_b_[i];
            Eigen::Vector3d cable_vec = attach - uav_position_[i];
            double length = cable_vec.norm();

            Eigen::Vector3d q_dir(0.0, 0.0, -1.0);
            if (length > 1e-6) {
                q_dir = cable_vec / length;
            }

            Eigen::Vector3d omega_raw(0.0, 0.0, 0.0);
            if (cable_prev_valid_[i]) {
                double dt = (now - cable_prev_time_[i]).seconds();
                if (dt < 1e-6) {
                    dt = nominal_dt;
                }
                Eigen::Vector3d q_dot = (q_dir - cable_prev_dir_[i]) / dt;
                omega_raw = q_dir.cross(q_dot);
            }

            if (cable_prev_valid_[i]) {
                cable_omega_filtered_[i] =
                    omega_filter_alpha_ * omega_raw +
                    (1.0 - omega_filter_alpha_) * cable_omega_filtered_[i];
            } else {
                cable_omega_filtered_[i] = omega_raw;
            }

            cable_direction_[i] = q_dir;
            cable_angular_velocity_[i] = cable_omega_filtered_[i];

            cable_prev_dir_[i] = q_dir;
            cable_prev_time_[i] = now;
            cable_prev_valid_[i] = true;
        }
    }

    bool solveNmpc()
    {
        if (!solver_initialized_) {
            return false;
        }

        double x0[MULTI_PAYLOAD_NX];
        if (have_prediction_) {
            for (int i = 0; i < MULTI_PAYLOAD_NX; ++i) {
                x0[i] = last_predicted_state_[i];
            }
        } else {
            for (int i = 0; i < MULTI_PAYLOAD_NX; ++i) {
                x0[i] = 0.0;
            }

            double total_mass = payload_mass_ + static_cast<double>(kNumUavs) * quad_mass_;
            double hover_thrust = (total_mass * gravity_) / static_cast<double>(kNumUavs);

            for (std::size_t i = 0; i < kNumUavs; ++i) {
                int base = kQuadStateStart + static_cast<int>(i) * kQuadStateSize;
                x0[base + 0] = 1.0;
                x0[base + 1] = 0.0;
                x0[base + 2] = 0.0;
                x0[base + 3] = 0.0;
                x0[base + 4] = hover_thrust;
            }
        }

        x0[0] = payload_position_(0);
        x0[1] = payload_position_(1);
        x0[2] = payload_position_(2);
        x0[3] = payload_velocity_(0);
        x0[4] = payload_velocity_(1);
        x0[5] = payload_velocity_(2);
        x0[6] = payload_attitude_.w();
        x0[7] = payload_attitude_.x();
        x0[8] = payload_attitude_.y();
        x0[9] = payload_attitude_.z();
        x0[10] = payload_angular_velocity_(0);
        x0[11] = payload_angular_velocity_(1);
        x0[12] = payload_angular_velocity_(2);

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            int cable_base = kCableStateStart + static_cast<int>(i) * kCableStateSize;
            x0[cable_base + 0] = cable_direction_[i](0);
            x0[cable_base + 1] = cable_direction_[i](1);
            x0[cable_base + 2] = cable_direction_[i](2);
            x0[cable_base + 3] = cable_angular_velocity_[i](0);
            x0[cable_base + 4] = cable_angular_velocity_[i](1);
            x0[cable_base + 5] = cable_angular_velocity_[i](2);
        }

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            int base = kQuadStateStart + static_cast<int>(i) * kQuadStateSize;
            x0[base + 0] = uav_attitude_[i].w();
            x0[base + 1] = uav_attitude_[i].x();
            x0[base + 2] = uav_attitude_[i].y();
            x0[base + 3] = uav_attitude_[i].z();
        }

        if (force_trajectory_reset_) {
            initializeTrajectory(x0);
            force_trajectory_reset_ = false;
        }

        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
            0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
            0, "ubx", x0);

        double yref[MULTI_PAYLOAD_NY0];
        double yref_e[MULTI_PAYLOAD_NYN];
        generateReference(yref, yref_e);

        for (int i = 0; i < MULTI_PAYLOAD_N; ++i) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MULTI_PAYLOAD_N, "yref", yref_e);

        int status = multi_payload_acados_solve(acados_ocp_capsule_);
        if (status == 0) {
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", last_predicted_state_.data());
            have_prediction_ = true;

            std::array<double, MULTI_PAYLOAD_NX> x_sol{};
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", x_sol.data());

            for (std::size_t i = 0; i < kNumUavs; ++i) {
                int base = kQuadStateStart + static_cast<int>(i) * kQuadStateSize;
                current_attitude_[i].w() = x_sol[base + 0];
                current_attitude_[i].x() = x_sol[base + 1];
                current_attitude_[i].y() = x_sol[base + 2];
                current_attitude_[i].z() = x_sol[base + 3];
                current_attitude_[i] = normalizeQuaternionSign(current_attitude_[i]);

                current_thrust_[i] = x_sol[base + 4];
            }
            if (qp_warm_start_ != 1) {
                setQpWarmStart(1);
            }
        } else {
            int qp_status = 0;
            if (nlp_solver_ != nullptr) {
                ocp_nlp_get(nlp_solver_, "qp_status", &qp_status);
            }
            RCLCPP_WARN(this->get_logger(),
                "NMPC solve failed: status=%d qp_status=%d",
                status, qp_status);
            have_prediction_ = false;
            nmpc_active_ = false;
            force_trajectory_reset_ = true;
            multi_payload_acados_reset(acados_ocp_capsule_, 1);
            setQpWarmStart(0);
        }

        printNmpcSnapshot(x0, yref);
        return status == 0;
    }

    void generateReference(double *yref, double *yref_e)
    {
        for (int i = 0; i < MULTI_PAYLOAD_NY0; ++i) {
            yref[i] = 0.0;
        }
        for (int i = 0; i < MULTI_PAYLOAD_NYN; ++i) {
            yref_e[i] = 0.0;
        }

        Eigen::Vector3d ref_pos(0.0, 0.0, 0.5);
        if (goal_pose_received_) {
            ref_pos = goal_payload_position_;
        }
        Eigen::Quaterniond ref_q(1.0, 0.0, 0.0, 0.0);

        yref[0] = ref_pos(0);
        yref[1] = ref_pos(1);
        yref[2] = ref_pos(2);
        yref[3] = 0.0;
        yref[4] = 0.0;
        yref[5] = 0.0;
        yref[6] = ref_q.w();
        yref[7] = ref_q.x();
        yref[8] = ref_q.y();
        yref[9] = ref_q.z();
        yref[10] = 0.0;
        yref[11] = 0.0;
        yref[12] = 0.0;

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            int cable_base = kCableStateStart + static_cast<int>(i) * kCableStateSize;
            yref[cable_base + 0] = 0.0;
            yref[cable_base + 1] = 0.0;
            yref[cable_base + 2] = -1.0;
            yref[cable_base + 3] = 0.0;
            yref[cable_base + 4] = 0.0;
            yref[cable_base + 5] = 0.0;

            // if(i == 0) {
            //     yref[cable_base + 0] = -0.5;
            //     yref[cable_base + 1] = 0.0;
            //     yref[cable_base + 2] = -0.86602540378;  // -sqrt(3)/2
            // } else if (i == 1) {
            //     yref[cable_base + 0] = 0.0;
            //     yref[cable_base + 1] = -0.5;
            //     yref[cable_base + 2] = -0.86602540378;  // -sqrt(3)/2
            // } else if (i == 2) {
            //     yref[cable_base + 0] = 0.0;
            //     yref[cable_base + 1] = 0.5;
            //     yref[cable_base + 2] = -0.86602540378;  // -sqrt(3)/2
            // }
        }

        double total_mass = payload_mass_ + static_cast<double>(kNumUavs) * quad_mass_;
        double hover_thrust = (total_mass * gravity_) / static_cast<double>(kNumUavs);

        for (std::size_t i = 0; i < kNumUavs; ++i) {
            int base = kQuadStateStart + static_cast<int>(i) * kQuadStateSize;
            yref[base + 0] = 1.0;
            yref[base + 1] = 0.0;
            yref[base + 2] = 0.0;
            yref[base + 3] = 0.0;
            yref[base + 4] = hover_thrust;
            yref[base + 5] = 0.0;
            yref[base + 6] = 0.0;
            yref[base + 7] = 0.0;
            yref[base + 8] = 0.0;
            yref[base + 9] = 0.0;
            yref[base + 10] = 0.0;
        }

        for (int i = 0; i < MULTI_PAYLOAD_NYN; ++i) {
            yref_e[i] = yref[i];
        }
    }

    void printNmpcSnapshot(const double *x0, const double *yref)
    {
        print_counter_++;
        if (print_counter_ % 10 != 0) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Target payload pos: [%.3f %.3f %.3f]",
            yref[0], yref[1], yref[2]);
        RCLCPP_INFO(this->get_logger(), "Current payload pos: [%.3f %.3f %.3f]",
            x0[0], x0[1], x0[2]);
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            RCLCPP_INFO(this->get_logger(),
                "[%s] thrust=%.3fN att=[%.3f %.3f %.3f %.3f]",
                uav_names_[i].c_str(),
                current_thrust_[i],
                current_attitude_[i].w(),
                current_attitude_[i].x(),
                current_attitude_[i].y(),
                current_attitude_[i].z());
        }
    }

    void publishTakeoffSetpoints()
    {
        rclcpp::Time now = this->now();
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            if (!takeoff_target_initialized_[i]) {
                continue;
            }
            takeoff_target_[i].header.stamp = now;
            setpoint_pos_pub_[i]->publish(takeoff_target_[i]);

            if (offboard_setpoint_counter_[i] < offboard_setpoint_threshold_) {
                ++offboard_setpoint_counter_[i];
            }
        }
    }

    void publishPredictedPaths()
    {
        if (!solver_initialized_ || !nmpc_active_ || !have_prediction_) {
            return;
        }

        rclcpp::Time now = this->now();

        nav_msgs::msg::Path payload_path_msg;
        payload_path_msg.header.stamp = now;
        payload_path_msg.header.frame_id = "map";

        std::array<nav_msgs::msg::Path, kNumUavs> uav_paths;
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            uav_paths[i].header.stamp = now;
            uav_paths[i].header.frame_id = "map";
        }

        const int N = MULTI_PAYLOAD_N;
        for (int k = 0; k <= N; ++k) {
            double x_pred[MULTI_PAYLOAD_NX];
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "x", x_pred);

            Eigen::Vector3d payload_pos(x_pred[0], x_pred[1], x_pred[2]);
            Eigen::Quaterniond payload_q(x_pred[6], x_pred[7], x_pred[8], x_pred[9]);
            payload_q = normalizeQuaternionSign(payload_q);

            geometry_msgs::msg::PoseStamped payload_pose;
            payload_pose.header.stamp = now;
            payload_pose.header.frame_id = "map";
            payload_pose.pose.position.x = payload_pos.x();
            payload_pose.pose.position.y = payload_pos.y();
            payload_pose.pose.position.z = payload_pos.z();
            payload_pose.pose.orientation.w = payload_q.w();
            payload_pose.pose.orientation.x = payload_q.x();
            payload_pose.pose.orientation.y = payload_q.y();
            payload_pose.pose.orientation.z = payload_q.z();
            payload_path_msg.poses.push_back(payload_pose);

            Eigen::Matrix3d R_L = payload_q.normalized().toRotationMatrix();

            for (std::size_t i = 0; i < kNumUavs; ++i) {
                int cable_base = kCableStateStart + static_cast<int>(i) * kCableStateSize;
                Eigen::Vector3d q_dir(
                    x_pred[cable_base + 0],
                    x_pred[cable_base + 1],
                    x_pred[cable_base + 2]);

                Eigen::Vector3d attach = payload_pos + R_L * r_b_[i];
                Eigen::Vector3d uav_pos = attach - cable_length_ * q_dir;

                int quad_base = kQuadStateStart + static_cast<int>(i) * kQuadStateSize;
                Eigen::Quaterniond uav_q(
                    x_pred[quad_base + 0],
                    x_pred[quad_base + 1],
                    x_pred[quad_base + 2],
                    x_pred[quad_base + 3]);
                uav_q = normalizeQuaternionSign(uav_q);

                geometry_msgs::msg::PoseStamped uav_pose;
                uav_pose.header.stamp = now;
                uav_pose.header.frame_id = "map";
                uav_pose.pose.position.x = uav_pos.x();
                uav_pose.pose.position.y = uav_pos.y();
                uav_pose.pose.position.z = uav_pos.z();
                uav_pose.pose.orientation.w = uav_q.w();
                uav_pose.pose.orientation.x = uav_q.x();
                uav_pose.pose.orientation.y = uav_q.y();
                uav_pose.pose.orientation.z = uav_q.z();
                uav_paths[i].poses.push_back(uav_pose);
            }
        }

        payload_path_pub_->publish(payload_path_msg);
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            uav_path_pub_[i]->publish(uav_paths[i]);
        }
    }

    void publishAttitudeSetpoints()
    {
        rclcpp::Time now = this->now();
        for (std::size_t i = 0; i < kNumUavs; ++i) {
            mavros_msgs::msg::AttitudeTarget msg;
            msg.header.stamp = now;
            msg.header.frame_id = "base_link";
            msg.type_mask =
                mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

            msg.orientation.w = current_attitude_[i].w();
            msg.orientation.x = current_attitude_[i].x();
            msg.orientation.y = current_attitude_[i].y();
            msg.orientation.z = current_attitude_[i].z();

            double thrust_norm = current_thrust_[i] / max_thrust_;
            thrust_norm = std::max(0.0, std::min(1.0, thrust_norm));
            msg.thrust = thrust_norm;

            attitude_pub_[i]->publish(msg);
        }
    }

    void setModeAsync(std::size_t index, const std::string &mode)
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
                    RCLCPP_INFO(this->get_logger(), "[%s] Mode set to %s",
                        uav_names_[index].c_str(), mode.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "[%s] Failed to set mode %s",
                        uav_names_[index].c_str(), mode.c_str());
                }
            });
    }

    void armAsync(std::size_t index, bool arm)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;
        auto client = arming_client_[index];
        client->async_send_request(
            request,
            [this, index, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
            {
                auto result = future.get();
                if (result && result->success) {
                    RCLCPP_INFO(this->get_logger(), "[%s] Vehicle %s",
                        uav_names_[index].c_str(), arm ? "armed" : "disarmed");
                } else {
                    RCLCPP_WARN(this->get_logger(), "[%s] Failed to %s vehicle",
                        uav_names_[index].c_str(), arm ? "arm" : "disarm");
                }
            });
    }

    Eigen::Quaterniond normalizeQuaternionSign(const Eigen::Quaterniond &q) const
    {
        if (q.w() < 0.0) {
            Eigen::Quaterniond q_norm = q;
            q_norm.coeffs() *= -1.0;
            return q_norm;
        }
        return q;
    }

    void initializeAcadosSolver()
    {
        acados_ocp_capsule_ = multi_payload_acados_create_capsule();
        int status = multi_payload_acados_create(acados_ocp_capsule_);
        if (status != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create acados solver, status=%d", status);
            solver_initialized_ = false;
            return;
        }

        nlp_in_ = multi_payload_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = multi_payload_acados_get_nlp_out(acados_ocp_capsule_);
        nlp_config_ = multi_payload_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = multi_payload_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_opts_ = multi_payload_acados_get_nlp_opts(acados_ocp_capsule_);
        nlp_solver_ = multi_payload_acados_get_nlp_solver(acados_ocp_capsule_);

        ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "print_level", &acados_print_level_);
        ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_log_residuals", &acados_rti_log_residuals_);
        solver_initialized_ = true;
    }

    void initializeTrajectory(double *x0)
    {
        if (!solver_initialized_) {
            return;
        }

        std::array<double, MULTI_PAYLOAD_NU> u0{};
        for (int i = 0; i < MULTI_PAYLOAD_N; ++i) {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x0);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u0.data());
        }
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, MULTI_PAYLOAD_N, "x", x0);
    }

    void setQpWarmStart(int enabled)
    {
        qp_warm_start_ = enabled;
        if (nlp_opts_ != nullptr && nlp_config_ != nullptr) {
            ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "qp_warm_start", &qp_warm_start_);
        }
    }

    void cleanupAcadosSolver()
    {
        if (!solver_initialized_) {
            return;
        }
        multi_payload_acados_free(acados_ocp_capsule_);
        multi_payload_acados_free_capsule(acados_ocp_capsule_);
        solver_initialized_ = false;
    }

    std::vector<std::string> uav_names_;
    double control_frequency_ = 100.0;
    double nmpc_frequency_ = 10.0;
    double takeoff_height_ = 2.0;
    double takeoff_tolerance_ = 0.2;
    int offboard_setpoint_threshold_ = 20;
    std::vector<double> takeoff_target_x_;
    std::vector<double> takeoff_target_y_;
    std::vector<double> takeoff_target_z_;

    double cable_length_ = 1.5;
    double quad_mass_ = 1.535;
    double payload_mass_ = 1.0;
    double max_thrust_ = 23.59;
    double omega_filter_alpha_ = 1.0;
    double gravity_ = 9.81;
    int acados_print_level_ = 0;
    int acados_rti_log_residuals_ = 0;

    std::array<Eigen::Vector3d, kNumUavs> r_b_;

    std::array<Eigen::Vector3d, kNumUavs> uav_position_;
    std::array<Eigen::Vector3d, kNumUavs> uav_velocity_;
    std::array<Eigen::Quaterniond, kNumUavs> uav_attitude_;
    std::array<mavros_msgs::msg::State, kNumUavs> uav_state_;

    std::array<bool, kNumUavs> uav_pose_received_;
    std::array<bool, kNumUavs> uav_velocity_received_;
    std::array<bool, kNumUavs> uav_state_received_;

    std::array<geometry_msgs::msg::PoseStamped, kNumUavs> takeoff_target_;
    std::array<bool, kNumUavs> takeoff_target_initialized_;
    std::array<int, kNumUavs> offboard_setpoint_counter_;

    Eigen::Vector3d payload_position_{0.0, 0.0, 0.0};
    Eigen::Vector3d payload_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d payload_angular_velocity_{0.0, 0.0, 0.0};
    Eigen::Quaterniond payload_attitude_{1.0, 0.0, 0.0, 0.0};
    bool payload_received_ = false;

    Eigen::Vector3d goal_payload_position_{0.0, 0.0, 0.0};
    Eigen::Quaterniond goal_payload_attitude_{1.0, 0.0, 0.0, 0.0};
    bool goal_pose_received_ = false;

    std::array<Eigen::Vector3d, kNumUavs> cable_direction_;
    std::array<Eigen::Vector3d, kNumUavs> cable_angular_velocity_;
    std::array<Eigen::Vector3d, kNumUavs> cable_prev_dir_;
    std::array<Eigen::Vector3d, kNumUavs> cable_omega_filtered_;
    std::array<rclcpp::Time, kNumUavs> cable_prev_time_;
    std::array<bool, kNumUavs> cable_prev_valid_;

    std::array<double, kNumUavs> current_thrust_{};
    std::array<Eigen::Vector3d, kNumUavs> current_body_rate_{};
    std::array<Eigen::Quaterniond, kNumUavs> current_attitude_{};

    bool nmpc_active_ = false;
    bool nmpc_pending_ = false;
    bool takeoff_complete_ = false;
    int print_counter_ = 0;
    bool solver_initialized_ = false;
    bool have_prediction_ = false;
    bool force_trajectory_reset_ = false;
    std::size_t solve_count_ = 0;
    std::size_t solve_success_count_ = 0;
    double avg_solve_time_ms_ = 0.0;

    std::array<double, MULTI_PAYLOAD_NX> last_predicted_state_{};

    multi_payload_solver_capsule *acados_ocp_capsule_ = nullptr;
    ocp_nlp_in *nlp_in_ = nullptr;
    ocp_nlp_out *nlp_out_ = nullptr;
    ocp_nlp_config *nlp_config_ = nullptr;
    ocp_nlp_dims *nlp_dims_ = nullptr;
    void *nlp_opts_ = nullptr;
    ocp_nlp_solver *nlp_solver_ = nullptr;
    int qp_warm_start_ = 1;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr solve_timer_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr payload_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, kNumUavs> uav_pose_sub_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr, kNumUavs> uav_velocity_sub_;
    std::array<rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr, kNumUavs> uav_state_sub_;

    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, kNumUavs> setpoint_pos_pub_;
    std::array<rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr, kNumUavs> attitude_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr payload_path_pub_;
    std::array<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr, kNumUavs> uav_path_pub_;

    std::array<rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr, kNumUavs> arming_client_;
    std::array<rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr, kNumUavs> set_mode_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiPayloadNmpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
