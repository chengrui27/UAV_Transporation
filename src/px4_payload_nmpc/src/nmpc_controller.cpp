/**
 * @file nmpc_controller.cpp
 * @brief PX4四旋翼悬挂负载NMPC控制器节点
 *
 * 基于论文: Nonlinear Backstepping Control of a Quadrotor-Slung Load System
 * 实现功能:
 * 1. 读取PX4和负载状态信息
 * 2. 计算NMPC状态变量(16维)
 * 3. 调用acados求解器解算控制输入
 * 4. 通过Offboard模式Body Rate控制无人机
 *
 * 坐标系说明:
 * - World Frame: ENU (East-North-Up)
 * - Body Frame: FLU (Forward-Left-Up)
 * - 所有数据统一使用ENU坐标系，无需转换
 *
 * @author PX4 Payload NMPC Team
 * @date 2025-10-17
 */

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
#include <memory>
#include <chrono>
#include <cmath>

// ESDF地图读取器
#include "px4_payload_nmpc/esdf_map_reader.h"

// acados generated solver interface
extern "C" {
    #include "acados_c/ocp_nlp_interface.h"
    #include "acados_solver_px4_payload_model.h"
}

using namespace std::chrono_literals;

/**
 * @brief NMPC控制器类
 */
class NMPCController : public rclcpp::Node {
public:
    NMPCController() : Node("nmpc_controller") {
        // 声明参数
        this->declare_parameter("cable_length", 1.0);          // 绳长 (m)
        this->declare_parameter("quad_mass", 2.095);             // 四旋翼质量 (kg)
        this->declare_parameter("payload_mass", 0.5);          // 负载质量 (kg)
        this->declare_parameter("control_frequency", 100.0);   // 控制频率 (Hz)
        this->declare_parameter("offboard_delay", 2.0);        // Offboard延迟 (s)

        // 获取参数
        cable_length_ = this->get_parameter("cable_length").as_double();
        quad_mass_ = this->get_parameter("quad_mass").as_double();
        payload_mass_ = this->get_parameter("payload_mass").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        offboard_delay_ = this->get_parameter("offboard_delay").as_double();
        gravity_ = 9.81;

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "NMPC Controller Node Initialized");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Cable length: %.2f m", cable_length_);
        RCLCPP_INFO(this->get_logger(), "  Quad mass: %.2f kg", quad_mass_);
        RCLCPP_INFO(this->get_logger(), "  Payload mass: %.2f kg", payload_mass_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.0f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "  Offboard delay: %.1f s", offboard_delay_);

        // 设置QoS为BEST_EFFORT以匹配MAVROS
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10));

        // 初始化订阅者
        quad_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_best_effort,
            std::bind(&NMPCController::quadPoseCallback, this, std::placeholders::_1));

        quad_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos_best_effort,
            std::bind(&NMPCController::quadVelocityCallback, this, std::placeholders::_1));

        payload_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/payload/odom", qos_reliable,
            std::bind(&NMPCController::payloadOdomCallback, this, std::placeholders::_1));

        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_best_effort,
            std::bind(&NMPCController::mavrosStateCallback, this, std::placeholders::_1));

        // 初始化发布者
        attitude_target_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
        predicted_payload_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/payload_nmpc/payload_predicted_path", 10);
        predicted_quad_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/payload_nmpc/quad_predicted_path", 10);

        // 初始化服务客户端
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");

        // 订阅RViz目标点 (负载参考位置)
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&NMPCController::goalPoseCallback, this, std::placeholders::_1));

        // 初始化acados求解器
        initializeAcadosSolver();

        // 创建控制定时器 (100Hz)
        auto timer_period = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / control_frequency_));
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&NMPCController::controlTimerCallback, this));

        // 创建预测轨迹发布定时器 (0.5秒 = 2Hz)
        path_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NMPCController::publishPredictedPath, this));

        RCLCPP_INFO(this->get_logger(), "Waiting for data from PX4 and Gazebo...");
    }

    ~NMPCController() {
        cleanupAcadosSolver();
        RCLCPP_INFO(this->get_logger(), "NMPC Controller shutdown");
    }

    /**
     * @brief 初始化ESDF地图读取器（在节点创建后调用）
     */
    void initializeEsdfReader() {
        auto node_ptr = rclcpp::Node::shared_from_this();
        esdf_reader_ = std::make_shared<ESDFMapReader>(node_ptr);
        RCLCPP_INFO(this->get_logger(), "✓ ESDF Map Reader (payload) initialized");
    }

private:
    // ========== 回调函数 ==========

    /**
     * @brief 四旋翼位置姿态回调 (ENU坐标系)
     */
    void quadPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        quad_position_enu_ << msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z;

        quad_attitude_enu_.w() = msg->pose.orientation.w;
        quad_attitude_enu_.x() = msg->pose.orientation.x;
        quad_attitude_enu_.y() = msg->pose.orientation.y;
        quad_attitude_enu_.z() = msg->pose.orientation.z;

        if (!quad_pose_received_) {
            quad_pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Quadrotor pose received");
        }
    }

    /**
     * @brief 四旋翼速度回调 (ENU坐标系)
     */
    void quadVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        quad_velocity_enu_ << msg->twist.linear.x,
                              msg->twist.linear.y,
                              msg->twist.linear.z;

        if (!quad_velocity_received_) {
            quad_velocity_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Quadrotor velocity received");
        }
    }

    /**
     * @brief 目标位置姿态回调 (从RViz，作为负载参考位置)
     */
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 保存参考位置 (ENU)
        goal_payload_position_ << msg->pose.position.x,
                                  msg->pose.position.y,
                                  msg->pose.position.z;

        // 保存参考姿态（四元数，用于参考机体姿态，当前仅跟踪位置，可视需要扩展）
        goal_attitude_.w() = msg->pose.orientation.w;
        goal_attitude_.x() = msg->pose.orientation.x;
        goal_attitude_.y() = msg->pose.orientation.y;
        goal_attitude_.z() = msg->pose.orientation.z;
        goal_attitude_.normalize();

        goal_pose_received_ = true;

        RCLCPP_INFO(this->get_logger(), "New payload goal received:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f] m",
                    goal_payload_position_(0),
                    goal_payload_position_(1),
                    goal_payload_position_(2));

        // 查询目标点在ESDF地图中的距离和梯度，方便调试避障行为
        // if (esdf_reader_ && esdf_reader_->isMapValid()) {
        //     Eigen::Vector3d goal_pos_enu(goal_payload_position_(0),
        //                                  goal_payload_position_(1),
        //                                  goal_payload_position_(2));
        //     double esdf_dist = esdf_reader_->getDistance(goal_pos_enu);
        //     Eigen::Vector3d esdf_grad = esdf_reader_->getGradient(goal_pos_enu);
        //     RCLCPP_INFO(this->get_logger(),
        //                 "ESDF at payload goal: dist=%.3f m, grad=[%.3f, %.3f, %.3f]",
        //                 esdf_dist, esdf_grad.x(), esdf_grad.y(), esdf_grad.z());
        // } else {
        //     RCLCPP_WARN(this->get_logger(),
        //                 "ESDF map not valid when goal received, cannot query dist/grad");
        // }

        // 重置ESDF相关状态：下次求解时从“干净”的参数开始
        first_solve_done_ = false;
        resetAcadosStateForNewGoal();
    }

    /**
     * @brief 负载里程计回调 (相对于无人机的位置和速度)
     *
     * payload_p3d插件现在发布相对于无人机body frame的位置和速度
     * 需要转换为全局ENU坐标系
     */
    void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 提取相对位置 (body frame)
        Eigen::Vector3d payload_relative_position;
        payload_relative_position << msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z;

        // 提取相对速度 (body frame)
        Eigen::Vector3d payload_relative_velocity;
        payload_relative_velocity << msg->twist.twist.linear.x,
                                     msg->twist.twist.linear.y,
                                     msg->twist.twist.linear.z;

        // 将相对位置从body frame转换到ENU全局坐标系
        // payload_global = quad_position + R_body_to_enu * payload_relative
        Eigen::Matrix3d R_body_to_enu = quad_attitude_enu_.toRotationMatrix();
        Eigen::Vector3d payload_position_global = quad_position_enu_ + R_body_to_enu * payload_relative_position;

        // 将相对速度从body frame转换到ENU全局坐标系
        // payload_velocity_global = quad_velocity + R_body_to_enu * payload_relative_velocity
        // 注意: 这里忽略了旋转引起的速度项 (ω × r)，对于慢速旋转可以近似
        Eigen::Vector3d payload_velocity_global = quad_velocity_enu_ + R_body_to_enu * payload_relative_velocity;

        // 保存全局位置和速度
        payload_position_enu_ = payload_position_global;
        payload_velocity_enu_ = payload_velocity_global;

        if (!payload_odom_received_) {
            payload_odom_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Payload odometry received (relative to quad)");
        }

        // 检查是否所有数据都已接收
        if(!all_data_ready_){
            if (quad_pose_received_ && quad_velocity_received_ && payload_odom_received_) {
                all_data_ready_ = true;
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "✓ All sensors ready!");
                RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before enabling Offboard...",
                           offboard_delay_);
                offboard_start_time_ = this->now();
            }
        }
    }

    /**
     * @brief MAVROS状态回调
     */
    void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        mavros_state_ = *msg;
    }

    /**
     * @brief 控制定时器回调 (100Hz)
     */
    void controlTimerCallback() {
        // 检查数据是否就绪
        if (!all_data_ready_) {
            // 即使数据未就绪，也发送心跳以满足 PX4 Offboard 要求
            publishHeartbeat();
            return;
        }

        // 检查是否需要启用Offboard模式 (等待2秒)
        if (!offboard_enabled_) {
            auto elapsed = (this->now() - offboard_start_time_).seconds();

            // 在启用 Offboard 之前持续发送心跳
            publishHeartbeat();

            if (elapsed >= offboard_delay_) {
                enableOffboardMode();
                armVehicle();
                offboard_enabled_ = true;
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "🚁 Offboard mode enabled! Starting NMPC control...");
                RCLCPP_INFO(this->get_logger(), "========================================");
            }
            return;
        }

        // 计算NMPC状态变量
        computeNMPCState();

        // 求解NMPC
        auto solve_start = std::chrono::high_resolution_clock::now();
        bool success = solveNMPC();
        auto solve_end = std::chrono::high_resolution_clock::now();

        double solve_time = std::chrono::duration<double, std::milli>(
            solve_end - solve_start).count();

        solve_count_++;
        if (success) {
            solve_success_count_++;
            avg_solve_time_ms_ = (avg_solve_time_ms_ * (solve_count_ - 1) + solve_time) / solve_count_;

            // 发布控制指令
            publishBodyRateControl();

            // 每100次打印一次统计
            if (solve_count_ % 100 == 0) {
                double success_rate = 100.0 * solve_success_count_ / solve_count_;
                RCLCPP_INFO(this->get_logger(),
                    "NMPC Stats: Success rate: %.1f%%, Avg solve time: %.2f ms",
                    success_rate, avg_solve_time_ms_);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "NMPC solver failed! Using previous control input.");
            // 使用上次的控制输入
            publishBodyRateControl();
        }
    }

    // ========== 状态计算函数 ==========

    /**
     * @brief 计算NMPC状态变量 (16维)
     *
     * 状态向量: x = [pL, vL, q, ω, quaternion]^T
     * 坐标系: 统一使用ENU坐标系
     */
    void computeNMPCState() {
        // 1. 直接使用ENU坐标系的负载位置和速度
        current_state_.payload_position_enu = payload_position_enu_;
        current_state_.payload_velocity_enu = payload_velocity_enu_;

        // 2. 归一化四元数符号（强制w>=0，避免双重覆盖问题）
        current_state_.quad_attitude_enu = normalizeQuaternionSign(quad_attitude_enu_);

        // 3. 计算绳子方向向量 q (ENU坐标系)
        // 公式(1): ℓq = pL - pQ
        Eigen::Vector3d cable_vector = current_state_.payload_position_enu - quad_position_enu_;
        double cable_length_actual = cable_vector.norm();

        if (cable_length_actual > 1e-6) {
            current_state_.cable_direction = cable_vector / cable_length_actual;
        } else {
            // 如果绳长为0，默认垂直向下 (ENU: z负方向)
            current_state_.cable_direction << 0.0, 0.0, -1.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Cable length is nearly zero!");
        }

        // 4. 计算绳子角速度 ω (ENU坐标系)
        // 从 q̇ = S(ω)q 推导 ω
        // 使用速度差计算
        Eigen::Vector3d velocity_diff = current_state_.payload_velocity_enu - quad_velocity_enu_;

        // ω = r x v / ℓ*ℓ (简化公式)
        if (cable_length_actual > 1e-6) {
            current_state_.cable_angular_velocity =
                current_state_.cable_direction.cross(velocity_diff) / cable_length_actual;
        } else {
            current_state_.cable_angular_velocity.setZero();
        }


    }

    /**
     * @brief 生成负载悬停参考 (来自RViz goal_pose)
     * @param yref 输出参考向量 (20维: 16维状态 + 4维控制)
     * @param yref_e 输出终端参考向量 (16维: 仅状态)
     */
    void generatePayloadReference(double* yref, double* yref_e) {
        // 若尚未收到goal_pose，则使用一个默认目标点，避免未初始化
        Eigen::Vector3d ref_pos = goal_payload_position_;
        if (!goal_pose_received_) {
            ref_pos << 0.0, 0.0, 0.5;
        }

        // 参考负载位置 (ENU)
        yref[0] = ref_pos(0);   // East (m)
        yref[1] = ref_pos(1);   // North (m)
        yref[2] = 0.5;   // Up (m)

        // 参考负载速度 (保持静止)
        yref[3] = 0.0;  // vE
        yref[4] = 0.0;  // vN
        yref[5] = 0.0;  // vU

        // 参考绳子方向 (ENU, 垂直向下)
        yref[6] = 0.0;   // qx
        yref[7] = 0.0;   // qy
        yref[8] = -1.0;  // qz

        // 参考绳子角速度
        yref[9]  = 0.0;  // ωx
        yref[10] = 0.0;  // ωy
        yref[11] = 0.0;  // ωz

        // 参考机体姿态（这里保持水平，机头朝东；如需跟随goal姿态可扩展）
        yref[12] = 1.0;  // q0 (w)
        yref[13] = 0.0;  // q1 (x)
        yref[14] = 0.0;  // q2 (y)
        yref[15] = 0.0;  // q3 (z)

        // 参考控制输入：悬停推力 + 零角速度
        double hover_thrust = (quad_mass_ + payload_mass_) * gravity_;
        yref[16] = hover_thrust;  // T (N)
        yref[17] = 0.0;           // Ωx
        yref[18] = 0.0;           // Ωy
        yref[19] = 0.0;           // Ωz

        // 终端参考 (仅状态, 16维)
        for (int i = 0; i < 16; i++) {
            yref_e[i] = yref[i];
        }
    }

    /**
     * @brief 求解NMPC（集成ESDF避障软约束）
     */
    bool solveNMPC() {
        if (acados_ocp_capsule_ == nullptr) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Acados solver not initialized!");
            return false;
        }

        // 准备状态向量 (16维)
        double x0[16];
        x0[0] = current_state_.payload_position_enu(0);
        x0[1] = current_state_.payload_position_enu(1);
        x0[2] = current_state_.payload_position_enu(2);
        x0[3] = current_state_.payload_velocity_enu(0);
        x0[4] = current_state_.payload_velocity_enu(1);
        x0[5] = current_state_.payload_velocity_enu(2);
        x0[6] = current_state_.cable_direction(0);
        x0[7] = current_state_.cable_direction(1);
        x0[8] = current_state_.cable_direction(2);
        x0[9] = current_state_.cable_angular_velocity(0);
        x0[10] = current_state_.cable_angular_velocity(1);
        x0[11] = current_state_.cable_angular_velocity(2);
        x0[12] = current_state_.quad_attitude_enu.w();
        x0[13] = current_state_.quad_attitude_enu.x();
        x0[14] = current_state_.quad_attitude_enu.y();
        x0[15] = current_state_.quad_attitude_enu.z();

        // 设置初始状态约束
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      0, "ubx", x0);

        // 初始化轨迹开始时间
        if (!trajectory_started_) {
            trajectory_start_time_ = this->now();
            trajectory_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting payload NMPC tracking using RViz goal_pose");
        }

        // 生成参考轨迹 (基于RViz目标点)
        double yref[20];      // 状态(16) + 控制(4)
        double yref_e[16];    // 终端状态
        generatePayloadReference(yref, yref_e);

        // 设置所有预测阶段的参考轨迹 (0 到 N-1)
        int N = PX4_PAYLOAD_MODEL_N;  // 预测时域长度 (与px4_payload_nmpc.py中一致)
        for (int i = 0; i < N; i++) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);
        }

        // 设置终端阶段的参考 (N)
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "yref", yref_e);

        // ========== ESDF避障约束参数更新（基于上一轮预测轨迹的初始线性化） ==========
        if (esdf_reader_ && esdf_reader_->isMapValid()) {
            Eigen::Vector3d payload_pos(x0[0], x0[1], x0[2]);
            Eigen::Vector3d rope_dir(x0[6], x0[7], x0[8]);
            Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

            double dist_L = esdf_reader_->getDistance(payload_pos);
            double dist_Q = esdf_reader_->getDistance(quad_pos);
            bool in_bound_L = esdf_reader_->isInLocalBound(payload_pos);
            bool in_bound_Q = esdf_reader_->isInLocalBound(quad_pos);
            (void)dist_L;
            (void)dist_Q;
            (void)in_bound_L;
            (void)in_bound_Q;
        }

        // 第一次求解时跳过参数更新（使用默认参数）
        if (!first_solve_done_) {
            RCLCPP_INFO(this->get_logger(), "First solve (payload): using default ESDF parameters");
            first_solve_done_ = true;
        } else if (esdf_reader_ && esdf_reader_->isMapValid()) {
            // 从第二次求解开始，为每个预测阶段查询ESDF并更新参数（使用上一轮预测轨迹）
            for (int i = 0; i < N; i++) {
                // 1. 提取该阶段的猜测状态
                double xi[16];  // [xp, yp, zp, vxp, vyp, vzp, qx, qy, qz, wx, wy, wz, q0, q1, q2, q3]
                ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

                // 2. 计算负载和无人机位置
                Eigen::Vector3d payload_pos(xi[0], xi[1], xi[2]);
                Eigen::Vector3d rope_dir(xi[6], xi[7], xi[8]);
                Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

                // 3. 查询ESDF距离和梯度
                double dL, dQ;
                Eigen::Vector3d gradL, gradQ;
                esdf_reader_->getDistanceAndGradient(payload_pos, dL, gradL);
                esdf_reader_->getDistanceAndGradient(quad_pos, dQ, gradQ);

                // 4. 构造 14 维参数向量:
                // [dL, gLx, gLy, gLz, xL, yL, zL,
                //  dQ, gQx, gQy, gQz, xQ, yQ, zQ]
                double p[14];
                p[0]  = dL;
                p[1]  = gradL.x();
                p[2]  = gradL.y();
                p[3]  = gradL.z();
                p[4]  = payload_pos.x();
                p[5]  = payload_pos.y();
                p[6]  = payload_pos.z();

                p[7]  = dQ;
                p[8]  = gradQ.x();
                p[9]  = gradQ.y();
                p[10] = gradQ.z();
                p[11] = quad_pos.x();
                p[12] = quad_pos.y();
                p[13] = quad_pos.z();

                // 5. 更新acados参数
                // int ret = px4_payload_model_acados_update_params(acados_ocp_capsule_, i, p, 14);
                // if (ret != 0) {
                //     RCLCPP_ERROR(this->get_logger(), "Failed to update params at stage %d, ret=%d", i, ret);
                // }
            }
        }

        // ========== 多次内层迭代：检查预测是否碰撞，必要时重新线性化ESDF并再次求解 ==========
        const int max_inner_iters = 3;
        // 与 px4_payload_nmpc.py 中的安全距离保持一致：
        // 负载安全距离 0.2 m，无人机安全距离 0.4 m
        const double safety_distance_L_check = 0.0;
        const double safety_distance_Q_check = 0.0;

        bool solver_ok = false;

        for (int iter = 0; iter < max_inner_iters; ++iter) {
            // 调用求解器
            int status = px4_payload_model_acados_solve(acados_ocp_capsule_);
            if (status != 0) {
                // RCLCPP_WARN(this->get_logger(),
                //             "Acados solve failed at inner iter %d with status %d", iter, status);
            }

            solver_ok = true;

            // 如果ESDF不可用，无法判断碰撞，直接使用当前解
            if (!(esdf_reader_ && esdf_reader_->isMapValid())) {
                break;
            }

            // 基于当前预测轨迹检查是否发生“碰撞”（负载或无人机距离小于各自安全距离）
            bool has_collision = false;
            double min_margin = 1e9;

            for (int k = 0; k <= N; ++k) {
                double x_pred[16];  // [xp, yp, zp, ...]
                ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "x", x_pred);

                Eigen::Vector3d payload_pos(x_pred[0], x_pred[1], x_pred[2]);
                Eigen::Vector3d rope_dir(x_pred[6], x_pred[7], x_pred[8]);
                Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

                double dist_L = esdf_reader_->getDistance(payload_pos);
                double dist_Q = esdf_reader_->getDistance(quad_pos);

                // 对应到 Python 中 h_L = d_L - 0.2, h_Q = d_Q - 0.4
                double margin_L = dist_L - safety_distance_L_check;
                double margin_Q = dist_Q - safety_distance_Q_check;

                double local_min_margin = std::min(margin_L, margin_Q);
                min_margin = std::min(min_margin, local_min_margin);

                if (margin_L < 0.0 || margin_Q < 0.0) {
                    const char* which =
                        (margin_L < 0.0 && margin_Q < 0.0) ? "both" :
                        (margin_L < 0.0) ? "payload" : "quad";
                    RCLCPP_WARN(this->get_logger(),
                                "ESDF collision detected at inner_iter=%d, node=%d (%s): "
                                "dist_L=%.3f, dist_Q=%.3f, margin_L=%.3f, margin_Q=%.3f",
                                iter, k, which, dist_L, dist_Q, margin_L, margin_Q);
                    has_collision = true;
                    break;
                }
            }

            if (!has_collision) {
                // 预测轨迹在整个时域内均满足安全距离，结束内层迭代
                break;
            }

            // 若预测轨迹仍存在碰撞且未达到最大迭代次数：基于当前预测轨迹重新线性化ESDF并更新参数，进入下一轮求解
            if (iter < max_inner_iters - 1) {
                for (int i = 0; i < N; ++i) {
                    double xi[16];
                    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

                    Eigen::Vector3d payload_pos(xi[0], xi[1], xi[2]);
                    Eigen::Vector3d rope_dir(xi[6], xi[7], xi[8]);
                    Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

                    double dL, dQ;
                    Eigen::Vector3d gradL, gradQ;
                    esdf_reader_->getDistanceAndGradient(payload_pos, dL, gradL);
                    esdf_reader_->getDistanceAndGradient(quad_pos, dQ, gradQ);

                    double p[14];
                    p[0]  = dL;
                    p[1]  = gradL.x();
                    p[2]  = gradL.y();
                    p[3]  = gradL.z();
                    p[4]  = payload_pos.x();
                    p[5]  = payload_pos.y();
                    p[6]  = payload_pos.z();

                    p[7]  = dQ;
                    p[8]  = gradQ.x();
                    p[9]  = gradQ.y();
                    p[10] = gradQ.z();
                    p[11] = quad_pos.x();
                    p[12] = quad_pos.y();
                    p[13] = quad_pos.z();

                    // int ret = px4_payload_model_acados_update_params(acados_ocp_capsule_, i, p, 14);
                    // (void)ret;
                }
                // 进入下一轮 inner iter
            } else {
                // 已达到最大迭代次数，使用当前解但发出警告
                RCLCPP_WARN(this->get_logger(),
                            "Payload NMPC predicted trajectory still colliding after %d inner iters "
                            "(min_margin=%.3f m), using last solution",
                            max_inner_iters, min_margin);
            }
        }

        if (!solver_ok) {
            return false;
        }

        // 提取控制输入 (4维: [T, Ωx, Ωy, Ωz])
        double u0[4];
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0);

        // 保存控制输入
        current_control_.thrust = u0[0];
        current_control_.body_rate_x = u0[1];
        current_control_.body_rate_y = u0[2];
        current_control_.body_rate_z = u0[3];

        // 打印状态和控制输出 (每10次打印一次，避免刷屏)
        print_counter_++;
        if (print_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "========== NMPC State & Control ==========");
            RCLCPP_INFO(this->get_logger(), "Payload Pos (ENU): [%.3f, %.3f, %.3f] m",
                x0[0], x0[1], x0[2]);
            RCLCPP_INFO(this->get_logger(), "Payload Vel (ENU): [%.3f, %.3f, %.3f] m/s",
                x0[3], x0[4], x0[5]);
            RCLCPP_INFO(this->get_logger(), "Cable Dir (ENU):   [%.3f, %.3f, %.3f]",
                x0[6], x0[7], x0[8]);
            RCLCPP_INFO(this->get_logger(), "Cable AngVel (ENU):[%.3f, %.3f, %.3f] rad/s",
                x0[9], x0[10], x0[11]);
            RCLCPP_INFO(this->get_logger(), "Quad Quat (ENU):   [%.3f, %.3f, %.3f, %.3f]",
                x0[12], x0[13], x0[14], x0[15]);
            RCLCPP_INFO(this->get_logger(), "NMPC Output:");
            RCLCPP_INFO(this->get_logger(), "  Thrust: %.3f N", u0[0]);
            RCLCPP_INFO(this->get_logger(), "  Body Rate: [%.3f, %.3f, %.3f] rad/s",
                u0[1], u0[2], u0[3]);
            RCLCPP_INFO(this->get_logger(), "Reference (Circular):");
            RCLCPP_INFO(this->get_logger(), "  Pos: [%.3f, %.3f, %.3f] m", yref[0], yref[1], yref[2]);
            RCLCPP_INFO(this->get_logger(), "  Vel: [%.3f, %.3f, %.3f] m/s", yref[3], yref[4], yref[5]);
            RCLCPP_INFO(this->get_logger(), "  Yaw: %.3f rad", atan2(yref[15], yref[12]) * 2.0);
            RCLCPP_INFO(this->get_logger(), "  Thrust: %.3f N", yref[16]);
            RCLCPP_INFO(this->get_logger(), "==========================================");
        }

        return true;
    }

    /**
     * @brief 发布NMPC预测的负载轨迹 (每0.5秒)
     */
    void publishPredictedPath() {
        if (acados_ocp_capsule_ == nullptr || !all_data_ready_) {
            return;
        }

        // 创建Path消息：负载轨迹与无人机轨迹
        auto payload_path_msg = nav_msgs::msg::Path();
        payload_path_msg.header.stamp = this->now();
        payload_path_msg.header.frame_id = "map";

        auto quad_path_msg = nav_msgs::msg::Path();
        quad_path_msg.header.stamp = this->now();
        quad_path_msg.header.frame_id = "map";

        // 预测时域长度
        int N = PX4_PAYLOAD_MODEL_N;  // 与生成的acados求解器保持一致

        // 提取预测轨迹 (0到N共N+1个点)
        for (int i = 0; i <= N; i++) {
            double x_pred[16];  // [xp, yp, zp, vxp, vyp, vzp, qx, qy, qz, wx, wy, wz, q0, q1, q2, q3]
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_pred);

            // 负载位置
            Eigen::Vector3d payload_pos(x_pred[0], x_pred[1], x_pred[2]);
            // 绳子方向和无人机位置
            Eigen::Vector3d rope_dir(x_pred[6], x_pred[7], x_pred[8]);
            Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

            // 负载轨迹点
            geometry_msgs::msg::PoseStamped payload_pose;
            payload_pose.header.stamp = this->now();
            payload_pose.header.frame_id = "map";
            payload_pose.pose.position.x = payload_pos.x();
            payload_pose.pose.position.y = payload_pos.y();
            payload_pose.pose.position.z = payload_pos.z();
            payload_pose.pose.orientation.w = x_pred[12];
            payload_pose.pose.orientation.x = x_pred[13];
            payload_pose.pose.orientation.y = x_pred[14];
            payload_pose.pose.orientation.z = x_pred[15];
            payload_path_msg.poses.push_back(payload_pose);

            // 无人机轨迹点（使用相同姿态，可视需要再细化）
            geometry_msgs::msg::PoseStamped quad_pose;
            quad_pose.header.stamp = this->now();
            quad_pose.header.frame_id = "map";
            quad_pose.pose.position.x = quad_pos.x();
            quad_pose.pose.position.y = quad_pos.y();
            quad_pose.pose.position.z = quad_pos.z();
            quad_pose.pose.orientation.w = x_pred[12];
            quad_pose.pose.orientation.x = x_pred[13];
            quad_pose.pose.orientation.y = x_pred[14];
            quad_pose.pose.orientation.z = x_pred[15];
            quad_path_msg.poses.push_back(quad_pose);
        }

        predicted_payload_path_pub_->publish(payload_path_msg);
        predicted_quad_path_pub_->publish(quad_path_msg);
    }

    /**
     * @brief 发布Body Rate控制指令
     */
    void publishBodyRateControl() {
        auto msg = mavros_msgs::msg::AttitudeTarget();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        // 设置控制模式: Body Rate + Thrust
        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

        // 设置Body Rate (rad/s, body frame)
        // ENU+FLU坐标系: 直接使用NMPC输出，无需转换
        msg.body_rate.x = current_control_.body_rate_x;
        msg.body_rate.y = current_control_.body_rate_y;
        msg.body_rate.z = current_control_.body_rate_z; 

        // 设置推力 (归一化 0-1)
        // T_normalized = T / (mQ + mL) / g
        double max_thrust = 40.6673;
        double thrust_normalized = current_control_.thrust / max_thrust;

        // 限制范围
        thrust_normalized = std::max(-1.0, std::min(1.0, thrust_normalized));
        msg.thrust = thrust_normalized;//0.7937;

        attitude_target_pub_->publish(msg);
    }

    /**
     * @brief 发送心跳消息 (在等待Offboard期间)
     */
    void publishHeartbeat() {
        auto msg = mavros_msgs::msg::AttitudeTarget();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = 0.0;
        msg.body_rate.y = 0.0;
        msg.body_rate.z = 0.0;

        // 计算悬停推力（归一化）
        double hover_thrust = 0.625;
        msg.thrust = std::max(0.0, std::min(1.0, hover_thrust));

        attitude_target_pub_->publish(msg);
    }

    // ========== 四元数工具函数 ==========

    /**
     * @brief 归一化四元数符号
     *
     * 四元数q和-q表示相同的旋转，但在NMPC中会产生不同的误差
     * 强制w分量为正，避免四元数双重覆盖问题
     */
    Eigen::Quaterniond normalizeQuaternionSign(const Eigen::Quaterniond& q) const {
        if (q.w() < 0) {
            Eigen::Quaterniond q_normalized = q;
            q_normalized.coeffs() *= -1.0;
            return q_normalized;
        }
        return q;
    }

    // ========== Offboard控制 ==========

    /**
     * @brief 启用Offboard模式
     */
    bool enableOffboardMode() {
        if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";

        auto result = set_mode_client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Offboard mode requested");
        return true;
    }

    /**
     * @brief 解锁飞机
     */
    bool armVehicle() {
        if (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto result = arming_client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Vehicle arming requested");
        return true;
    }

    // ========== acados求解器管理 ==========

    /**
     * @brief 初始化acados求解器
     */
    void initializeAcadosSolver() {
        RCLCPP_INFO(this->get_logger(), "Initializing acados solver...");

        // 创建求解器capsule
        acados_ocp_capsule_ = px4_payload_model_acados_create_capsule();

        if (acados_ocp_capsule_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create acados capsule!");
            return;
        }

        // 创建求解器
        int status = px4_payload_model_acados_create(acados_ocp_capsule_);

        if (status != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create acados solver! Status: %d", status);
            return;
        }

        // 获取求解器组件
        nlp_config_ = px4_payload_model_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = px4_payload_model_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_in_ = px4_payload_model_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = px4_payload_model_acados_get_nlp_out(acados_ocp_capsule_);
        nlp_solver_ = px4_payload_model_acados_get_nlp_solver(acados_ocp_capsule_);
        nlp_opts_ = px4_payload_model_acados_get_nlp_opts(acados_ocp_capsule_);

        // 为所有阶段设置折扣后的代价权重矩阵
        applyStageCostDiscount();

        RCLCPP_INFO(this->get_logger(), "✓ Acados solver initialized successfully");
    }

    /**
     * @brief 清理acados求解器
     */
    void cleanupAcadosSolver() {
        if (acados_ocp_capsule_ != nullptr) {
            px4_payload_model_acados_free(acados_ocp_capsule_);
            px4_payload_model_acados_free_capsule(acados_ocp_capsule_);
            RCLCPP_INFO(this->get_logger(), "Acados solver cleaned up");
        }
    }

    /**
     * @brief 在接收到新目标点时重置acados内部状态和ESDF参数
     *
     * - 调用 px4_payload_model_acados_reset 重置求解器内部状态和QP内存
     * - 将运行时ESDF参数初始化为: 距离=10, 梯度=0，参考点为当前负载位置
     */
    void resetAcadosStateForNewGoal() {
        if (acados_ocp_capsule_ == nullptr) {
            return;
        }

        // 重置求解器内部状态和QP求解器内存
        int reset_status = px4_payload_model_acados_reset(acados_ocp_capsule_, 1);
        if (reset_status != 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to reset payload acados solver, status=%d", reset_status);
        }

        // 使用当前负载和无人机位置作为线性化参考点，距离设为10，梯度设为0
        Eigen::Vector3d payload_pos = payload_position_enu_;
        Eigen::Vector3d rope_dir = current_state_.cable_direction;
        Eigen::Vector3d quad_pos = payload_pos - cable_length_ * rope_dir;

        double p[14];
        // 负载
        p[0]  = 10.0;
        p[1]  = 0.0;
        p[2]  = 0.0;
        p[3]  = 0.0;
        p[4]  = payload_pos.x();
        p[5]  = payload_pos.y();
        p[6]  = payload_pos.z();
        // 无人机
        p[7]  = 10.0;
        p[8]  = 0.0;
        p[9]  = 0.0;
        p[10] = 0.0;
        p[11] = quad_pos.x();
        p[12] = quad_pos.y();
        p[13] = quad_pos.z();

        // 预测时域长度，与生成的acados求解器保持一致
        int N = PX4_PAYLOAD_MODEL_N;
        for (int i = 0; i < N; ++i) {
            int ret = px4_payload_model_acados_update_params(acados_ocp_capsule_, i, p, 14);
            if (ret != 0) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to init params at stage %d when resetting goal, ret=%d",
                            i, ret);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Payload acados state and ESDF params reset for new goal");
    }

    /**
     * @brief 为所有阶段设置折扣代价权重 W_k = 0.9^k * 50 * I
     */
    void applyStageCostDiscount() {
        if (acados_ocp_capsule_ == nullptr) {
            return;
        }

        const int N  = PX4_PAYLOAD_MODEL_N;
        const int ny = PX4_PAYLOAD_MODEL_NY;

        // 基础权重矩阵 W_base = 50 * I
        double W_base[PX4_PAYLOAD_MODEL_NY * PX4_PAYLOAD_MODEL_NY] = {0.0};
        for (int i = 0; i < ny; ++i) {
            W_base[i * ny + i] = 50.0;
        }

        W_base[2 * ny + 2] = 200.0;  // 负载位置 z

        const double gamma = 0.9;

        // 为每个阶段设置折扣后的 W_k
        for (int stage = 0; stage < N; ++stage) {
            double factor = std::pow(gamma, stage);
            double W_stage[PX4_PAYLOAD_MODEL_NY * PX4_PAYLOAD_MODEL_NY];

            for (int i = 0; i < ny * ny; ++i) {
                W_stage[i] = factor * W_base[i];
            }

            ocp_nlp_cost_model_set(
                nlp_config_,
                nlp_dims_,
                nlp_in_,
                stage,
                "W",
                W_stage);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Stage cost discount applied: gamma=0.9, W_base=50*I");
    }

    // ========== 数据结构 ==========

    struct NMPCState {
        Eigen::Vector3d payload_position_enu;
        Eigen::Vector3d payload_velocity_enu;
        Eigen::Vector3d cable_direction;
        Eigen::Vector3d cable_angular_velocity;
        Eigen::Quaterniond quad_attitude_enu;
    };

    struct NMPCControl {
        double thrust;
        double body_rate_x;
        double body_rate_y;
        double body_rate_z;
    };

    // ========== 数据成员 ==========

    // ROS2订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr quad_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr quad_velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr payload_odom_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    // ROS2发布者
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_payload_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_quad_path_pub_;

    // ROS2服务客户端
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;

    // 状态变量 (ENU坐标系)
    Eigen::Vector3d quad_position_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d quad_velocity_enu_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quad_attitude_enu_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d payload_position_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d payload_velocity_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_payload_position_ = Eigen::Vector3d(0.0, 0.0, 0.8);
    Eigen::Quaterniond goal_attitude_ = Eigen::Quaterniond::Identity();

    // NMPC状态和控制
    NMPCState current_state_;
    NMPCControl current_control_;

    // ESDF地图读取器
    std::shared_ptr<ESDFMapReader> esdf_reader_;

    // MAVROS状态
    mavros_msgs::msg::State mavros_state_;

    // 数据有效性标志
    bool quad_pose_received_ = false;
    bool quad_velocity_received_ = false;
    bool payload_odom_received_ = false;
    bool all_data_ready_ = false;
    bool goal_pose_received_ = false;

    // Offboard模式管理
    bool offboard_enabled_ = false;
    rclcpp::Time offboard_start_time_;

    // acados求解器
    px4_payload_model_solver_capsule* acados_ocp_capsule_ = nullptr;
    ocp_nlp_config* nlp_config_ = nullptr;
    ocp_nlp_dims* nlp_dims_ = nullptr;
    ocp_nlp_in* nlp_in_ = nullptr;
    ocp_nlp_out* nlp_out_ = nullptr;
    ocp_nlp_solver* nlp_solver_ = nullptr;
    void* nlp_opts_ = nullptr;

    // 系统参数
    double cable_length_;
    double quad_mass_;
    double payload_mass_;
    double gravity_;
    double control_frequency_;
    double offboard_delay_;

    // 圆形轨迹参数
    const double circle_radius_ = 2.0;          // 圆形轨迹半径 (m)
    const double circle_angular_velocity_ = 1.0; // 圆形轨迹角速度 (rad/s)
    const double circle_center_x_ = 0.0;        // 圆心X坐标 (m, ENU)
    const double circle_center_y_ = 0.0;        // 圆心Y坐标 (m, ENU)
    const double circle_height_ = 3.0;          // 圆形轨迹高度 (m)

    // 时间跟踪
    rclcpp::Time trajectory_start_time_;
    bool trajectory_started_ = false;

    // 统计信息
    int solve_count_ = 0;
    int solve_success_count_ = 0;
    double avg_solve_time_ms_ = 0.0;
    int print_counter_ = 0;
    bool first_solve_done_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<NMPCController>();
        node->initializeEsdfReader();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("nmpc_controller"),
                     "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
