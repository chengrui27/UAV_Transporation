/**
 * @file nmpc_controller.cpp
 * @brief PX4四旋翼NMPC控制器节点
 *
 * 实现功能:
 * 1. 读取PX4状态信息
 * 2. 调用acados求解器解算控制输入
 * 3. 通过Offboard模式Body Rate控制无人机
 *
 * 坐标系说明:
 * - World Frame: ENU (East-North-Up)
 * - Body Frame: FLU (Forward-Left-Up)
 *
 * @author PX4 NMPC Team
 * @date 2025-10-18
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <px4ctrl/msg/position_command.hpp>
#include <px4ctrl/msg/position_command_trajectory.hpp>

#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <cmath>

// ESDF地图读取器
#include "px4_nmpc/esdf_map_reader.h"

// acados generated solver interface
extern "C" {
    #include "acados_c/ocp_nlp_interface.h"
    #include "acados_solver_px4_model.h"
}

using namespace std::chrono_literals;

/**
 * @brief NMPC控制器类
 */
class NMPCController : public rclcpp::Node {
public:
    NMPCController() : Node("nmpc_controller") {
        // 声明参数
        this->declare_parameter("quad_mass", 1.535);           // 四旋翼质量 (kg)
        this->declare_parameter("control_frequency", 10.0);   // 控制频率 (Hz)
        this->declare_parameter("offboard_delay", 2.0);        // Offboard延迟 (s)

        // 获取参数
        quad_mass_ = this->get_parameter("quad_mass").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        offboard_delay_ = this->get_parameter("offboard_delay").as_double();
        gravity_ = 9.81;

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "NMPC Controller Node Initialized");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Quad mass: %.2f kg", quad_mass_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.0f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "  Offboard delay: %.1f s", offboard_delay_);

        // 设置QoS为BEST_EFFORT以匹配MAVROS
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // 初始化订阅者
        quad_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_best_effort,
            std::bind(&NMPCController::quadPoseCallback, this, std::placeholders::_1));

        quad_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos_best_effort,
            std::bind(&NMPCController::quadVelocityCallback, this, std::placeholders::_1));

        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_best_effort,
            std::bind(&NMPCController::mavrosStateCallback, this, std::placeholders::_1));

        // 订阅RViz发布的目标位置
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&NMPCController::goalPoseCallback, this, std::placeholders::_1));

        // 初始化发布者
        attitude_target_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);

        predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/nmpc/predicted_path", 10);
        position_cmd_traj_pub_ = this->create_publisher<px4ctrl::msg::PositionCommandTrajectory>(
            "/cmd_traj", 10);

        // 初始化服务客户端
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");

        // 初始化acados求解器
        initializeAcadosSolver();

        // 创建控制定时器
        auto timer_period = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / control_frequency_));
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&NMPCController::controlTimerCallback, this));

        // 创建预测轨迹发布定时器 (0.5秒 = 2Hz)
        path_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NMPCController::publishPredictedPath, this));

        RCLCPP_INFO(this->get_logger(), "Waiting for data from PX4...");
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
        RCLCPP_INFO(this->get_logger(), "✓ ESDF Map Reader initialized");
    }

private:
    // ========== 回调函数 ==========

    /**
     * @brief 四旋翼位置姿态回调 (ENU坐标系)
     */
    void quadPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        quad_position_ << msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z;

        quad_attitude_.w() = msg->pose.orientation.w;
        quad_attitude_.x() = msg->pose.orientation.x;
        quad_attitude_.y() = msg->pose.orientation.y;
        quad_attitude_.z() = msg->pose.orientation.z;

        if (!quad_pose_received_) {
            quad_pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Quadrotor pose received");
        }
    }

    /**
     * @brief 四旋翼速度回调 (ENU坐标系)
     */
    void quadVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        quad_velocity_ << msg->twist.linear.x,
                          msg->twist.linear.y,
                          msg->twist.linear.z;

        if (!quad_velocity_received_) {
            quad_velocity_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ Quadrotor velocity received");
        }

        // 检查是否所有数据都已接收
        if(!all_data_ready_){
            if (quad_pose_received_ && quad_velocity_received_) {
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
     * @brief 目标位置姿态回调 (从RViz)
     */
    int  traj_cnt = 0;  //规划轨迹次数
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 保存参考位置
        goal_position_ << msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z;
        goal_position_(2) = 1.0; // 高度固定为1.0米

        // 保存参考姿态（四元数）
        goal_attitude_.w() = msg->pose.orientation.w;
        goal_attitude_.x() = msg->pose.orientation.x;
        goal_attitude_.y() = msg->pose.orientation.y;
        goal_attitude_.z() = msg->pose.orientation.z;

        // 归一化四元数
        goal_attitude_.normalize();

        // 标记已接收到目标点,更新重规划相关标志位
        goal_pose_received_ = true;
        first_solve_done_ = false;
        traj_cnt = 0;
        internal_state_initialized_ = false;

        RCLCPP_INFO(this->get_logger(), "New goal received:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f] m",
                    goal_position_(0), goal_position_(1), goal_position_(2));
        RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
                    goal_attitude_.w(), goal_attitude_.x(),
                    goal_attitude_.y(), goal_attitude_.z());

        // 查询目标点在ESDF地图中的距离和梯度，方便调试避障行为
        if (esdf_reader_ && esdf_reader_->isMapValid()) {
            double esdf_dist = esdf_reader_->getDistance(goal_position_);
            Eigen::Vector3d esdf_grad = esdf_reader_->getGradient(goal_position_);
            RCLCPP_INFO(this->get_logger(),
                        "ESDF at goal: dist=%.3f m, grad=[%.3f, %.3f, %.3f]",
                        esdf_dist, esdf_grad.x(), esdf_grad.y(), esdf_grad.z());
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "ESDF map not valid when goal received, cannot query dist/grad");
        }

        // 重置ESDF相关状态：下次求解时从“干净”的参数和内部状态开始    
        resetAcadosStateForNewGoal();
    }

    /**
     * @brief 控制定时器回调
     */
    void controlTimerCallback() {
        // 检查数据是否就绪
        if (!all_data_ready_) {
            return;
        }

        // // 检查是否需要启用Offboard模式
        // if (!offboard_enabled_) {
        //     auto elapsed = (this->now() - offboard_start_time_).seconds();

        //     if (elapsed >= offboard_delay_) {
        //         enableOffboardMode();
        //         armVehicle();
        //         offboard_enabled_ = true;
        //         RCLCPP_INFO(this->get_logger(), "========================================");
        //         RCLCPP_INFO(this->get_logger(), "Offboard mode enabled! Starting NMPC control...");
        //         RCLCPP_INFO(this->get_logger(), "========================================");
        //     }
        //     return;
        // }

        // 接收到新的目标点后迭代求解10次MPC轨迹，轨迹收敛需要一定次数的迭代
        if(traj_cnt > 10){
            return;
        }

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

            // 发布轨迹消息
            if(traj_cnt == 10){
                publishPositionCommandTrajectory();
            }
            traj_cnt++;        

            // 计算MPC平均求解时间和成功率，每100次打印一次统计
            if (solve_count_ % 100 == 0) {
                double success_rate = 100.0 * solve_success_count_ / solve_count_;
                RCLCPP_INFO(this->get_logger(),
                    "NMPC Stats: Success rate: %.1f%%, Avg solve time: %.2f ms",
                    success_rate, avg_solve_time_ms_);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "NMPC solver failed!");
        }
    }

    /**
     * @brief 生成悬停参考轨迹
     * @param yref 输出参考向量 (22维: 17维状态 + 4维控制 + 1维避障代价项)
     * @param yref_e 输出终端参考向量 (17维: 仅状态)
     */
    void generateHoverReference(double* yref, double* yref_e) {
        // 参考位置 (ENU坐标系) - 使用来自RViz的目标位置
        yref[0] = goal_position_(0);   // East (m)
        yref[1] = goal_position_(1);   // North (m)
        yref[2] = goal_position_(2);   // Up (m)

        // 参考速度 (ENU坐标系)
        yref[3] = 0.0;   // vE
        yref[4] = 0.0;   // vN
        yref[5] = 0.0;   // vU

        // 参考四元数 (ENU坐标系) - 使用来自RViz的目标姿态
        yref[6] = goal_attitude_.w();  // qw
        yref[7] = goal_attitude_.x();  // qx
        yref[8] = goal_attitude_.y();  // qy
        yref[9] = goal_attitude_.z();  // qz

        // 参考扩展状态
        double hover_thrust = quad_mass_ * gravity_;  // 悬停推力
        yref[10] = hover_thrust;  // T (N)
        yref[11] = 0.0;           // ωx (rad/s)
        yref[12] = 0.0;           // ωy (rad/s)
        yref[13] = 0.0;           // ωz (rad/s)
        yref[14] = 0.0;           // d_ωx
        yref[15] = 0.0;           // d_ωy
        yref[16] = 0.0;           // d_ωz

        // 参考控制输入 (dT, ddωx, ddωy, ddωz)
        yref[17] = 0.0;
        yref[18] = 0.0;
        yref[19] = 0.0;
        yref[20] = 0.0;

        // 避障代价项参考值
        yref[21] = 0.0;

        // 终端参考 (仅状态, 17维)
        for (int i = 0; i < 17; i++) {
            yref_e[i] = yref[i];
        }
    }

    /**
     * @brief 新目标触发后的初始猜测（线性插值位置 + 悬停推力）
     */
    void initializeGuessForNewGoal(const Eigen::Quaterniond& current_attitude) {
        if (acados_ocp_capsule_ == nullptr) {
            return;
        }

        int N = PX4_MODEL_N;
        double hover_thrust = quad_mass_ * gravity_;

        for (int i = 0; i <= N; ++i) {
            double alpha = static_cast<double>(i) / static_cast<double>(N);

            double x_guess[PX4_MODEL_NX];
            for (int k = 0; k < PX4_MODEL_NX; ++k) {
                x_guess[k] = 0.0;
            }

            x_guess[0] = (1.0 - alpha) * quad_position_(0) + alpha * goal_position_(0);
            x_guess[1] = (1.0 - alpha) * quad_position_(1) + alpha * goal_position_(1);
            x_guess[2] = (1.0 - alpha) * quad_position_(2) + alpha * goal_position_(2);

            x_guess[3] = 0.0;
            x_guess[4] = 0.0;
            x_guess[5] = 0.0;

            x_guess[6] = current_attitude.w();
            x_guess[7] = current_attitude.x();
            x_guess[8] = current_attitude.y();
            x_guess[9] = current_attitude.z();

            x_guess[10] = hover_thrust;
            x_guess[11] = 0.0;
            x_guess[12] = 0.0;
            x_guess[13] = 0.0;
            x_guess[14] = 0.0;
            x_guess[15] = 0.0;
            x_guess[16] = 0.0;

            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_guess);

            if (i < N) {
                double u_guess[PX4_MODEL_NU];
                for (int k = 0; k < PX4_MODEL_NU; ++k) {
                    u_guess[k] = 0.0;
                }
                ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_guess);
            }
        }
    }

    /**
     * @brief 读取MPC每个stage的状态，构建PositionCommand消息
     */
    px4ctrl::msg::PositionCommand buildPositionCommandFromStage(const double* x) {
        px4ctrl::msg::PositionCommand cmd;

        Eigen::Quaterniond att(x[6], x[7], x[8], x[9]);
        att = normalizeQuaternionSign(att);
        Eigen::Matrix3d R_body_to_enu = att.toRotationMatrix();
        Eigen::Vector3d r3 = R_body_to_enu.col(2);

        double thrust = x[10];
        Eigen::Vector3d accel_enu = (thrust / quad_mass_) * r3 - Eigen::Vector3d(0.0, 0.0, gravity_);
        Eigen::Vector3d thrust_enu = thrust * r3;

        cmd.position.x = x[0];
        cmd.position.y = x[1];
        cmd.position.z = x[2];

        cmd.velocity.x = x[3];
        cmd.velocity.y = x[4];
        cmd.velocity.z = x[5];

        cmd.acceleration.x = accel_enu.x();
        cmd.acceleration.y = accel_enu.y();
        cmd.acceleration.z = accel_enu.z();

        cmd.thrust.x = thrust_enu.x();
        cmd.thrust.y = thrust_enu.y();
        cmd.thrust.z = thrust_enu.z();

        cmd.yaw = yawFromQuat(att);
        cmd.yaw_rate = x[13];

        return cmd;
    }

    /**
     * @brief 求解NMPC
     */
    bool solveNMPC() {
        if (acados_ocp_capsule_ == nullptr) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Acados solver not initialized!");
            return false;
        }

        // 归一化四元数
        Eigen::Quaterniond quad_attitude_normalized = normalizeQuaternionSign(quad_attitude_);

        // 首次求解时初始化部分内部状态
        if (!internal_state_initialized_) {
            thrust_state_ = quad_mass_ * gravity_;
            body_rate_.setZero();
            body_rate_dot_.setZero();
            initializeGuessForNewGoal(quad_attitude_normalized);
            internal_state_initialized_ = true;
        }

        // 准备状态向量 (17维)
        double x0[17];
        x0[0] = quad_position_(0);
        x0[1] = quad_position_(1);
        x0[2] = quad_position_(2);
        x0[3] = quad_velocity_(0);
        x0[4] = quad_velocity_(1);
        x0[5] = quad_velocity_(2);
        x0[6] = quad_attitude_normalized.w();
        x0[7] = quad_attitude_normalized.x();
        x0[8] = quad_attitude_normalized.y();
        x0[9] = quad_attitude_normalized.z();
        x0[10] = thrust_state_;
        x0[11] = body_rate_(0);
        x0[12] = body_rate_(1);
        x0[13] = body_rate_(2);
        x0[14] = body_rate_dot_(0);
        x0[15] = body_rate_dot_(1);
        x0[16] = body_rate_dot_(2);

        // 设置初始状态约束
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      0, "ubx", x0);

        // 生成参考轨迹
        double yref[22];      // 状态(17) + 控制(4) + 避障代价项(1)
        double yref_e[17];    // 终端状态
        generateHoverReference(yref, yref_e);

        // 设置所有预测阶段的参考轨迹 (0 到 N-1)
        int N = PX4_MODEL_N;  // 预测时域长度 (与生成的acados求解器保持一致)
        for (int i = 0; i < N; i++) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);
        }

        // 设置终端阶段的参考 (N)
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "yref", yref_e);

        // // ========== 打印当前位置的ESDF避障信息 ==========
        // if (esdf_reader_ && esdf_reader_->isMapValid()) {
        //     Eigen::Vector3d current_pos(x0[0], x0[1], x0[2]);
        //     double current_dist = esdf_reader_->getDistance(current_pos);
        //     bool in_bound = esdf_reader_->isInLocalBound(current_pos);

        //     if (solve_count_ % 100 == 0) {
        //         RCLCPP_INFO(this->get_logger(),
        //             "Current pos: [%.2f, %.2f, %.2f], ESDF dist: %.3f m, in_bound: %d",
        //             current_pos.x(), current_pos.y(), current_pos.z(), current_dist, in_bound);
        //     }
        // }

        // 第一次求解时跳过参数更新（使用默认参数）
        if (!first_solve_done_) {
            RCLCPP_INFO(this->get_logger(), "First solve: using default parameters");
            first_solve_done_ = true;
        } else if (esdf_reader_ && esdf_reader_->isMapValid()) {
            // 从第二次求解开始，为每个预测阶段查询ESDF并更新参数（使用上一轮预测轨迹）
            for (int i = 0; i < N; i++) {
                // 1. 提取该阶段的猜测状态
                double xi[17];
                ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

                // 2. 查询ESDF距离和梯度
                Eigen::Vector3d pos(xi[0], xi[1], xi[2]);
                double esdf_dist;
                Eigen::Vector3d esdf_grad;
                esdf_reader_->getDistanceAndGradient(pos, esdf_dist, esdf_grad);

                // 3. 构造参数向量 [d_ref, gx, gy, gz, x_ref, y_ref, z_ref]
                double p[7];
                p[0] = esdf_dist;      // 距离
                p[1] = esdf_grad.x();  // 梯度x
                p[2] = esdf_grad.y();  // 梯度y
                p[3] = esdf_grad.z();  // 梯度z
                p[4] = pos.x();        // 查询点x
                p[5] = pos.y();        // 查询点y
                p[6] = pos.z();        // 查询点z

                // 4. 更新acados参数
                int ret = px4_model_acados_update_params(acados_ocp_capsule_, i, p, 7);
                if (ret != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to update params at stage %d, ret=%d", i, ret);
                }
            }
        }

        // ========== 多次内层迭代：检查预测是否碰撞，必要时重新线性化ESDF并再次求解 ==========
        const int max_inner_iters = 10;
        const double safety_distance_check = 0.4;  // 与px4_nmpc.py中的安全距离保持一致

        bool solver_ok = false;

        for (int iter = 0; iter < max_inner_iters; ++iter) {
            // 调用求解器
            int status = px4_model_acados_solve(acados_ocp_capsule_);
            if (status != 0) {
                internal_state_initialized_ = false;
                RCLCPP_WARN(this->get_logger(),
                            "Acados solve failed at inner iter %d with status %d", iter, status);
            }else{
                solver_ok = true;
            }

            // 如果ESDF不可用，无法判断碰撞，直接使用当前解
            if (!(esdf_reader_ && esdf_reader_->isMapValid())) {
                break;
            }

            // 基于当前预测轨迹检查是否发生“碰撞”（距离小于安全距离）
            bool has_collision = false;
            double min_dist = 1e9;

            for (int k = 0; k <= N; ++k) {
                double x_pred[17];
                ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "x", x_pred);

                Eigen::Vector3d pos(x_pred[0], x_pred[1], x_pred[2]);
                double dist = esdf_reader_->getDistance(pos);
                min_dist = std::min(min_dist, dist);

                if (dist < safety_distance_check) {
                    has_collision = true;
                    break;
                }
            }

            // 预测轨迹在整个时域内均满足安全距离，结束内层迭代，跳出循环
            if (!has_collision) {
                break;
            }

            // 若预测轨迹仍存在碰撞且未达到最大迭代次数：基于当前预测轨迹重新线性化ESDF并更新参数，进入下一轮求解
            if (iter < max_inner_iters - 1) {
                for (int i = 0; i < N; ++i) {
                    double xi[17];
                    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

                    Eigen::Vector3d pos(xi[0], xi[1], xi[2]);
                    double esdf_dist;
                    Eigen::Vector3d esdf_grad;
                    esdf_reader_->getDistanceAndGradient(pos, esdf_dist, esdf_grad);

                    double p[7];
                    p[0] = esdf_dist;
                    p[1] = esdf_grad.x();
                    p[2] = esdf_grad.y();
                    p[3] = esdf_grad.z();
                    p[4] = pos.x();
                    p[5] = pos.y();
                    p[6] = pos.z();

                    int ret = px4_model_acados_update_params(acados_ocp_capsule_, i, p, 7);
                }
                // 进入下一轮 inner iter
            } else {
                // 已达到最大迭代次数，使用当前解但发出警告
                RCLCPP_WARN(this->get_logger(),
                            "NMPC predicted trajectory still colliding after %d inner iters "
                            "(min_dist=%.3f m), using last solution",
                            max_inner_iters, min_dist);
            }
        }

        if (!solver_ok) {
            return false;
        }

        // 使用stage1状态作为下一时刻的初始值
        double x_next[17];
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", x_next);
        thrust_state_ = x_next[10];
        body_rate_(0) = x_next[11];
        body_rate_(1) = x_next[12];
        body_rate_(2) = x_next[13];
        body_rate_dot_(0) = x_next[14];
        body_rate_dot_(1) = x_next[15];
        body_rate_dot_(2) = x_next[16];

        // 保存控制输入用于发布
        current_control_.thrust = thrust_state_;
        current_control_.body_rate_x = body_rate_(0);
        current_control_.body_rate_y = body_rate_(1);
        current_control_.body_rate_z = body_rate_(2);

        // // 每10次打印一次，避免刷屏
        // print_counter_++;
        // if (print_counter_ % 10 == 0) {
        //     RCLCPP_INFO(this->get_logger(), "========== NMPC State & Control ==========");
        //     RCLCPP_INFO(this->get_logger(), "Position (ENU): [%.3f, %.3f, %.3f] m",
        //         x0[0], x0[1], x0[2]);
        //     RCLCPP_INFO(this->get_logger(), "Velocity (ENU): [%.3f, %.3f, %.3f] m/s",
        //         x0[3], x0[4], x0[5]);
        //     RCLCPP_INFO(this->get_logger(), "Quaternion (ENU): [%.3f, %.3f, %.3f, %.3f]",
        //         x0[6], x0[7], x0[8], x0[9]);

        //     ESDF信息（暂时注释，避免段错误）
        //     if (esdf_reader_ && esdf_reader_->isMapValid()) {
        //         Eigen::Vector3d current_pos(x0[0], x0[1], x0[2]);
        //         double esdf_dist = esdf_reader_->getDistance(current_pos);
        //         Eigen::Vector3d esdf_grad = esdf_reader_->getGradient(current_pos);
        //         RCLCPP_INFO(this->get_logger(), "ESDF: dist=%.3f m, grad=[%.3f, %.3f, %.3f]",
        //             esdf_dist, esdf_grad.x(), esdf_grad.y(), esdf_grad.z());
        //     } else {
        //         RCLCPP_WARN(this->get_logger(), "ESDF map not available");
        //     }

        //     RCLCPP_INFO(this->get_logger(), "NMPC Output:");
        //     RCLCPP_INFO(this->get_logger(), "  Thrust: %.3f N", u0[0]);
        //     RCLCPP_INFO(this->get_logger(), "  Body Rate: [%.3f, %.3f, %.3f] rad/s",
        //         u0[1], u0[2], u0[3]);
        //     RCLCPP_INFO(this->get_logger(), "Reference:");
        //     RCLCPP_INFO(this->get_logger(), "  Pos: [%.3f, %.3f, %.3f] m", yref[0], yref[1], yref[2]);
        //     RCLCPP_INFO(this->get_logger(), "  Thrust: %.3f N", yref[10]);
        //     RCLCPP_INFO(this->get_logger(), "==========================================");
        // }

        return true;
    }

    /**
     * @brief 发布PositionCommandTrajectory
     */
    void publishPositionCommandTrajectory() {

        px4ctrl::msg::PositionCommandTrajectory msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.dt = 0.1;

        int N = PX4_MODEL_N;
        msg.points.reserve(N + 1);
        for (int i = 0; i <= N; ++i) {
            double x_stage[PX4_MODEL_NX];
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_stage);
            msg.points.push_back(buildPositionCommandFromStage(x_stage));
        }

        position_cmd_traj_pub_->publish(msg);
    }

    /**
     * @brief 发布Body Rate控制指令(已经将MPC控制器改为规划器，此函数已弃用)
     */
    void publishBodyRateControl() {
        auto msg = mavros_msgs::msg::AttitudeTarget();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";

        // 设置控制模式: Body Rate + Thrust
        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

        // 设置Body Rate (rad/s, body frame)
        msg.body_rate.x = current_control_.body_rate_x;
        msg.body_rate.y = current_control_.body_rate_y;
        msg.body_rate.z = current_control_.body_rate_z;

        // 设置推力 (归一化 0-1)
        double max_thrust = 22.629;//23.298;  // 最大推力 (N)
        double thrust_normalized = current_control_.thrust / max_thrust;

        // 限制范围
        thrust_normalized = std::max(-1.0, std::min(1.0, thrust_normalized));
        msg.thrust = thrust_normalized;

        attitude_target_pub_->publish(msg);
    }

    /**
     * @brief 发布NMPC预测轨迹 (每0.5秒)
     */
    void publishPredictedPath() {
        if (acados_ocp_capsule_ == nullptr || !all_data_ready_) {
            return;
        }

        // 创建Path消息
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        // 预测时域长度
        int N = PX4_MODEL_N;  // 与生成的acados求解器保持一致

        // 提取预测轨迹 (0到N共21个点)
        for (int i = 0; i <= N; i++) {
            double x_pred[17];
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_pred);

            // 创建PoseStamped
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";

            // 设置位置
            pose.pose.position.x = x_pred[0];  // East
            pose.pose.position.y = x_pred[1];  // North
            pose.pose.position.z = x_pred[2];  // Up

            // 设置姿态（虽然Path主要显示位置，但也保存姿态信息）
            pose.pose.orientation.w = x_pred[6];
            pose.pose.orientation.x = x_pred[7];
            pose.pose.orientation.y = x_pred[8];
            pose.pose.orientation.z = x_pred[9];

            path_msg.poses.push_back(pose);
        }

        // 发布
        predicted_path_pub_->publish(path_msg);
    }

    // ========== 四元数工具函数 ==========

    /**
     * @brief 归一化四元数符号
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

    double yawFromQuat(const Eigen::Quaterniond& q) const {
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return std::atan2(siny_cosp, cosy_cosp);
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
        acados_ocp_capsule_ = px4_model_acados_create_capsule();

        if (acados_ocp_capsule_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create acados capsule!");
            return;
        }

        // 创建求解器
        int status = px4_model_acados_create(acados_ocp_capsule_);

        if (status != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create acados solver! Status: %d", status);
            return;
        }

        // 获取求解器组件
        nlp_config_ = px4_model_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = px4_model_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_in_ = px4_model_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = px4_model_acados_get_nlp_out(acados_ocp_capsule_);
        nlp_solver_ = px4_model_acados_get_nlp_solver(acados_ocp_capsule_);
        nlp_opts_ = px4_model_acados_get_nlp_opts(acados_ocp_capsule_);

        RCLCPP_INFO(this->get_logger(), "✓ Acados solver initialized successfully");
    }

    /**
     * @brief 在接收到新目标点时重置acados内部状态和ESDF参数
     *
     * - 将运行时参数初始化为: 距离=10, 梯度=0
     * - 调用 px4_model_acados_reset 重置求解器内部状态
     */
    void resetAcadosStateForNewGoal() {
        if (acados_ocp_capsule_ == nullptr) {
            return;
        }

        // 重置求解器内部状态和QP求解器内存
        int reset_status = px4_model_acados_reset(acados_ocp_capsule_, 1);
        if (reset_status != 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to reset acados solver, status=%d", reset_status);
        }

        // 使用当前位置作为线性化参考点，距离设为10，梯度设为0
        double p[7];
        p[0] = 10.0;                // 距离
        p[1] = 0.0;                 // grad_x
        p[2] = 0.0;                 // grad_y
        p[3] = 0.0;                 // grad_z
        p[4] = quad_position_(0);   // x_ref
        p[5] = quad_position_(1);   // y_ref
        p[6] = quad_position_(2);   // z_ref

        // 预测时域长度，与生成的acados求解器保持一致
        int N = PX4_MODEL_N;
        for (int i = 0; i < N; ++i) {
            int ret = px4_model_acados_update_params(acados_ocp_capsule_, i, p, 7);
            if (ret != 0) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to init params at stage %d when resetting goal, ret=%d",
                            i, ret);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Acados state and ESDF params reset for new goal");
    }

    /**
     * @brief 清理acados求解器
     */
    void cleanupAcadosSolver() {
        if (acados_ocp_capsule_ != nullptr) {
            px4_model_acados_free(acados_ocp_capsule_);
            px4_model_acados_free_capsule(acados_ocp_capsule_);
            RCLCPP_INFO(this->get_logger(), "Acados solver cleaned up");
        }
    }

    // ========== 数据结构 ==========

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
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    // ROS2发布者
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<px4ctrl::msg::PositionCommandTrajectory>::SharedPtr position_cmd_traj_pub_;

    // ROS2服务客户端
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;

    // ESDF地图读取器
    std::shared_ptr<ESDFMapReader> esdf_reader_;

    // 状态变量 (ENU坐标系)
    Eigen::Vector3d quad_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d quad_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quad_attitude_ = Eigen::Quaterniond::Identity();

    // 目标位置和姿态 (来自RViz)
    Eigen::Vector3d goal_position_ = Eigen::Vector3d(0.0, 0.0, 1.0);  // 默认悬停在1米高度
    Eigen::Quaterniond goal_attitude_ = Eigen::Quaterniond::Identity();
    bool goal_pose_received_ = false;

    // NMPC控制
    NMPCControl current_control_;
    double thrust_state_ = 0.0;
    Eigen::Vector3d body_rate_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rate_dot_ = Eigen::Vector3d::Zero();
    bool internal_state_initialized_ = false;

    // MAVROS状态
    mavros_msgs::msg::State mavros_state_;

    // 数据有效性标志
    bool quad_pose_received_ = false;
    bool quad_velocity_received_ = false;
    bool all_data_ready_ = false;

    // Offboard模式管理
    bool offboard_enabled_ = false;
    rclcpp::Time offboard_start_time_;

    // acados求解器
    px4_model_solver_capsule* acados_ocp_capsule_ = nullptr;
    ocp_nlp_config* nlp_config_ = nullptr;
    ocp_nlp_dims* nlp_dims_ = nullptr;
    ocp_nlp_in* nlp_in_ = nullptr;
    ocp_nlp_out* nlp_out_ = nullptr;
    ocp_nlp_solver* nlp_solver_ = nullptr;
    void* nlp_opts_ = nullptr;

    // 系统参数
    double quad_mass_;
    double gravity_;
    double control_frequency_;
    double offboard_delay_;

    // 统计信息
    int solve_count_ = 0;
    int solve_success_count_ = 0;
    double avg_solve_time_ms_ = 0.0;
    int print_counter_ = 0;

    // ESDF避障标志
    bool first_solve_done_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<NMPCController>();

        // 在节点创建后初始化ESDF地图读取器
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
