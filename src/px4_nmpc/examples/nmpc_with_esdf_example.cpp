/**
 * @file nmpc_with_esdf_example.cpp
 * @brief NMPC控制器集成ESDF避障示例
 *
 * 展示如何在NMPC中使用ESDF距离和梯度信息进行避障
 */

#include <rclcpp/rclcpp.hpp>
#include "px4_nmpc/esdf_map_reader.h"
#include <Eigen/Dense>

// ACADOS求解器接口
extern "C" {
    #include "acados_c/ocp_nlp_interface.h"
    #include "acados_solver_px4_model.h"
}

class NMPCWithESDFExample : public rclcpp::Node {
public:
    NMPCWithESDFExample() : Node("nmpc_esdf_example") {
        // 初始化ESDF地图读取器
        esdf_reader_ = std::make_shared<ESDFMapReader>(shared_from_this());

        // 初始化ACADOS求解器
        acados_ocp_capsule_ = px4_model_acados_create_capsule();
        int status = px4_model_acados_create(acados_ocp_capsule_);

        if (status) {
            RCLCPP_ERROR(get_logger(), "Failed to create ACADOS solver");
            return;
        }

        // 获取求解器配置
        nlp_config_ = px4_model_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = px4_model_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_in_ = px4_model_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = px4_model_acados_get_nlp_out(acados_ocp_capsule_);

        N_ = 20;  // 预测步数

        RCLCPP_INFO(get_logger(), "NMPC with ESDF initialized");
    }

    ~NMPCWithESDFExample() {
        if (acados_ocp_capsule_) {
            px4_model_acados_free(acados_ocp_capsule_);
            px4_model_acados_free_capsule(acados_ocp_capsule_);
        }
    }

    /**
     * @brief 方法1: 使用ESDF作为非线性约束
     *
     * 在ACADOS中设置约束: distance >= safe_distance
     */
    void updateESDFConstraints(const Eigen::Vector3d& current_pos) {
        if (!esdf_reader_->isMapValid()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "ESDF map not valid yet");
            return;
        }

        // 为预测时域中的每一步更新ESDF参数
        for (int i = 0; i < N_; i++) {
            // 获取第i步的预测位置
            double* xi = new double[10];  // 状态维度: [x,y,z,vx,vy,vz,qw,qx,qy,qz]
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

            Eigen::Vector3d pred_pos(xi[0], xi[1], xi[2]);

            // 查询ESDF距离和梯度
            double dist;
            Eigen::Vector3d grad;
            esdf_reader_->getDistanceAndGradient(pred_pos, dist, grad);

            // 设置参数: [distance, grad_x, grad_y, grad_z]
            double p[4] = {dist, grad(0), grad(1), grad(2)};
            px4_model_acados_update_params(acados_ocp_capsule_, i, p, 4);

            delete[] xi;

            RCLCPP_DEBUG(get_logger(),
                        "Stage %d: pos=[%.2f,%.2f,%.2f], dist=%.3f, grad=[%.3f,%.3f,%.3f]",
                        i, pred_pos(0), pred_pos(1), pred_pos(2),
                        dist, grad(0), grad(1), grad(2));
        }
    }

    /**
     * @brief 方法2: 使用ESDF梯度修改参考轨迹(势场法)
     *
     * 当距离过近时,在参考轨迹上叠加排斥力
     */
    void updateReferenceWithRepulsiveForce(const Eigen::Vector3d& current_pos) {
        if (!esdf_reader_->isMapValid()) return;

        double dist;
        Eigen::Vector3d grad;
        esdf_reader_->getDistanceAndGradient(current_pos, dist, grad);

        const double repulsive_threshold = 2.0;  // 排斥力作用范围(米)
        const double k_repulsive = 1.0;          // 排斥力增益

        if (dist < repulsive_threshold) {
            // 计算排斥力
            grad.normalize();
            double force_magnitude = k_repulsive * (1.0/dist - 1.0/repulsive_threshold);
            Eigen::Vector3d repulsive_force = force_magnitude * grad;

            RCLCPP_INFO(get_logger(),
                       "Obstacle nearby! dist=%.3f, repulsive_force=[%.3f,%.3f,%.3f]",
                       dist, repulsive_force(0), repulsive_force(1), repulsive_force(2));

            // 修改参考轨迹
            double dt = 0.05;  // 时间步长
            for (int i = 0; i < N_; i++) {
                double yref[14];  // [x,y,z,vx,vy,vz,qw,qx,qy,qz,thrust,wx,wy,wz]
                ocp_nlp_cost_model_get(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);

                // 在参考位置上叠加排斥力
                yref[0] += repulsive_force(0) * dt * i;
                yref[1] += repulsive_force(1) * dt * i;
                yref[2] += repulsive_force(2) * dt * i;

                ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);
            }
        }
    }

    /**
     * @brief 方法3: 使用ESDF距离作为代价函数惩罚项
     *
     * 在代价函数中添加障碍物惩罚: cost = exp(-dist/sigma)
     */
    double computeObstacleCost(const Eigen::Vector3d& pos) {
        if (!esdf_reader_->isMapValid()) return 0.0;

        double dist = esdf_reader_->getDistance(pos);

        const double safe_distance = 0.5;
        const double sigma = 0.3;

        // 指数惩罚
        double cost = std::exp(-dist / sigma);

        // 或使用软约束惩罚
        // double cost = std::max(0.0, safe_distance - dist);

        return cost;
    }

    /**
     * @brief 完整的NMPC求解循环
     */
    void solveNMPC(const Eigen::Vector3d& current_pos,
                   const Eigen::Vector3d& current_vel,
                   const Eigen::Vector4d& current_quat) {

        // 1. 更新ESDF约束/参考轨迹
        updateESDFConstraints(current_pos);
        // 或者使用势场法
        // updateReferenceWithRepulsiveForce(current_pos);

        // 2. 设置初始状态
        double x0[10] = {
            current_pos(0), current_pos(1), current_pos(2),
            current_vel(0), current_vel(1), current_vel(2),
            current_quat(0), current_quat(1), current_quat(2), current_quat(3)
        };
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x0);

        // 3. 求解OCP
        int status = px4_model_acados_solve(acados_ocp_capsule_);

        if (status != 0) {
            RCLCPP_WARN(get_logger(), "ACADOS solver failed with status %d", status);
            return;
        }

        // 4. 获取最优控制输入
        double u0[4];  // [thrust, wx, wy, wz]
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0);

        RCLCPP_INFO(get_logger(),
                   "Optimal control: thrust=%.3f, body_rates=[%.3f,%.3f,%.3f]",
                   u0[0], u0[1], u0[2], u0[3]);

        // 5. 检查预测轨迹是否安全
        checkPredictedTrajectory();
    }

    /**
     * @brief 检查预测轨迹是否与障碍物碰撞
     */
    void checkPredictedTrajectory() {
        if (!esdf_reader_->isMapValid()) return;

        const double safe_distance = 0.5;
        bool collision_detected = false;

        for (int i = 0; i < N_; i++) {
            double* xi = new double[10];
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xi);

            Eigen::Vector3d pred_pos(xi[0], xi[1], xi[2]);
            double dist = esdf_reader_->getDistance(pred_pos);

            if (dist < safe_distance) {
                RCLCPP_WARN(get_logger(),
                           "Collision risk at stage %d: pos=[%.2f,%.2f,%.2f], dist=%.3f",
                           i, pred_pos(0), pred_pos(1), pred_pos(2), dist);
                collision_detected = true;
            }

            delete[] xi;
        }

        if (!collision_detected) {
            RCLCPP_INFO(get_logger(), "Predicted trajectory is safe");
        }
    }

private:
    std::shared_ptr<ESDFMapReader> esdf_reader_;

    // ACADOS求解器
    px4_model_solver_capsule* acados_ocp_capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;

    int N_;  // 预测步数
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NMPCWithESDFExample>();

    // 测试示例
    Eigen::Vector3d pos(0.0, 0.0, 1.0);
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    Eigen::Vector4d quat(1.0, 0.0, 0.0, 0.0);

    rclcpp::Rate rate(10);  // 10Hz
    while (rclcpp::ok()) {
        node->solveNMPC(pos, vel, quat);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
