from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, AcadosSimSolver
from casadi import vertcat, SX, dot, log, exp
from px4_model import *
import numpy as np
import scipy.linalg

np.set_printoptions(suppress=True)  # 禁用科学输入法输出

# 创建nmpc求解器
def create_nmpc_solver():
    # 创建一个acados最优控制问题的对象
    ocp = AcadosOcp()

    # 设置模型
    model = px4_model()
    ocp.model = model

    #读取状态变量、输入的维数
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # 设置代价函数类型
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # 状态变量权重矩阵
    Q = np.eye(nx)
    Q[0, 0] = 50    # x位置权重降低
    Q[1, 1] = 50    # y位置权重降低
    Q[2, 2] = 50    # z位置权重降低
    Q[3, 3] = 10    # vx速度权重降低
    Q[4, 4] = 10    # vy速度权重降低
    Q[5, 5] = 10    # vz速度权重降低
    Q[6, 6] = 20    # qw四元数权重增加
    Q[7, 7] = 20    # qx四元数权重增加
    Q[8, 8] = 20    # qy四元数权重增加
    Q[9, 9] = 20    # qz四元数权重增加
    Q[10, 10] = 5   # 推力权重
    Q[11, 11] = 8   # roll角速度权重
    Q[12, 12] = 8   # pitch角速度权重
    Q[13, 13] = 8   # yaw角速度权重
    Q[14, 14] = 5   # roll角速度导数权重
    Q[15, 15] = 5   # pitch角速度导数权重
    Q[16, 16] = 5   # yaw角速度导数权重

    # 输入权重矩阵（增加以抑制控制饱和）
    R = np.eye(nu)
    R[0, 0] = 1.0   # 推力导数权重增加
    R[1, 1] = 3.0   # roll角速度二阶导数权重增加
    R[2, 2] = 3.0   # pitch角速度二阶导数权重增加
    R[3, 3] = 3.0   # yaw角速度二阶导数权重增加

    # 终端状态变量权重矩阵
    W_e = np.eye(nx)
    W_e[0, 0] = 50    # x位置权重降低
    W_e[1, 1] = 50    # y位置权重降低
    W_e[2, 2] = 50    # z位置权重降低
    W_e[3, 3] = 10    # vx速度权重降低
    W_e[4, 4] = 10    # vy速度权重降低
    W_e[5, 5] = 10    # vz速度权重降低
    W_e[6, 6] = 20    # qw四元数权重增加
    W_e[7, 7] = 20    # qx四元数权重增加
    W_e[8, 8] = 20    # qy四元数权重增加
    W_e[9, 9] = 20    # qz四元数权重增加
    W_e[10, 10] = 5   # 推力权重
    W_e[11, 11] = 5   # roll角速度权重
    W_e[12, 12] = 5   # pitch角速度权重
    W_e[13, 13] = 5   # yaw角速度权重
    W_e[14, 14] = 1   # roll角速度导数权重
    W_e[15, 15] = 1   # pitch角速度导数权重
    W_e[16, 16] = 1   # yaw角速度导数权重

    # 设置参考向量（状态+输入）
    thrust = 1.535 * 9.81
    
    # 构建控制输入约束
    ocp.constraints.lbu = np.array([-1.5, -1.0, -1.0, -1.0])
    ocp.constraints.ubu = np.array([1.5, 1.0, 1.0, 1.0])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.array([0.0, 0.0, 2.0,
                                   0.0, 0.0, 0.0,
                                   1.0, 0.0, 0.0, 0.0,
                                   thrust, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 状态约束：推力、速度
    # max_thrust = 20.0
    # min_thrust = 10.0
    # max_vel = 2.0
    # ocp.constraints.idxbx = np.array([3, 4, 5, 10])
    # ocp.constraints.lbx = np.array([-max_vel, -max_vel, -max_vel, min_thrust])
    # ocp.constraints.ubx = np.array([max_vel, max_vel, max_vel, max_thrust])

    # ========== ESDF避障约束 ==========
    # 定义运行时参数: [d_ref, grad_x, grad_y, grad_z, x_ref, y_ref, z_ref]
    x = model.x
    u = model.u
    y_track = vertcat(x, u)

    p = SX.sym('p', 7)
    model.p = p

    # 避障参数
    beta = 10.0
    lambda_c = 1.0
    safety_distance = 0.5  # 安全距离 (m)

    # 避障代价项列表
    obs_terms = []

    # 提取状态中的位置
    pos = x[0:3]  # [px, py, pz]

    # 提取参数
    d_ref = p[0]        # ESDF查询点的距离
    grad = p[1:4]       # ESDF查询点的梯度 [gx, gy, gz]
    pos_ref = p[4:7]    # ESDF查询点的位置 [x_ref, y_ref, z_ref]

    # 一阶Taylor展开：d(pos) ≈ d_ref + grad·(pos - pos_ref)
    d_approx = d_ref + dot(grad, pos - pos_ref)

    # 用对数函数和指数函数构造避障代价项
    z = beta * (safety_distance**2 - d_approx**2)
    obs_cost = lambda_c * log(1 + exp(z))
    obs_terms.append(obs_cost)

    # 构建总的输出表达式（状态+输入+避障代价项）
    y_expr = vertcat(y_track, vertcat(*obs_terms))
    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = x

    # 重新计算输出维数
    n_extra_terms = len(obs_terms)
    ny = nx + nu + n_extra_terms
    ny_e = nx

    # 设置权重矩阵
    W_track = scipy.linalg.block_diag(Q, R)
    if n_extra_terms > 0:
        W_obs = 1000.0 * np.eye(n_extra_terms)
        ocp.cost.W = scipy.linalg.block_diag(W_track, W_obs)
    else:
        ocp.cost.W = W_track
    ocp.cost.W_e = 20 * W_e

    # 设置参考向量（状态+输入+避障代价项）
    x_ref = np.array([0.0, 1.0, 3.0,
                      0.0, 0.0, 0.0,
                      1.0, 0.0, 0.0, 0.0,
                      thrust, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0])
    yref = np.zeros(ny)
    yref[:nx] = x_ref
    ocp.cost.yref = yref
    ocp.cost.yref_e = x_ref

    # 设置参数初始值 [d_ref, gx, gy, gz, x_ref, y_ref, z_ref]
    # 默认值：假设远离障碍物
    ocp.parameter_values = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 设置求解器
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'   # PARTIAL_CONDENSING_HPIPM
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0

    # 设置预测时域：2s 内 40 个离散步长（dt=0.05s）
    N = 40
    T = 4
    ocp.dims.N = N
    ocp.solver_options.tf = T

    # 创建求解器
    acados_ocp = AcadosOcpSolver(ocp, json_file = 'quadrotor_ocp.json')
    acados_sim = AcadosSimSolver(ocp, json_file = 'quadrotor_ocp.json')

    return acados_ocp, acados_sim


def nmpc_sim():

    # 设置初始状态
    x0 = np.array([0, 0, 2, 
                   0, 0, 0, 
                   1, 0, 0, 0,
                   1.535 * 9.81, 0, 0, 0, 0, 0, 0])
    
    # 创建求解器和仿真器
    ocp_solver, integrator = create_nmpc_solver()

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    # 设置仿真循环次数
    Nsim = 100
    simX = np.zeros((Nsim+1, nx))
    simU = np.zeros((Nsim, nu))

    t_preparation = np.zeros(Nsim)
    t_feedback = np.zeros(Nsim)

    simX[0, :] = x0

    # 仿真循环
    for i in range(Nsim):

        # Phase 1: 准备阶段 - 基于先前状态进行线性化
        ocp_solver.options_set('rti_phase', 1)
        status = ocp_solver.solve()
        t_preparation[i] = ocp_solver.get_stats('time_tot')

        if status not in [0, 2, 5]:
            raise Exception(f'acados returned status {status}. Exiting.')
        
        # 更新初始状态
        ocp_solver.set(0, "lbx", simX[i, :])
        ocp_solver.set(0, "ubx", simX[i, :])

        # Phase 2: 反馈阶段 - 求解QP问题
        ocp_solver.options_set('rti_phase', 2)
        status = ocp_solver.solve()
        t_feedback[i] = ocp_solver.get_stats('time_tot')

        # 读取求解的第一个输入
        simU[i, :] = ocp_solver.get(0, "u")

        if status not in [0, 2, 5]:
            raise Exception(f'acados returned status {status}. Exiting.')
        
        # 仿真器求解下一时刻状态
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])

    # 将求解时间换算到ms打印
    t_feedback *= 1000
    t_preparation *= 1000
    print(f'Computation time in preparation phase in ms: \
            min {np.min(t_preparation):.3f} median {np.median(t_preparation):.3f} max {np.max(t_preparation):.3f}')
    print(f'Computation time in feedback phase in ms:    \
            min {np.min(t_feedback):.3f} median {np.median(t_feedback):.3f} max {np.max(t_feedback):.3f}')
    
    # 打印最终状态
    x_terminal = simX[-1, :]
    print(x_terminal)
    

if __name__ == '__main__':
    nmpc_sim()
