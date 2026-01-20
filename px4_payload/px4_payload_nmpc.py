# 将输入修改为推力的导数和角速度的二阶导数，使无人机运动更加平滑

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, AcadosSimSolver
from casadi import vertcat, SX, dot
from px4_payload_model import *
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib


np.set_printoptions(suppress=True)  # 禁用科学输入法输出

def create_nmpc_solver():
    
    # 创建一个acados最优控制问题的对象
    ocp = AcadosOcp()

    # 设置模型
    model = px4_payload_model()
    ocp.model = model

    # 读取状态变量、输入的维数
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # 设置代价函数类型
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # 状态变量权重矩阵
    Q = np.eye(nx)
    Q[0, 0] = 150      # 负载x位置权重
    Q[1, 1] = 150      # 负载y位置权重
    Q[2, 2] = 50      # 负载z位置权重
    Q[3, 3] = 50      # 负载vx速度权重
    Q[4, 4] = 50      # 负载vy速度权重
    Q[5, 5] = 50      # 负载vz速度权重
    Q[6, 6] = 20      # 绳子x方向权重
    Q[7, 7] = 20      # 绳子y方向权重
    Q[8, 8] = 20      # 绳子z方向权重
    Q[9, 9] = 20      # 绳子x角速度权重
    Q[10, 10] = 20    # 绳子y角速度权重
    Q[11, 11] = 20    # 绳子z角速度权重
    Q[12, 12] = 20    # 四元数qw权重
    Q[13, 13] = 20    # 四元数qx权重
    Q[14, 14] = 20    # 四元数qy权重
    Q[15, 15] = 20    # 四元数qz权重
    Q[16, 16] = 5    # 推力 T 状态权重
    Q[17, 17] = 5    # 机体系角速度 Ωx 状态权重
    Q[18, 18] = 5    # 机体系角速度 Ωy 状态权重
    Q[19, 19] = 5    # 机体系角速度 Ωz 状态权重
    Q[20, 20] = 1    # 角速度导数 Ωdot_x 状态权重
    Q[21, 21] = 1    # 角速度导数 Ωdot_y 状态权重
    Q[22, 22] = 1    # 角速度导数 Ωdot_z 状态权重

    # Q[0, 0] = 50      # 负载x位置权重
    # Q[1, 1] = 50      # 负载y位置权重
    # Q[2, 2] = 50      # 负载z位置权重
    # Q[3, 3] = 10      # 负载vx速度权重
    # Q[4, 4] = 10      # 负载vy速度权重
    # Q[5, 5] = 10      # 负载vz速度权重
    # Q[6, 6] = 0      # 绳子x方向权重
    # Q[7, 7] = 0      # 绳子y方向权重
    # Q[8, 8] = 0      # 绳子z方向权重
    # Q[9, 9] = 0      # 绳子x角速度权重
    # Q[10, 10] = 0    # 绳子y角速度权重
    # Q[11, 11] = 0    # 绳子z角速度权重
    # Q[12, 12] = 20    # 四元数qw权重
    # Q[13, 13] = 20    # 四元数qx权重
    # Q[14, 14] = 20    # 四元数qy权重
    # Q[15, 15] = 20    # 四元数qz权重

    # 输入权重矩阵（增加以抑制控制变化率）
    R = np.eye(nu)
    R[0, 0] = 1.0   # 推力导数权重
    R[1, 1] = 1.0   # roll角速度二阶导权重
    R[2, 2] = 1.0   # pitch角速度二阶导权重
    R[3, 3] = 1.0   # yaw角速度二阶导权重

    # 设置过程和末端的权重矩阵
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    # ocp.cost.W_e = 50 * np.eye(ny_e) * 20

    W_e = np.eye(ny_e)
    W_e[0, 0] = 150      # 负载x位置权重
    W_e[1, 1] = 150      # 负载y位置权重
    W_e[2, 2] = 50      # 负载z位置权重
    W_e[3, 3] = 50      # 负载vx速度权重
    W_e[4, 4] = 50      # 负载vy速度权重
    W_e[5, 5] = 50      # 负载vz速度权重
    W_e[6, 6] = 20      # 绳子x方向权重
    W_e[7, 7] = 20      # 绳子y方向权重
    W_e[8, 8] = 20      # 绳子z方向权重
    W_e[9, 9] = 20      # 绳子x角速度权重
    W_e[10, 10] = 20    # 绳子y角速度权重
    W_e[11, 11] = 20    # 绳子z角速度权重
    W_e[12, 12] = 20    # 四元数qw权重
    W_e[13, 13] = 20    # 四元数qx权重
    W_e[14, 14] = 20    # 四元数qy权重
    W_e[15, 15] = 20    # 四元数qz权重
    W_e[16, 16] = 5    # 推力 T 状态权重
    W_e[17, 17] = 5    # 机体系角速度 Ωx 状态权重
    W_e[18, 18] = 5    # 机体系角速度 Ωy 状态权重
    W_e[19, 19] = 5    # 机体系角速度 Ωz 状态权重
    W_e[20, 20] = 1    # 角速度导数 Ωdot_x 状态权重
    W_e[21, 21] = 1    # 角速度导数 Ωdot_y 状态权重
    W_e[22, 22] = 1    # 角速度导数 Ωdot_z 状态权重

    ocp.cost.W_e = 20 * W_e

    # 投影矩阵，使代价在 y = [x; u] 上为 J = (y - yref)^T W (y - yref)
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:, :] = np.eye(ny_e)
    ocp.cost.Vx_e = Vx_e

    # 设置参考向量（状态+输入, ENU坐标系）
    hover_thrust = (1.535 + 0.35) * 9.81
    yref = np.zeros(ny)
    yref_e = np.zeros(ny_e)

    # ENU: x=East, y=North, z=Up
    # 参考位置: [1m East, 1m North, 3m Up] - 悬停在空中3米
    # 绳子方向: [0, 0, -1] - 垂直向下（在ENU中向下为负Z）
    yref[0] = 2.0   # 负载 x
    yref[1] = 0.0   # 负载 y
    yref[2] = 2.0   # 负载 z
    yref[3] = 0.0   # 负载 vx
    yref[4] = 0.0   # 负载 vy
    yref[5] = 0.0   # 负载 vz
    yref[6] = 0.0   # 绳子方向 qx
    yref[7] = 0.0   # 绳子方向 qy
    yref[8] = -1.0  # 绳子方向 qz
    yref[9] = 0.0   # 绳子角速度 wx
    yref[10] = 0.0  # 绳子角速度 wy
    yref[11] = 0.0  # 绳子角速度 wz
    yref[12] = 1.0  # 四元数 q0
    yref[13] = 0.0  # 四元数 q1
    yref[14] = 0.0  # 四元数 q2
    yref[15] = 0.0  # 四元数 q3
    # 推力状态参考值（悬停）
    yref[16] = hover_thrust

    # 终端参考仅对状态
    yref_e[:] = yref[:ny_e]

    ocp.cost.yref = yref
    ocp.cost.yref_e = yref_e
    
    # 构建约束：现在约束的是推力导数和角速度二阶导
    ocp.constraints.lbu = np.array([-50.0, -50.0, -50.0, -50.0])
    ocp.constraints.ubu = np.array([50.0, 50.0, 50.0, 50.0])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    # 初始状态 (ENU坐标系)
    # 位置: [0, 0, 1m Up], 绳子方向: [0, 0, -1] (向下)，推力为悬停推力，角速度及其导数为0
    x0_init = np.array([
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, -1.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        hover_thrust,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ])
    ocp.constraints.x0 = x0_init
    
    # ========== ESDF避障软约束（负载 + 无人机） ==========
    # 运行时参数 (14维):
    # [dL_ref, gLx, gLy, gLz, xL_ref, yL_ref, zL_ref,
    #  dQ_ref, gQx, gQy, gQz, xQ_ref, yQ_ref, zQ_ref]
    p = SX.sym('p', 14)
    model.p = p

    x_sym = model.x
    payload_pos = x_sym[0:3]  # [xp, yp, zp]
    rope_dir = x_sym[6:9]      # 绳子方向 q = [qx, qy, qz]

    # 由负载位置和绳子方向恢复无人机位置
    rope_length = 1.0          # 与动力学模型和ROS参数保持一致
    quad_pos = payload_pos - rope_length * rope_dir

    # 负载参数段
    dL_ref = p[0]
    gradL = p[1:4]
    posL_ref = p[4:7]

    # 无人机参数段
    dQ_ref = p[7]
    gradQ = p[8:11]
    posQ_ref = p[11:14]

    # 一阶Taylor展开：
    # d_L(payload_pos) ≈ dL_ref + gradL·(payload_pos - posL_ref)
    # d_Q(quad_pos)    ≈ dQ_ref + gradQ·(quad_pos    - posQ_ref)
    dL_approx = dL_ref + dot(gradL, payload_pos - posL_ref)
    dQ_approx = dQ_ref + dot(gradQ, quad_pos    - posQ_ref)

    safety_distance_L = 0.2   # 负载安全距离 (m)
    safety_distance_Q = 0.4   # 无人机安全距离 (m)

    h_L = dL_approx - safety_distance_L
    h_Q = dQ_approx - safety_distance_Q

    # 2 维非线性约束向量
    h_expr = vertcat(h_L, h_Q)

    # 设置路径约束
    model.con_h_expr = h_expr
    ocp.constraints.lh = np.array([0.0, 0.0])
    ocp.constraints.uh = np.array([100.0, 100.0])

    # 使用软约束：对 h_L >= 0 和 h_Q >= 0 引入 slack，并在代价函数中强惩罚
    ocp.constraints.idxsh = np.array([0, 1], dtype=int)
    ocp.constraints.lsh = np.array([0.0, 0.0])     # slack >= 0
    ocp.constraints.ush = np.array([1e3, 1e3])     # 上界给一个较大值

    soft_weight = 2e3                              # 软约束权重
    soft_weight1 = 3e3
    ocp.cost.Zl = 2 * np.array([soft_weight, soft_weight1])
    ocp.cost.Zu = 2 * np.array([soft_weight, soft_weight1])
    ocp.cost.zl = 2 * np.array([soft_weight, soft_weight1])
    ocp.cost.zu = 2 * np.array([soft_weight, soft_weight1])

    # 参数初始值: 负载与无人机都远离障碍物
    ocp.parameter_values = np.array([
        10.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,     # payload
        10.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0      # quadrotor
    ])
    
    # 设置求解器
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'   # PARTIAL_CONDENSING_HPIPM
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0

    # 设置预测时域
    N = 20
    T = 2
    ocp.dims.N = N
    ocp.solver_options.tf = T

    # 创建求解器
    acados_ocp = AcadosOcpSolver(ocp, json_file = 'quadrotor_ocp.json')
    acados_sim = AcadosSimSolver(ocp, json_file = 'quadrotor_ocp.json')

    return acados_ocp, acados_sim

def nmpc_sim():

    # 创建求解器和仿真器
    ocp_solver, integrator = create_nmpc_solver()

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    # 设置初始状态 (ENU坐标系)，与 create_nmpc_solver 中一致
    hover_thrust = (1.585 + 0.35) * 9.81
    x0 = np.array([
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, -1.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        hover_thrust,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ])

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
        
        # 更新初始状态约束
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
    
    # 状态变量名称
    state_names = [
        'xp   (负载位置 x)',
        'yp   (负载位置 y)',
        'zp   (负载位置 z)',
        'vxp  (负载速度 x)',
        'vyp  (负载速度 y)',
        'vzp  (负载速度 z)',
        'qx   (绳子方向 x)',
        'qy   (绳子方向 y)',
        'qz   (绳子方向 z)',
        'wx   (绳子角速度 x)',
        'wy   (绳子角速度 y)',
        'wz   (绳子角速度 z)',
        'q0   (四元数 w)',
        'q1   (四元数 x)',
        'q2   (四元数 y)',
        'q3   (四元数 z)',
        'T    (推力)',
        'Ωx   (机体系角速度 x)',
        'Ωy   (机体系角速度 y)',
        'Ωz   (机体系角速度 z)',
        'Ωdot_x (角速度导数 x)',
        'Ωdot_y (角速度导数 y)',
        'Ωdot_z (角速度导数 z)',
    ]

    print("\n" + "="*60)
    print("最终状态:")
    print("="*60)
    for i, (name, value) in enumerate(zip(state_names, x_terminal)):
        print(f"{i:2d}. {name:25s} = {value:10.6f}")
    print("="*60)

    return simX, simU


def quaternion_to_rotation_matrix(q):
    """
    将四元数转换为旋转矩阵

    参数:
        q: 四元数 [qw, qx, qy, qz]

    返回:
        R: 3x3 旋转矩阵
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    # 归一化四元数
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

    # 计算旋转矩阵
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])

    return R


def visualize_drone_payload(simX, simU, rope_length=1.0, interval=50, save_animation=False):
    """
    可视化无人机、绳子和负载的3D动画

    参数:
        simX: 状态轨迹数组，形状为 (Nsim+1, nx)
              状态变量:
              [xp, yp, zp, vxp, vyp, vzp, qx, qy, qz, wx, wy, wz,
               q0, q1, q2, q3, T, omega_x, omega_y, omega_z,
               omega_dot_x, omega_dot_y, omega_dot_z]
        simU: 控制输入轨迹数组，形状为 (Nsim, nu)
              控制输入: [T_dot, omega_ddot_x, omega_ddot_y, omega_ddot_z]
        rope_length: 绳子长度 (米)
        interval: 动画帧间隔 (毫秒)
        save_animation: 是否保存动画为gif文件
    """

    # 创建3D图形
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 提取负载位置
    payload_x = simX[:, 0]  # xp
    payload_y = simX[:, 1]  # yp
    payload_z = simX[:, 2]  # zp

    # 提取绳子方向向量 (单位向量，从无人机指向负载)
    rope_dir_x = simX[:, 6]  # qx
    rope_dir_y = simX[:, 7]  # qy
    rope_dir_z = simX[:, 8]  # qz

    # 计算无人机位置 = 负载位置 - 绳子方向 * 绳长
    # 因为绳子方向是从无人机指向负载，所以要减去
    drone_x = payload_x - rope_dir_x * rope_length
    drone_y = payload_y - rope_dir_y * rope_length
    drone_z = payload_z - rope_dir_z * rope_length

    # 设置图形边界
    all_x = np.concatenate([payload_x, drone_x])
    all_y = np.concatenate([payload_y, drone_y])
    all_z = np.concatenate([payload_z, drone_z])

    max_range = np.array([all_x.max()-all_x.min(),
                          all_y.max()-all_y.min(),
                          all_z.max()-all_z.min()]).max() / 2.0

    mid_x = (all_x.max()+all_x.min()) * 0.5
    mid_y = (all_y.max()+all_y.min()) * 0.5
    mid_z = (all_z.max()+all_z.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # 设置标签 (ENU坐标系)
    ax.set_xlabel('East (m)', fontsize=10)
    ax.set_ylabel('North (m)', fontsize=10)
    ax.set_zlabel('Up (m)', fontsize=10)
    ax.set_title('Drone-Rope-Payload System Animation', fontsize=14, fontweight='bold')

    # 绘制轨迹
    ax.plot(payload_x, payload_y, payload_z, 'b--', alpha=0.3, linewidth=1, label='Payload Trajectory')
    ax.plot(drone_x, drone_y, drone_z, 'r--', alpha=0.3, linewidth=1, label='Drone Trajectory')

    # 绘制障碍物（使用与约束相同的参数）
    obs_x = 4.0
    obs_y = 0.0
    obs_z = 1.0
    obs_radius = 1.0
    safety_margin = 0.1
    min_distance = obs_radius + safety_margin

    # 辅助函数：绘制球体
    def draw_sphere(ax, center_x, center_y, center_z, radius, color, alpha):
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center_x + radius * np.outer(np.cos(u), np.sin(v))
        y = center_y + radius * np.outer(np.sin(u), np.sin(v))
        z = center_z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, color=color, alpha=alpha)

    # 绘制障碍物和安全区域
    draw_sphere(ax, obs_x, obs_y, obs_z, obs_radius, 'red', 0.3)
    # draw_sphere(ax, obs_x, obs_y, obs_z, min_distance, 'orange', 0.15)

    # 为图例创建虚拟的代理对象
    from matplotlib.patches import Patch
    obstacle_patch = Patch(color='red', alpha=0.3, label=f'Obstacle (r={obs_radius}m)')
    safety_patch = Patch(color='orange', alpha=0.15, label=f'Safety Zone (r={min_distance}m)')

    # 初始化动画元素
    drone_point, = ax.plot([], [], [], 'ko', markersize=10, label='Drone')
    payload_point, = ax.plot([], [], [], 'bs', markersize=10, label='Payload')
    rope_line, = ax.plot([], [], [], 'k-', linewidth=2, label='Rope')

    # 无人机的旋翼表示（用四个小点表示）
    rotor_size = 0.15  # 旋翼臂长度
    rotor_points, = ax.plot([], [], [], 'k^', markersize=6)

    # 无人机姿态坐标系（机体坐标系）
    axis_length = 0.3  # 坐标轴长度
    x_axis, = ax.plot([], [], [], 'r-', linewidth=2.5, label='X-axis (Forward)')
    y_axis, = ax.plot([], [], [], 'g-', linewidth=2.5, label='Y-axis (LEFT)')
    z_axis, = ax.plot([], [], [], 'b-', linewidth=2.5, label='Z-axis (UP)')

    # 时间文本
    time_text = ax.text2D(0.01, 0.98, '', transform=ax.transAxes, fontsize=11,
                          fontweight='bold', verticalalignment='top')

    # 状态信息文本
    state_text = ax.text2D(0.01, 0.92, '', transform=ax.transAxes, fontsize=8,
                          verticalalignment='top', family='monospace',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    # 控制输入文本
    control_text = ax.text2D(0.01, 0.65, '', transform=ax.transAxes, fontsize=8,
                            verticalalignment='top', family='monospace',
                            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

    # 创建图例，包括障碍物的代理对象
    handles, labels = ax.get_legend_handles_labels()
    handles.extend([obstacle_patch, safety_patch])
    ax.legend(handles=handles, loc='upper right', fontsize=8)

    # 动画更新函数
    def update(frame):
        # 更新负载位置
        payload_point.set_data([payload_x[frame]], [payload_y[frame]])
        payload_point.set_3d_properties([payload_z[frame]])

        # 更新无人机位置
        drone_point.set_data([drone_x[frame]], [drone_y[frame]])
        drone_point.set_3d_properties([drone_z[frame]])

        # 更新绳子
        rope_line.set_data([payload_x[frame], drone_x[frame]],
                          [payload_y[frame], drone_y[frame]])
        rope_line.set_3d_properties([payload_z[frame], drone_z[frame]])

        # 获取当前帧的四元数并计算旋转矩阵
        q = simX[frame, 12:16]  # [q0, q1, q2, q3] = [qw, qx, qy, qz]
        R = quaternion_to_rotation_matrix(q)

        # 无人机机体坐标系的三个轴（在ENU坐标系中）
        # X轴（前）：红色
        # Y轴（右）：绿色
        # Z轴（下）：蓝色
        body_x = R[:, 0] * axis_length  # 机体X轴（前方）
        body_y = R[:, 1] * axis_length  # 机体Y轴（右方）
        body_z = R[:, 2] * axis_length  # 机体Z轴（下方）

        # 绘制机体坐标系轴
        drone_pos = np.array([drone_x[frame], drone_y[frame], drone_z[frame]])

        x_axis.set_data([drone_pos[0], drone_pos[0] + body_x[0]],
                       [drone_pos[1], drone_pos[1] + body_x[1]])
        x_axis.set_3d_properties([drone_pos[2], drone_pos[2] + body_x[2]])

        y_axis.set_data([drone_pos[0], drone_pos[0] + body_y[0]],
                       [drone_pos[1], drone_pos[1] + body_y[1]])
        y_axis.set_3d_properties([drone_pos[2], drone_pos[2] + body_y[2]])

        z_axis.set_data([drone_pos[0], drone_pos[0] + body_z[0]],
                       [drone_pos[1], drone_pos[1] + body_z[1]])
        z_axis.set_3d_properties([drone_pos[2], drone_pos[2] + body_z[2]])

        # 更新旋翼位置（四个旋翼，根据机体坐标系旋转）
        rotor_x = []
        rotor_y = []
        rotor_z = []
        for angle in [45, 135, 225, 315]:  # 四个旋翼的角度
            rad = np.deg2rad(angle)
            # 在机体坐标系XY平面上的旋翼位置
            rotor_body = np.array([rotor_size * np.cos(rad),
                                   rotor_size * np.sin(rad),
                                   0.0])
            # 转换到世界坐标系
            rotor_world = drone_pos + R @ rotor_body
            rotor_x.append(rotor_world[0])
            rotor_y.append(rotor_world[1])
            rotor_z.append(rotor_world[2])

        rotor_points.set_data(rotor_x, rotor_y)
        rotor_points.set_3d_properties(rotor_z)

        # 更新时间
        dt = 0.05  # 假设时间步长为50ms
        time_text.set_text(f'Time: {frame * dt:.2f} s')

        # 计算欧拉角用于显示（Roll, Pitch, Yaw）
        # 从旋转矩阵计算欧拉角
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])

        # 更新状态信息
        state_str = '--- STATE ---\n'
        state_str += f'Payload: ({payload_x[frame]:5.2f}, {payload_y[frame]:5.2f}, {payload_z[frame]:5.2f}) m\n'
        state_str += f'Drone:   ({drone_x[frame]:5.2f}, {drone_y[frame]:5.2f}, {drone_z[frame]:5.2f}) m\n'
        state_str += f'Rope:    ({rope_dir_x[frame]:5.2f}, {rope_dir_y[frame]:5.2f}, {rope_dir_z[frame]:5.2f})\n'
        state_str += f'Roll:  {np.rad2deg(roll):6.1f} deg\n'
        state_str += f'Pitch: {np.rad2deg(pitch):6.1f} deg\n'
        state_str += f'Yaw:   {np.rad2deg(yaw):6.1f} deg'
        state_text.set_text(state_str)

        # 更新控制输入信息（显示状态中的 T, Ω 以及输入中的导数）
        T_state = simX[frame, 16]
        omega_x_state = simX[frame, 17]
        omega_y_state = simX[frame, 18]
        omega_z_state = simX[frame, 19]

        if frame < len(simU):
            u_T_dot = simU[frame, 0]
            u_omega_ddot_x = simU[frame, 1]
            u_omega_ddot_y = simU[frame, 2]
            u_omega_ddot_z = simU[frame, 3]
        else:
            u_T_dot = simU[-1, 0]
            u_omega_ddot_x = simU[-1, 1]
            u_omega_ddot_y = simU[-1, 2]
            u_omega_ddot_z = simU[-1, 3]

        control_str = '--- CONTROL (STATE & INPUT) ---\n'
        control_str += f'T:        {T_state:7.2f} N\n'
        control_str += (
            f'Omega:    [{omega_x_state:6.3f}, '
            f'{omega_y_state:6.3f}, {omega_z_state:6.3f}] rad/s\n'
        )
        control_str += f'T_dot:    {u_T_dot:7.2f} N/s\n'
        control_str += (
            'Omega_ddot: '
            f'[{u_omega_ddot_x:6.2f}, {u_omega_ddot_y:6.2f}, '
            f'{u_omega_ddot_z:6.2f}] rad/s^2'
        )
        control_text.set_text(control_str)

        return drone_point, payload_point, rope_line, rotor_points, x_axis, y_axis, z_axis, time_text, state_text, control_text

    # 创建动画
    anim = FuncAnimation(fig, update, frames=len(simX), interval=interval, blit=True, repeat=True)

    # 保存动画（可选）
    if save_animation:
        print("Saving animation to drone_payload_animation.gif ...")
        anim.save('drone_payload_animation.gif', writer='pillow', fps=20)
        print("Animation saved!")

    plt.tight_layout()
    plt.show()

    return anim


if __name__ == '__main__':
    # 运行仿真
    simX, simU = nmpc_sim()

    # 可视化结果
    print("\nGenerating animation...")
    visualize_drone_payload(simX, simU, rope_length=1.0, interval=50, save_animation=False)
