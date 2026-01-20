from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, AcadosSimSolver
from casadi import SX, vertcat, mtimes, cross
import numpy as np
import scipy.linalg


def quat_to_rotation_matrix(q):
    """
    四元数 q = [qw, qx, qy, qz] -> 旋转矩阵 R(q) (body->inertial).
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    row0 = vertcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy))
    row1 = vertcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx))
    row2 = vertcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2))
    return vertcat(row0.T, row1.T, row2.T)


def skew(v):
    """Skew-symmetric matrix [v]_x for v ∈ R^3."""
    return vertcat(
        vertcat(0, -v[2], v[1]).T,
        vertcat(v[2], 0, -v[0]).T,
        vertcat(-v[1], v[0], 0).T,
    )


def create_single_cable_model():
    """
    单条钢索 + 负载的局部模型:
        - 状态: 负载 (p, v, q, ω) + 本机钢索 (s, r, ṙ, r̈, t, ṫ)
        - 控制: 本机钢索的 γ, λ
        - 参数 p_ext: [F_ext(3), tau_ext(3), a_self(3)]
          由其他两架无人机的绳子张力提供外力/外力矩, a_self 为本机挂点.
    """
    model_name = "distributed_single_cable"

    g_val = 9.81
    m_load = 1.0

    # 简单长方体惯量
    Lx, Ly, Lz = 0.6, 0.4, 0.2
    Jxx = (m_load / 12.0) * (Ly**2 + Lz**2)
    Jyy = (m_load / 12.0) * (Lx**2 + Lz**2)
    Jzz = (m_load / 12.0) * (Lx**2 + Ly**2)
    J_load_np = np.diag([Jxx, Jyy, Jzz])

    J_load = SX(J_load_np)
    J_inv = SX(np.linalg.inv(J_load_np))

    # 钢索长度 (本机)
    cable_length = 1.5

    # -----------------------
    # 状态
    # -----------------------
    p = SX.sym("p", 3)
    v = SX.sym("v", 3)
    q = SX.sym("q", 4)
    omega = SX.sym("omega", 3)

    s = SX.sym("s", 3)
    r = SX.sym("r", 3)
    rdot = SX.sym("rdot", 3)
    rddot = SX.sym("rddot", 3)
    t = SX.sym("t")
    tdot = SX.sym("tdot")

    x = vertcat(p, v, q, omega, s, r, rdot, rddot, t, tdot)

    # -----------------------
    # 控制
    # -----------------------
    gamma = SX.sym("gamma", 3)
    lam = SX.sym("lam")
    u = vertcat(gamma, lam)

    # -----------------------
    # 参数: 外力/外力矩 + 挂点位置
    # -----------------------
    p_ext = SX.sym("p_ext", 9)
    F_ext = p_ext[0:3]
    tau_ext = p_ext[3:6]
    a_self = p_ext[6:9]

    # -----------------------
    # 动力学
    # -----------------------
    xdot = SX.sym("xdot", x.size()[0])

    g_vec = vertcat(0.0, 0.0, -g_val)

    R = quat_to_rotation_matrix(q)

    # 负载平动
    dp = v
    force_sum = t * s + F_ext
    dv = -(1.0 / m_load) * force_sum + g_vec

    # 负载转动
    Jomega = mtimes(J_load, omega)
    torque_self = t * cross(mtimes(R.T, s), a_self)
    domega = mtimes(J_inv, -cross(omega, Jomega) + torque_self + tau_ext)

    # 四元数运动学
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    omega_quat = vertcat(0.0, omega[0], omega[1], omega[2])
    # q_dot = 0.5 * q ⊗ [0; ω]
    q_dot0 = 0.5 * (-qx * omega[0] - qy * omega[1] - qz * omega[2])
    q_dot1 = 0.5 * (qw * omega[0] + qy * omega[2] - qz * omega[1])
    q_dot2 = 0.5 * (qw * omega[1] + qz * omega[0] - qx * omega[2])
    q_dot3 = 0.5 * (qw * omega[2] + qx * omega[1] - qy * omega[0])
    dq = vertcat(q_dot0, q_dot1, q_dot2, q_dot3)

    # 钢索动力学
    ds = cross(r, s)
    dr = rdot
    drdot = rddot
    drddot = gamma
    dt = tdot
    dtdot = lam

    f_expl = vertcat(dp, dv, dq, domega, ds, dr, drdot, drddot, dt, dtdot)
    f_impl = xdot - f_expl

    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p_ext
    model.z = vertcat()
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    params = {
        "g": g_val,
        "m_load": m_load,
        "J_load": J_load_np,
        "cable_length": cable_length,
    }

    return model, params


def create_single_cable_solver():
    """
    为单条钢索局部模型创建 acados OCP 和仿真器.
    """
    ocp = AcadosOcp()
    model, params = create_single_cable_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # NONLINEAR_LS cost: 这里只做简单 tracking，避障等可以后续扩展
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    Q = np.eye(nx)
    Q[0, 0] = 50.0
    Q[1, 1] = 50.0
    Q[2, 2] = 50.0

    Q[3, 3] = 20.0
    Q[4, 4] = 20.0
    Q[5, 5] = 20.0

    Q[6, 6] = 20.0
    Q[7, 7] = 20.0
    Q[8, 8] = 20.0
    Q[9, 9] = 20.0

    Q[10, 10] = 10.0
    Q[11, 11] = 10.0
    Q[12, 12] = 10.0

    R = np.eye(nu)
    R[0, 0] = 1.0
    R[1, 1] = 1.0
    R[2, 2] = 1.0
    R[3, 3] = 0.1

    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = 20.0 * Q

    Vx = np.zeros((ny, nx))
    for i in range(nx):
        Vx[i, i] = 1.0
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    for i in range(nu):
        Vu[nx + i, i] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.eye(ny_e)
    ocp.cost.Vx_e = Vx_e

    # 参考轨迹: 负载到 (5,0,0)，其余为 0
    yref = np.zeros(ny)
    yref[0:3] = np.array([5.0, 0.0, 0.0])
    yref[6] = 1.0  # qw
    ocp.cost.yref = yref
    ocp.cost.yref_e = yref[:nx]

    # 输入约束
    ocp.constraints.lbu = np.array([-5.0, -5.0, -5.0, -50.0])
    ocp.constraints.ubu = np.array([5.0, 5.0, 5.0, 50.0])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    # 初始状态 (由外部设置)
    x0 = np.zeros(nx)
    x0[6] = 1.0  # 四元数单位
    ocp.constraints.x0 = x0

    # 参数默认值: F_ext = 0, tau_ext = 0, a_self = 0
    ocp.parameter_values = np.zeros(9)

    # solver 选项
    # 使用 QPOASES 在小规模问题上通常更稳健
    ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0

    N = 20
    T = 2.0
    ocp.dims.N = N
    ocp.solver_options.tf = T

    json_name = "distributed_single_cable_ocp.json"
    ocp_solver = AcadosOcpSolver(ocp, json_file=json_name)
    sim_solver = AcadosSimSolver(ocp, json_file=json_name)

    return ocp_solver, sim_solver, params


def distributed_sim():
    """
    使用三套局部 NMPC 求解器进行近似分布式三机吊运仿真.
    这里只是演示结构:
        - 每个 UAV 有自己的 solver, 只建本机钢索.
        - 外力/外力矩通过其他 UAV 上一步预测的张力近似.
    """
    n_uav = 3
    ocp_solvers = []
    sim_solvers = []
    params_list = []

    for _ in range(n_uav):
        ocp_solver, sim_solver, params = create_single_cable_solver()
        ocp_solvers.append(ocp_solver)
        sim_solvers.append(sim_solver)
        params_list.append(params)

    nx = ocp_solvers[0].acados_ocp.dims.nx
    nu = ocp_solvers[0].acados_ocp.dims.nu
    N = ocp_solvers[0].acados_ocp.dims.N

    # 三个挂点 (世界坐标中绕负载中心均匀分布)
    radius = 0.5 * max(0.6, 0.4)
    a_list = []
    for i in range(n_uav):
        angle = 2.0 * np.pi * i / n_uav
        a_list.append(np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0]))

    # 初始状态: 负载在原点, 三根绳竖直向下
    def initial_state_for_uav():
        x0 = np.zeros(nx)
        x0[6] = 1.0
        # s = [0,0,-1]
        x0[13:16] = np.array([0.0, 0.0, -1.0])
        return x0

    X = [np.zeros((N + 1, nx)) for _ in range(n_uav)]
    U = [np.zeros((N, nu)) for _ in range(n_uav)]
    x_curr = [initial_state_for_uav() for _ in range(n_uav)]

    for i in range(n_uav):
        X[i][0, :] = x_curr[i]

    Nsim = 60

    for it in range(Nsim):
        # 1) 从上一轮预测构造外力/力矩参数 (Jacobi 型分布式耦合)
        for i in range(n_uav):
            for k in range(N):
                # 默认外力/力矩为 0
                F_ext = np.zeros(3)
                tau_ext = np.zeros(3)

                if it > 0:
                    # 使用上一轮预测轨迹估计其他两架 UAV 对负载的拉力
                    # 近似使用 UAV0 的负载姿态作为全局负载姿态
                    q_k = X[0][k, 6:10]
                    qw, qx, qy, qz = q_k
                    R_k = np.array(
                        [
                            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
                            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
                        ]
                    )

                    for j in range(n_uav):
                        if j == i:
                            continue
                        # 从上一轮预测中取得绳子方向和张力
                        s_j = X[j][k, 13:16]
                        t_j = X[j][k, 25]
                        F_ext += t_j * s_j
                        a_j = a_list[j]
                        s_body = R_k.T @ s_j
                        tau_ext += t_j * np.cross(s_body, a_j)

                p_ext = np.zeros(9)
                p_ext[0:3] = F_ext
                p_ext[3:6] = tau_ext
                p_ext[6:9] = a_list[i]
                ocp_solvers[i].set(k, "p", p_ext)

        # 3) 每个 UAV 求解本地 OCP
        for i in range(n_uav):
            ocp = ocp_solvers[i]
            sim = sim_solvers[i]

            # RTI phase 1
            ocp.options_set("rti_phase", 1)
            status = ocp.solve()
            if status not in [0, 2, 5]:
                print(f"[WARN] UAV {i}: preparation phase status {status}, continue.")

            # 更新初始状态约束
            ocp.set(0, "lbx", x_curr[i])
            ocp.set(0, "ubx", x_curr[i])

            # RTI phase 2
            ocp.options_set("rti_phase", 2)
            status = ocp.solve()
            if status not in [0, 2, 5]:
                print(f"[WARN] UAV {i}: feedback phase status {status}, fallback to last control.")
                if it == 0:
                    u0 = np.zeros(nu)
                else:
                    u0 = U[i][(it - 1) % N, :]
            else:
                # 读取控制
                u0 = ocp.get(0, "u")

            # 前向仿真
            x_next = sim.simulate(x=x_curr[i], u=u0)

            # 保存当前步的控制和状态
            U[i][it % N, :] = u0
            X[i][0, :] = x_curr[i]
            X[i][1, :] = x_next

            # 更新预测轨迹 (仅当求解成功时才更新; 否则保留上一轮)
            if status in [0, 2, 5]:
                for k in range(1, N + 1):
                    X[i][k, :] = ocp.get(k, "x")

            x_curr[i] = x_next

    print("Distributed simulation finished.")


if __name__ == "__main__":
    distributed_sim()
