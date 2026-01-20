from acados_template import AcadosModel
from casadi import SX, vertcat, mtimes, cross, dot
import numpy as np


def quat_to_rotation_matrix(q):
    """
    将四元数 q = [qw, qx, qy, qz] 转换为旋转矩阵 R(q) ∈ SO(3).
    这里采用与其他模块一致的 Hamilton 约定。
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    row0 = vertcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy))
    row1 = vertcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx))
    row2 = vertcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2))

    return vertcat(row0.T, row1.T, row2.T)


def quat_omega_matrix(q):
    """
    Hamilton 约定下的 Λ(q)，满足:
        q_dot = 0.5 * Λ(q) * [0; ω]
    其中 q = [qw, qx, qy, qz], ω 为在负载坐标系中的角速度.
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return vertcat(
        vertcat(qw, -qx, -qy, -qz).T,
        vertcat(qx, qw, -qz, qy).T,
        vertcat(qy, qz, qw, -qx).T,
        vertcat(qz, -qy, qx, qw).T,
    )


def create_kinodynamic_model(n_cables: int = 3):
    """
    构建刚体负载 + 多条钢索的动力学模型.

    状态:
        x = [p, v, q, ω,
             s_1, r_1, ṙ_1, r̈_1, t_1, ṫ_1,
             ...
             s_n, r_n, ṙ_n, r̈_n, t_n, ṫ_n]
        维度: nx = 13 + 14 * n_cables

    控制:
        u = [γ_1, λ_1, ..., γ_n, λ_n], 其中 γ_i ∈ R^3, λ_i ∈ R
        维度: nu = 4 * n_cables

    返回:
        model: AcadosModel
        params: 包含质量、惯量、钢索长度、安装点等物理参数的字典
    """

    model_name = f"kinodynamic_load_{n_cables}cables"

    # 物理参数（刚性长方体）
    g_val = 9.81
    m_load = 1.0  # 负载质量 [kg]

    # 长方体尺寸（负载坐标系下，以质心为原点）
    # x 轴指向“前方”，y 轴为“横向”，z 轴为“竖直”
    Lx = 0.6  # 前后长度
    Ly = 0.4  # 左右宽度
    Lz = 0.2  # 高度

    # 长方体惯量矩阵（绕质心，标准公式）
    Jxx = (m_load / 12.0) * (Ly**2 + Lz**2)
    Jyy = (m_load / 12.0) * (Lx**2 + Lz**2)
    Jzz = (m_load / 12.0) * (Lx**2 + Ly**2)
    J_load_np = np.diag([Jxx, Jyy, Jzz])

    # 将惯量矩阵转换为 CasADi 类型
    J_load = SX(J_load_np)
    J_inv = SX(np.linalg.inv(J_load_np))

    # 钢索长度和安装点（负载坐标系下）
    # 挂载点围绕质心在 x-y 平面上等角度均匀分布在一圈圆周上，
    # 这样当三根绳子方向相同且张力相等时，合力矩为零（静力平衡更好）。
    cable_length = 1.5
    # 半径取长方体在 x-y 平面外接矩形的一半
    radius = 0.5 * max(Lx, Ly)
    a_list_np = []
    for i in range(n_cables):
        angle = 2.0 * np.pi * i / max(n_cables, 1)
        a_list_np.append(
            np.array(
                [
                    radius * np.cos(angle),
                    radius * np.sin(angle),
                    0.0,
                ]
            )
        )

    # -----------------------
    # 一些工具函数
    # -----------------------
    def skew(v):
        """Skew-symmetric matrix [v]_x for v ∈ R^3."""
        return vertcat(
            vertcat(0, -v[2], v[1]).T,
            vertcat(v[2], 0, -v[0]).T,
            vertcat(-v[1], v[0], 0).T,
        )

    # -----------------------
    # 状态变量
    # -----------------------
    p = SX.sym("p", 3)  # 负载位置 (inertial)
    v = SX.sym("v", 3)  # 负载线速度
    q = SX.sym("q", 4)  # 负载姿态四元数
    omega = SX.sym("omega", 3)  # 负载角速度 (body frame)

    s_list = []
    r_list = []
    rdot_list = []
    rddot_list = []
    t_list = []
    tdot_list = []

    x_list = [p, v, q, omega]

    for i in range(n_cables):
        s_i = SX.sym(f"s_{i+1}", 3)
        r_i = SX.sym(f"r_{i+1}", 3)
        rdot_i = SX.sym(f"rdot_{i+1}", 3)
        rddot_i = SX.sym(f"rddot_{i+1}", 3)
        t_i = SX.sym(f"t_{i+1}")
        tdot_i = SX.sym(f"tdot_{i+1}")

        s_list.append(s_i)
        r_list.append(r_i)
        rdot_list.append(rdot_i)
        rddot_list.append(rddot_i)
        t_list.append(t_i)
        tdot_list.append(tdot_i)

        x_list += [s_i, r_i, rdot_i, rddot_i, t_i, tdot_i]

    x = vertcat(*x_list)

    # -----------------------
    # 控制输入
    # -----------------------
    gamma_list = []  # 每条钢索的角 snap γ_i ∈ R^3
    lambda_list = []  # 张力的二阶导 λ_i ∈ R
    u_list = []

    for i in range(n_cables):
        gamma_i = SX.sym(f"gamma_{i+1}", 3)
        lambda_i = SX.sym(f"lambda_{i+1}")

        gamma_list.append(gamma_i)
        lambda_list.append(lambda_i)

        u_list += [gamma_i, lambda_i]

    u = vertcat(*u_list)

    # -----------------------
    # 状态导数变量（隐式模型用）
    # -----------------------
    xdot = SX.sym("xdot", x.size()[0])

    # -----------------------
    # 连续时间动力学 f(x, u)
    # -----------------------

    # 重力向量
    g_vec = vertcat(0.0, 0.0, -g_val)

    # 负载平动
    dp = v
    force_sum = vertcat(0.0, 0.0, 0.0)
    torque_sum = vertcat(0.0, 0.0, 0.0)

    R = quat_to_rotation_matrix(q)

    for i in range(n_cables):
        s_i = s_list[i]
        t_i = t_list[i]

        # 平动中的张力合力项
        force_sum += t_i * s_i

        # 转动中的力矩项: t_i * (R(q)^T s_i × a_i)
        a_i_np = a_list_np[i]
        a_i = vertcat(a_i_np[0], a_i_np[1], a_i_np[2])
        s_body = mtimes(R.T, s_i)
        torque_sum += t_i * cross(s_body, a_i)

    dv = -(1.0 / m_load) * force_sum + g_vec

    # 负载旋转动力学
    Jomega = mtimes(J_load, omega)
    domega = mtimes(J_inv, -cross(omega, Jomega) + torque_sum)

    # 四元数运动学
    Lambda_q = quat_omega_matrix(q)
    omega_quat = vertcat(0.0, omega[0], omega[1], omega[2])
    dq = 0.5 * mtimes(Lambda_q, omega_quat)

    # 钢索动力学: s_i, r_i, ṙ_i, r̈_i, t_i, ṫ_i
    xdot_list = [dp, dv, dq, domega]

    for i in range(n_cables):
        s_i = s_list[i]
        r_i = r_list[i]
        rdot_i = rdot_list[i]
        rddot_i = rddot_list[i]
        t_i = t_list[i]
        tdot_i = tdot_list[i]
        gamma_i = gamma_list[i]
        lambda_i = lambda_list[i]

        ds_i = cross(r_i, s_i)
        dr_i = rdot_i
        drdot_i = rddot_i
        drddot_i = gamma_i
        dt_i = tdot_i
        dtdot_i = lambda_i

        xdot_list += [ds_i, dr_i, drdot_i, drddot_i, dt_i, dtdot_i]

    f_expl = vertcat(*xdot_list)
    f_impl = xdot - f_expl

    # -----------------------
    # 无人机推力约束相关表达式（Section 7.2）
    # -----------------------
    # 每架无人机质量
    m_quad = 2.0

    # 负载加速度 a_L = dv
    a_load = dv

    # 预计算刚体旋转二阶项
    omega_skew = skew(omega)
    omega_skew_sq = mtimes(omega_skew, omega_skew)
    omega_dot_skew = skew(domega)

    thrust_terms = []

    for i in range(n_cables):
        s_i = s_list[i]
        r_i = r_list[i]
        rdot_i = rdot_list[i]
        t_i = t_list[i]

        a_i_np = a_list_np[i]
        a_i = vertcat(a_i_np[0], a_i_np[1], a_i_np[2])

        # R(q) * a_i 的二阶导数项: R * ( [ω]^2 + [ω̇] ) * a_i
        acc_rot_i = mtimes(R, mtimes(omega_skew_sq + omega_dot_skew, a_i))

        # s_i 的二阶导数: s̈_i = ṙ_i × s_i + r_i × (r_i × s_i)
        s_ddot_i = cross(rdot_i, s_i) + cross(r_i, cross(r_i, s_i))

        # quad 位置的加速度: p̈_i = a_L + acc_rot_i - l * s̈_i
        ddot_p_i = a_load + acc_rot_i - cable_length * s_ddot_i

        # 推力近似向量: m_i (p̈_i - g) - t_i s_i
        force_vec = m_quad * (ddot_p_i - g_vec) - t_i * s_i

        # 使用平方范数以保持光滑: T_i^2 = ||force_vec||^2
        T_sq_i = dot(force_vec, force_vec)
        thrust_terms.append(T_sq_i)

    # 仅将所有无人机的 T_i^2 组成一个向量，用于路径约束（推力硬约束）
    con_h_expr = vertcat(*thrust_terms)

    # -----------------------
    # 构建 AcadosModel
    # -----------------------
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = vertcat()  # 无代数状态
    model.p = vertcat()  # 暂不使用参数
    model.con_h_expr = con_h_expr
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    params = {
        "n_cables": n_cables,
        "g": g_val,
        "m_load": m_load,
        "J_load": J_load_np,
        "dims": np.array([Lx, Ly, Lz]),
        "cable_length": cable_length,
        "attachment_points": np.array(a_list_np),
    }

    return model, params


if __name__ == "__main__":
    # 简单自检: 创建模型并打印维度
    model, params = create_kinodynamic_model(n_cables=3)
    print(f"Model name: {model.name}")
    print(f"Number of states: {model.x.shape[0]}")
    print(f"Number of controls: {model.u.shape[0]}")
    print(f"Params: {params}")
