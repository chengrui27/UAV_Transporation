from acados_template import AcadosModel
from casadi import SX, vertcat

def px4_payload_model():

    model_name = 'px4_payload_model'

    # 参数
    g = 9.81
    mQ = 2.095
    mL = 0.5
    l = 1.0

    # 状态变量 (16维,ENU坐标系)
    xp = SX.sym('xp')        # 负载位置 x (East)
    yp = SX.sym('yp')        # 负载位置 y (North)
    zp = SX.sym('zp')        # 负载位置 z (Up)
    vxp = SX.sym('vxp')      # 负载速度 x
    vyp = SX.sym('vyp')      # 负载速度 y
    vzp = SX.sym('vzp')      # 负载速度 z
    qx = SX.sym('qx')        # 绳子方向单位向量 x
    qy = SX.sym('qy')        # 绳子方向单位向量 y
    qz = SX.sym('qz')        # 绳子方向单位向量 z
    wx = SX.sym('wx')        # 绳子角速度 x
    wy = SX.sym('wy')        # 绳子角速度 y
    wz = SX.sym('wz')        # 绳子角速度 z
    q0 = SX.sym('q0')        # 四元数 w
    q1 = SX.sym('q1')        # 四元数 x
    q2 = SX.sym('q2')        # 四元数 y
    q3 = SX.sym('q3')        # 四元数 z
    x = vertcat(xp, yp, zp, vxp, vyp, vzp, qx, qy, qz, wx, wy, wz, q0, q1, q2, q3)

    # 系统控制输入
    T = SX.sym('T')              # 推力
    Omega_x = SX.sym('Omega_x')  # x轴角速度
    Omega_y = SX.sym('Omega_y')  # y轴角速度
    Omega_z = SX.sym('Omega_z')  # z轴角速度
    u = vertcat(T, Omega_x, Omega_y, Omega_z)

    # 状态变量的导数
    xp_dot = SX.sym('xp_dot')        # 负载位置 x
    yp_dot = SX.sym('yp_dot')        # 负载位置 y
    zp_dot = SX.sym('zp_dot')        # 负载位置 z
    vxp_dot = SX.sym('vxp_dot')      # 负载速度 x
    vyp_dot = SX.sym('vyp_dot')      # 负载速度 y
    vzp_dot = SX.sym('vzp_dot')      # 负载速度 z
    qx_dot = SX.sym('qx_dot')        # 绳子方向单位向量 x
    qy_dot = SX.sym('qy_dot')        # 绳子方向单位向量 y
    qz_dot = SX.sym('qz_dot')        # 绳子方向单位向量 z
    wx_dot = SX.sym('wx_dot')        # 绳子角速度 x
    wy_dot = SX.sym('wy_dot')        # 绳子角速度 y
    wz_dot = SX.sym('wz_dot')        # 绳子角速度 z
    q0_dot = SX.sym('q0_dot')        # 四元数 w
    q1_dot = SX.sym('q1_dot')        # 四元数 x
    q2_dot = SX.sym('q2_dot')        # 四元数 y
    q3_dot = SX.sym('q3_dot')        # 四元数 z
    x_dot = vertcat(xp_dot, yp_dot, zp_dot, vxp_dot, vyp_dot, vzp_dot, qx_dot, qy_dot, qz_dot, wx_dot, wy_dot, wz_dot, q0_dot, q1_dot, q2_dot, q3_dot)

    # 从四元数计算旋转矩阵的第三列 (论文中的r3)
    r3x = 2 * (q1*q3 + q0*q2)
    r3y = 2 * (q2*q3 - q0*q1)
    r3z = q0*q0 - q1*q1 - q2*q2 + q3*q3

    # 负载位置动力学 (公式8)
    dot_xp = vxp
    dot_yp = vyp
    dot_zp = vzp

    # 推力向量 (ENU+FLU坐标系)
    Fx = T * r3x
    Fy = T * r3y
    Fz = T * r3z

    # 计算投影 P_q(F) = q * (q^T * F)，沿绳子方向的投影
    qT_F = qx * Fx + qy * Fy + qz * Fz  # q^T * F (标量)
    Pq_Fx = qx * qT_F
    Pq_Fy = qy * qT_F
    Pq_Fz = qz * qT_F

    # 绳子张力的影响项 (离心力项)
    rope_tension_factor = mQ * l * (wx*wx + wy*wy + wz*wz)

    # 负载速度动力学 (公式9)
    # ENU坐标系: 重力沿-z方向 (Up为正，重力向下为负)
    dot_vxp = Pq_Fx / (mQ + mL) - rope_tension_factor * qx / (mQ + mL)
    dot_vyp = Pq_Fy / (mQ + mL) - rope_tension_factor * qy / (mQ + mL)
    dot_vzp = Pq_Fz / (mQ + mL) - rope_tension_factor * qz / (mQ + mL) - g

    # 绳子方向动力学 (公式10)
    dot_qx = wy * qz - wz * qy
    dot_qy = wz * qx - wx * qz
    dot_qz = wx * qy - wy * qx

    # 绳子角速度动力学 (公式11)
    # ω̇ = -S(q)F / (m_Q ℓ) = (q × F) / (m_Q ℓ)
    # 在ENU+FLU中: F = +T * r3，所以 q × F = +T * (q × r3)
    dot_wx = -T * (qy * r3z - qz * r3y) / (mQ * l)
    dot_wy = -T * (qz * r3x - qx * r3z) / (mQ * l)
    dot_wz = -T * (qx * r3y - qy * r3x) / (mQ * l)

    # 四元数动力学 (公式12转换为标准形式)
    dot_q0 = 0.5 * (-q1 * Omega_x - q2 * Omega_y - q3 * Omega_z)
    dot_q1 = 0.5 * (q0 * Omega_x - q3 * Omega_y + q2 * Omega_z)
    dot_q2 = 0.5 * (q3 * Omega_x + q0 * Omega_y - q1 * Omega_z)
    dot_q3 = 0.5 * (-q2 * Omega_x + q1 * Omega_y + q0 * Omega_z)

    # 构建显式表达式
    f_expl = vertcat(
        dot_xp, dot_yp, dot_zp,
        dot_vxp, dot_vyp, dot_vzp,
        dot_qx, dot_qy, dot_qz,
        dot_wx, dot_wy, dot_wz,
        dot_q0, dot_q1, dot_q2, dot_q3
    )

    # 构建隐式表达式
    f_impl = x_dot - f_expl

    z = []
    p = []

    # 创建acados模型
    model = AcadosModel()

    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    return model


