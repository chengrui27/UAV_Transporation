# 将输入修改为推力的导数和角速度的二阶导数，使无人机运动更加平滑

from acados_template import AcadosModel
from casadi import SX, vertcat

def px4_payload_model():

    model_name = 'px4_payload_model'

    # 参数
    g = 9.81
    mQ = 1.585
    mL = 0.35
    l = 1.0

    # 状态变量 (扩展后, ENU坐标系)
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
    T = SX.sym('T')          # 当前推力
    Omega_x = SX.sym('Omega_x')  # 机体系角速度 x
    Omega_y = SX.sym('Omega_y')  # 机体系角速度 y
    Omega_z = SX.sym('Omega_z')  # 机体系角速度 z
    Omega_dot_x = SX.sym('Omega_dot_x')  # 角速度导数 x
    Omega_dot_y = SX.sym('Omega_dot_y')  # 角速度导数 y
    Omega_dot_z = SX.sym('Omega_dot_z')  # 角速度导数 z
    x = vertcat(
        xp,
        yp,
        zp,
        vxp,
        vyp,
        vzp,
        qx,
        qy,
        qz,
        wx,
        wy,
        wz,
        q0,
        q1,
        q2,
        q3,
        T,
        Omega_x,
        Omega_y,
        Omega_z,
        Omega_dot_x,
        Omega_dot_y,
        Omega_dot_z,
    )

    # 系统控制输入（推力导数 + 角速度二阶导数）
    u_T_dot = SX.sym('u_T_dot')                  # 推力一阶导数
    u_Omega_ddot_x = SX.sym('u_Omega_ddot_x')    # 角速度二阶导数 x
    u_Omega_ddot_y = SX.sym('u_Omega_ddot_y')    # 角速度二阶导数 y
    u_Omega_ddot_z = SX.sym('u_Omega_ddot_z')    # 角速度二阶导数 z
    u = vertcat(u_T_dot, u_Omega_ddot_x, u_Omega_ddot_y, u_Omega_ddot_z)

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
    T_dot_state = SX.sym('T_dot_state')              # 推力导数（状态导数）
    Omega_x_dot_state = SX.sym('Omega_x_dot_state')  # 角速度 x 导数
    Omega_y_dot_state = SX.sym('Omega_y_dot_state')  # 角速度 y 导数
    Omega_z_dot_state = SX.sym('Omega_z_dot_state')  # 角速度 z 导数
    Omega_dot_x_dot_state = SX.sym('Omega_dot_x_dot_state')  # 角加速度 x 导数
    Omega_dot_y_dot_state = SX.sym('Omega_dot_y_dot_state')  # 角加速度 y 导数
    Omega_dot_z_dot_state = SX.sym('Omega_dot_z_dot_state')  # 角加速度 z 导数
    x_dot = vertcat(
        xp_dot,
        yp_dot,
        zp_dot,
        vxp_dot,
        vyp_dot,
        vzp_dot,
        qx_dot,
        qy_dot,
        qz_dot,
        wx_dot,
        wy_dot,
        wz_dot,
        q0_dot,
        q1_dot,
        q2_dot,
        q3_dot,
        T_dot_state,
        Omega_x_dot_state,
        Omega_y_dot_state,
        Omega_z_dot_state,
        Omega_dot_x_dot_state,
        Omega_dot_y_dot_state,
        Omega_dot_z_dot_state,
    )

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

    # 四元数动力学 (公式12转换为标准形式)，使用状态中的机体系角速度
    dot_q0 = 0.5 * (-q1 * Omega_x - q2 * Omega_y - q3 * Omega_z)
    dot_q1 = 0.5 * (q0 * Omega_x - q3 * Omega_y + q2 * Omega_z)
    dot_q2 = 0.5 * (q3 * Omega_x + q0 * Omega_y - q1 * Omega_z)
    dot_q3 = 0.5 * (-q2 * Omega_x + q1 * Omega_y + q0 * Omega_z)

    # 推力和角速度扩展状态动力学
    dot_T = u_T_dot
    dot_Omega_x = Omega_dot_x
    dot_Omega_y = Omega_dot_y
    dot_Omega_z = Omega_dot_z
    dot_Omega_dot_x = u_Omega_ddot_x
    dot_Omega_dot_y = u_Omega_ddot_y
    dot_Omega_dot_z = u_Omega_ddot_z

    # 构建显式表达式
    f_expl = vertcat(
        dot_xp,
        dot_yp,
        dot_zp,
        dot_vxp,
        dot_vyp,
        dot_vzp,
        dot_qx,
        dot_qy,
        dot_qz,
        dot_wx,
        dot_wy,
        dot_wz,
        dot_q0,
        dot_q1,
        dot_q2,
        dot_q3,
        dot_T,
        dot_Omega_x,
        dot_Omega_y,
        dot_Omega_z,
        dot_Omega_dot_x,
        dot_Omega_dot_y,
        dot_Omega_dot_z,
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

