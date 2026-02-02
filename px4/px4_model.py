from acados_template import AcadosModel
from casadi import SX, vertcat

def px4_model():
    model_name = 'px4_model'

    # 参数
    g = 9.81
    m = 1.535

    # 状态变量（NED坐标系）
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    qw = SX.sym('qw')
    qx = SX.sym('qx')
    qy = SX.sym('qy')
    qz = SX.sym('qz')
    T = SX.sym('T')
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    d_wx = SX.sym('d_wx')
    d_wy = SX.sym('d_wy')
    d_wz = SX.sym('d_wz')
    x = vertcat(px, py, pz, vx, vy ,vz, qw, qx, qy, qz, T, wx, wy, wz, d_wx, d_wy, d_wz)

    # 系统控制输入：推力和三轴角速度(FRD坐标系)
    d_T = SX.sym('d_T')      # 推力导数
    dd_wx = SX.sym('dd_wx')  # roll轴角速度二阶导数
    dd_wy = SX.sym('dd_wy')  # pitch轴角速度二阶导数
    dd_wz = SX.sym('dd_wz')  # yaw轴角速度二阶导数
    u = vertcat(d_T, dd_wx, dd_wy, dd_wz)

    # 状态变量的导数
    px_dot = SX.sym('px_dot')
    py_dot = SX.sym('py_dot')
    pz_dot = SX.sym('pz_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')
    qw_dot = SX.sym('qw_dot')
    qx_dot = SX.sym('qx_dot')
    qy_dot = SX.sym('qy_dot')
    qz_dot = SX.sym('qz_dot')
    T_dot = SX.sym('d_T_dot')
    wx_dot = SX.sym('wx_dot')
    wy_dot = SX.sym('wy_dot')
    wz_dot = SX.sym('wz_dot')
    d_wx_dot = SX.sym('d_wx_dot')
    d_wy_dot = SX.sym('d_wy_dot')
    d_wz_dot = SX.sym('d_wz_dot')
    x_dot = vertcat(px_dot, py_dot, pz_dot, vx_dot, vy_dot ,vz_dot, qw_dot, qx_dot, qy_dot, qz_dot, T_dot, wx_dot, wy_dot, wz_dot, d_wx_dot, d_wy_dot, d_wz_dot)

    # 动力学方程
    px_d = vx
    py_d = vy
    pz_d = vz
    
    # 从四元数计算旋转矩阵的第三列 (论文中的r3)
    r3x = 2 * (qx*qz + qw*qy)
    r3y = 2 * (qy*qz - qw*qx)
    r3z = qw*qw - qx*qx - qy*qy + qz*qz

    vx_d = (T * r3x) / m
    vy_d = (T * r3y) / m
    vz_d = (T * r3z) / m - g

    qw_d = (-qx*wx - qy*wy - qz*wz) / 2
    qx_d = ( qw*wx - qz*wy + qy*wz) / 2
    qy_d = ( qz*wx + qw*wy - qx*wz) / 2
    qz_d = ( qx*wy - qy*wx + qw*wz) / 2

    T_dot = d_T
    wx_dot = d_wx
    wy_dot = d_wy
    wz_dot = d_wz
    d_wx_dot = dd_wx
    d_wy_dot = dd_wy
    d_wz_dot = dd_wz

    f_expl = vertcat(px_d, py_d, pz_d, vx_d, vy_d, vz_d, qw_d, qx_d, qy_d, qz_d, T_dot, wx_dot, wy_dot, wz_dot, d_wx_dot, d_wy_dot, d_wz_dot)  # 构建显式表达式
    f_impl = x_dot - f_expl  # 构建隐式表达式

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