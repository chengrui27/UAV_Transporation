"""
Multi-Quadrotor Payload Transportation Model using Acados
Based on: "Geometric Control of Multiple Quadrotors Transporting a Rigid-body Load"
by Guofan Wu and Koushil Sreenath (CDC 2014)

System: 3 quadrotors transporting a rectangular payload via cables
"""

import numpy as np
from acados_template import AcadosModel
from casadi import SX, vertcat, cos, sin, norm_2, cross, mtimes, sqrt, inv, dot

def create_multi_payload_model():
    """
    Create the dynamic model for multiple quadrotors with rigid-body load.

    System Configuration:
    - Payload: Rectangular box (0.6m x 0.4m x 0.2m, mass 1kg)
    - 3 quadrotors with cables attached on a circle in the x-y plane

    State Variables (using quaternions for attitude):
    - Load: position xL (3), velocity vL (3), quaternion qL (4), angular velocity Omega_L (3)
    - Cables: direction qi (3), angular velocity omega_i (3) for each cable, i=1,2,3
    - Quadrotors: quaternion qRi (4), thrust fi (1), angular velocity Omega_i (3),
      angular acceleration Omega_dot_i (3) for each quadrotor, i=1,2,3

    Total states: 13 (load) + 18 (3 cables) + 33 (3 quadrotors) = 64

    Control Inputs (Modified from paper):
    - Thrust derivative f_dot_i (1) and angular velocity second derivative Omega_ddot_i (3)
      for each quadrotor, i=1,2,3
    Total controls: 12
    """

    model_name = 'multi_payload'

    # System parameters
    n_quad = 3  # Number of quadrotors
    g = 9.81  # Gravity acceleration

    # Load parameters (rectangular box)
    m_L = 1.0  # Mass of load [kg]
    Lx = 1.2  # Length in body x [m]
    Ly = 0.8  # Length in body y [m]
    Lz = 0.2  # Length in body z [m]

    # Inertia matrix of rectangular box (body frame)
    J_Lxx = (m_L / 12.0) * (Ly**2 + Lz**2)
    J_Lyy = (m_L / 12.0) * (Lx**2 + Lz**2)
    J_Lzz = (m_L / 12.0) * (Lx**2 + Ly**2)
    J_L = np.diag([J_Lxx, J_Lyy, J_Lzz])

    # Quadrotor parameters
    m_i = 1.535  # Mass of each quadrotor [kg]
    J_i = np.diag([0.0023, 0.0023, 0.004])  # Inertia matrix of quadrotor（Not used in current model）

    # Cable parameters
    # Attachment points in payload body frame (match rect_payload_box model.sdf)
    r_b = np.array([
        [0.6, 0.0, 0.0],
        [-0.3, 0.4, 0.0],
        [-0.3, -0.4, 0.0],
    ])

    # Cable lengths
    L_cable = np.full(n_quad, 1.5)  # Cable length [m]

    # ------------------------
    # State Variables
    # ------------------------
    # Load states
    x_L = SX.sym('x_L', 3)  # Load position in inertial frame
    v_L = SX.sym('v_L', 3)  # Load velocity in inertial frame
    q_L = SX.sym('q_L', 4)  # Load quaternion (w, x, y, z) - rotation from body to inertial
    Omega_L = SX.sym('Omega_L', 3)  # Load angular velocity in body frame

    # Cable states (direction vectors and angular velocities)
    q_cables = []  # Direction vectors qi in inertial frame
    omega_cables = []  # Angular velocities
    for i in range(n_quad):
        q_cables.append(SX.sym(f'q_{i+1}', 3))  # Cable direction (unit vector)
        omega_cables.append(SX.sym(f'omega_{i+1}', 3))  # Cable angular velocity

    # Quadrotor states (attitude, thrust, angular velocity and its derivative)
    q_R = []  # Quadrotor quaternions
    f_thrust = []  # Thrust magnitude
    Omega_R = []  # Angular velocity
    Omega_dot_R = []  # Angular acceleration
    for i in range(n_quad):
        q_R.append(SX.sym(f'q_R{i+1}', 4))  # Quadrotor quaternion (w, x, y, z)
        f_thrust.append(SX.sym(f'f_{i+1}'))
        Omega_R.append(SX.sym(f'Omega_R{i+1}', 3))
        Omega_dot_R.append(SX.sym(f'Omega_dot_R{i+1}', 3))

    # Concatenate all states
    x = vertcat(x_L, v_L, q_L, Omega_L)
    for i in range(n_quad):
        x = vertcat(x, q_cables[i], omega_cables[i])
    for i in range(n_quad):
        x = vertcat(x, q_R[i], f_thrust[i], Omega_R[i], Omega_dot_R[i])

    # ------------------------
    # Control Inputs
    # ------------------------
    # Control: thrust derivative and angular velocity second derivative for each quadrotor
    controls = []
    f_dot = []
    Omega_ddot = []
    for i in range(n_quad):
        f_dot.append(SX.sym(f'f_dot_{i+1}'))  # Thrust derivative
        Omega_ddot.append(SX.sym(f'Omega_ddot_{i+1}', 3))  # Angular velocity second derivative
        controls.append(f_dot[i])
        controls.append(Omega_ddot[i])

    u = vertcat(*controls)

    # ------------------------
    # Helper Functions
    # ------------------------
    def quat_to_rotation_matrix(q):
        """Convert quaternion to rotation matrix (inertial to body)"""
        # q = [qw, qx, qy, qz]
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        R = vertcat(
            vertcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)).T,
            vertcat(2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)).T,
            vertcat(2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)).T
        )
        return R

    def quat_multiply(q1, q2):
        """Quaternion multiplication"""
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
        w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

        return vertcat(
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )

    def omega_to_quat_dot(q, omega):
        """Convert angular velocity to quaternion derivative"""
        # q_dot = 0.5 * q � [0; omega]
        omega_quat = vertcat(0, omega)
        q_dot = 0.5 * quat_multiply(q, omega_quat)
        return q_dot

    def skew(v):
        """Skew-symmetric matrix from vector"""
        return vertcat(
            vertcat(0, -v[2], v[1]).T,
            vertcat(v[2], 0, -v[0]).T,
            vertcat(-v[1], v[0], 0).T
        )

    # ------------------------
    # Dynamics
    # ------------------------
    # Gravity vector in inertial frame
    e3 = vertcat(0, 0, 1)
    g_vec = g * e3

    # Load rotation matrix (body to inertial)
    R_L = quat_to_rotation_matrix(q_L)

    # Cable directions in body frame
    q_ib = []
    for i in range(n_quad):
        q_ib.append(mtimes(R_L.T, q_cables[i]))

    # Attachment points in inertial frame
    r_i = []
    r_i_inertial = []
    for i in range(n_quad):
        r_i.append(vertcat(r_b[i, 0], r_b[i, 1], r_b[i, 2]))
        r_i_inertial.append(mtimes(R_L, r_i[i]))

    # Quadrotor positions in inertial frame
    p_quad = []
    for i in range(n_quad):
        p_quad.append(x_L + r_i_inertial[i] - L_cable[i] * q_cables[i])

    # Pairwise collision avoidance constraints (soft constraints in OCP)
    d_safe = 0.2
    h_list = []
    for i in range(n_quad):
        for j in range(i + 1, n_quad):
            delta = p_quad[i] - p_quad[j]
            h_list.append(dot(delta, delta) - d_safe ** 2)

    # Build matrices A, G, d from paper (Section II, page 3)
    # A = [[A11, A12], [A21, A22]]
    A11 = m_L * SX.eye(3)
    A12 = SX.zeros(3, 3)
    A22 = SX(J_L)

    for i in range(n_quad):
        A11 += m_i * mtimes(q_cables[i], q_cables[i].T)
        A12 -= m_i * mtimes(mtimes(q_cables[i], q_ib[i].T), skew(r_i[i]))
        A22 += m_i * mtimes(mtimes(skew(r_i[i]), q_ib[i]), mtimes(q_ib[i].T, skew(r_i[i]).T))

    A21 = A12.T
    A_matrix = vertcat(
        vertcat(A11.T, A21.T).T,
        vertcat(A12.T, A22.T).T
    )

    # G matrix
    G_list = []
    for i in range(n_quad):
        G_i = vertcat(
            m_i * L_cable[i] * q_cables[i],
            m_i * L_cable[i] * mtimes(skew(r_i[i]), q_ib[i])
        )
        G_list.append(G_i)

    G_matrix = vertcat(*[g.T for g in G_list]).T

    # d vector (nonlinear terms)
    d1 = SX.zeros(3, 1)
    d2 = -mtimes(skew(Omega_L), mtimes(SX(J_L), Omega_L))

    Omega_L_skew = skew(Omega_L)
    for i in range(n_quad):
        omega_dot_omega = mtimes(omega_cables[i].T, omega_cables[i])
        r_term = mtimes(mtimes(q_ib[i].T, Omega_L_skew), mtimes(Omega_L_skew, r_i[i]))

        d1 -= m_i * (r_term + L_cable[i] * omega_dot_omega) * q_cables[i]
        d2 -= m_i * (r_term + L_cable[i] * omega_dot_omega) * mtimes(skew(r_i[i]), q_ib[i])

    d_vec = vertcat(d1, d2)

    # Control input vector u_i = f_i/(m_i*L_i) * R_i * e3
    u_i_list = []
    for i in range(n_quad):
        R_i = quat_to_rotation_matrix(q_R[i])
        u_i_list.append((f_thrust[i] / (m_i * L_cable[i])) * mtimes(R_i, e3))

    # Build RHS of load dynamics
    # A * [a_L; Omega_dot_L] = sum(G_i * (q_i � u_i - omega_i � omega_i)) + d
    G_u = SX.zeros(6, 1)
    for i in range(n_quad):
        q_dot_u = mtimes(q_cables[i].T, u_i_list[i])
        omega_dot_omega = mtimes(omega_cables[i].T, omega_cables[i])
        G_u += G_list[i] * (q_dot_u - omega_dot_omega)

    # Solve for accelerations: [a_L; Omega_dot_L] = A^(-1) * (G_u + d)
    acc_vec = mtimes(inv(A_matrix), G_u + d_vec)
    a_L = acc_vec[0:3]
    Omega_dot_L = acc_vec[3:6]

    # Load dynamics
    x_L_dot = v_L
    v_L_dot = a_L - g_vec
    q_L_dot = omega_to_quat_dot(q_L, Omega_L)

    # Cable dynamics for each cable
    q_cables_dot = []
    omega_cables_dot = []

    for i in range(n_quad):
        # q_dot_i = omega_i � q_i
        q_cables_dot.append(cross(omega_cables[i], q_cables[i]))

        # omega_dot_i = q_i � [-u_i + (1/L_i)*(a_L + g*e3 + R_L*(Omega_L_skew^2 + Omega_dot_L_skew)*r_i)]
        Omega_dot_L_skew = skew(Omega_dot_L)
        acc_term = a_L + g_vec + mtimes(R_L, mtimes(Omega_L_skew, mtimes(Omega_L_skew, r_i[i])) + mtimes(Omega_dot_L_skew, r_i[i]))

        omega_cables_dot.append(
            cross(q_cables[i], -u_i_list[i] + (1.0 / L_cable[i]) * acc_term)
        )

    # Quadrotor attitude kinematics (only quaternion derivative, angular velocity is input)
    q_R_dot = []

    for i in range(n_quad):
        # Quaternion kinematics: q_dot = 0.5 * q o [0; Omega]
        q_R_dot.append(omega_to_quat_dot(q_R[i], Omega_R[i]))

    # Concatenate all dynamics
    x_dot = vertcat(x_L_dot, v_L_dot, q_L_dot, Omega_dot_L)
    for i in range(n_quad):
        x_dot = vertcat(x_dot, q_cables_dot[i], omega_cables_dot[i])
    for i in range(n_quad):
        f_thrust_dot = f_dot[i]
        Omega_R_dot = Omega_dot_R[i]
        Omega_dot_R_dot = Omega_ddot[i]
        x_dot = vertcat(x_dot, q_R_dot[i], f_thrust_dot, Omega_R_dot, Omega_dot_R_dot)

    # Create Acados model
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = SX.sym('xdot', x.shape[0])
    model.u = u
    model.f_expl_expr = x_dot
    model.f_impl_expr = model.xdot - x_dot
    model.con_h_expr = vertcat(*h_list) if h_list else vertcat()

    # Store parameters for reference
    model.p = vertcat()  # No parameters for now

    return model, {
        'n_quad': n_quad,
        'g': g,
        'm_L': m_L,
        'J_L': J_L,
        'm_i': m_i,
        'J_i': J_i,
        'L_cable': L_cable,
        'r_b': r_b,
        'dims': np.array([Lx, Ly, Lz]),
        'd_safe': d_safe
    }


if __name__ == '__main__':
    # Test model creation
    model, params = create_multi_payload_model()

    print(f"Model name: {model.name}")
    print(f"Number of states: {model.x.shape[0]}")
    print(f"Number of controls: {model.u.shape[0]}")
    print(f"\nSystem parameters:")
    print(f"  Number of quadrotors: {params['n_quad']}")
    print(f"  Load mass: {params['m_L']} kg")
    print(f"  Load inertia:\n{params['J_L']}")
    print(f"  Quadrotor mass: {params['m_i']} kg")
    print(f"  Cable lengths: {params['L_cable']} m")
    print(f"  Load dimensions: {params['dims']} m")
    print(f"  Attachment points (body frame):\n{params['r_b']}")
    print("\nModel created successfully!")
