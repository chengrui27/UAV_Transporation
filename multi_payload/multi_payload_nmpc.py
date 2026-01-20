# -*- coding: utf-8 -*-
"""
Multi-quadrotor rigid-body payload NMPC solver and simulation
Based on: "Geometric Control of Multiple Quadrotors Transporting a Rigid-body Load"

System: 3 quadrotors carrying a rectangular payload via cables
"""

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from multi_payload_model import create_multi_payload_model
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

np.set_printoptions(suppress=True, precision=4)


def create_nmpc_solver():
    """Create NMPC solver"""

    # Create acados OCP object
    ocp = AcadosOcp()

    # Load model
    model, params = create_multi_payload_model()
    ocp.model = model
    n_quad = params['n_quad']

    # Get dimensions
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    print(f"State dimension nx = {nx}")
    print(f"Control dimension nu = {nu}")

    # Set cost function type
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # State weight matrix
    Q = np.eye(nx)

    # Load position weights (0-2)
    Q[0, 0] = 100.0    # x position
    Q[1, 1] = 100.0    # y position
    Q[2, 2] = 100.0    # z position

    # Load velocity weights (3-5)
    Q[3, 3] = 20.0     # vx
    Q[4, 4] = 20.0     # vy
    Q[5, 5] = 20.0     # vz

    # Load quaternion weights (6-9)
    Q[6, 6] = 50.0     # qw
    Q[7, 7] = 50.0     # qx
    Q[8, 8] = 50.0     # qy
    Q[9, 9] = 50.0     # qz

    # Load angular velocity weights (10-12)
    Q[10, 10] = 10.0   # Omega_x
    Q[11, 11] = 10.0   # Omega_y
    Q[12, 12] = 10.0   # Omega_z

    # Cable direction and angular velocity weights (3 cables x 6 dims)
    cable_state_start = 13
    quad_state_start = cable_state_start + 6 * n_quad
    for i in range(cable_state_start, quad_state_start):
        Q[i, i] = 5.0

    # Quadrotor state weights (q_R, thrust, Omega_R, Omega_dot_R)
    for i in range(n_quad):
        base = quad_state_start + 11 * i
        for j in range(4):
            Q[base + j, base + j] = 20.0
        Q[base + 4, base + 4] = 5.0
        for j in range(3):
            Q[base + 5 + j, base + 5 + j] = 5.0
            Q[base + 8 + j, base + 8 + j] = 1.0

    # Control input weight matrix (thrust derivative + angular velocity second derivative)
    R = np.eye(nu)
    for i in range(n_quad):
        R[4 * i, 4 * i] = 1.0
        R[4 * i + 1, 4 * i + 1] = 1.0
        R[4 * i + 2, 4 * i + 2] = 1.0
        R[4 * i + 3, 4 * i + 3] = 1.0

    # Set process and terminal weight matrices
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = 20.0 * Q

    # Projection matrix Vx (state part)
    Vx = np.zeros((ny, nx))
    for i in range(nx):
        Vx[i, i] = 1.0
    ocp.cost.Vx = Vx

    # Projection matrix Vu (control input part)
    Vu = np.zeros((ny, nu))
    for i in range(nu):
        Vu[nx + i, i] = 1.0
    ocp.cost.Vu = Vu

    # Terminal projection matrix
    Vx_e = np.eye(ny_e)
    ocp.cost.Vx_e = Vx_e

    # Set reference trajectory (ENU frame: East-North-Up)
    # Target: load hover at (2, 2, 4) position
    yref = np.zeros(ny)

    # Load reference position and velocity
    yref[0:3] = [2.0, 2.0, 4.0]  # position (x, y, z)
    yref[3:6] = [0.0, 0.0, 0.0]  # velocity

    # Load reference attitude (unit quaternion: no rotation)
    yref[6:10] = [1.0, 0.0, 0.0, 0.0]  # qL = [qw, qx, qy, qz]

    # Load reference angular velocity
    yref[10:13] = [0.0, 0.0, 0.0]

    # Cable reference direction (vertically downward: from quad to load)
    # Note: In ENU inertial frame, z-axis is up, so downward is (0, 0, -1)
    for i in range(n_quad):
        yref[13 + 6*i : 13 + 6*i + 3] = [0.0, 0.0, -1.0]  # qi direction
        yref[13 + 6*i + 3 : 13 + 6*i + 6] = [0.0, 0.0, 0.0]  # omega_i

    g = params['g']
    m_L = params['m_L']
    m_i = params['m_i']
    total_mass = m_L + n_quad * m_i
    thrust_hover = (total_mass * g) / float(n_quad)

    # Quadrotor reference (attitude, thrust, angular velocity, angular acceleration)
    for i in range(n_quad):
        base = quad_state_start + 11 * i
        yref[base : base + 4] = [1.0, 0.0, 0.0, 0.0]
        yref[base + 4] = thrust_hover
        yref[base + 5 : base + 8] = [0.0, 0.0, 0.0]
        yref[base + 8 : base + 11] = [0.0, 0.0, 0.0]

    ocp.cost.yref = yref
    ocp.cost.yref_e = yref[:nx]

    # Set control input constraints
    lbu = []
    ubu = []
    for i in range(n_quad):
        lbu.extend([-50.0, -50.0, -50.0, -50.0])
        ubu.extend([50.0, 50.0, 50.0, 50.0])

    ocp.constraints.lbu = np.array(lbu)
    ocp.constraints.ubu = np.array(ubu)
    ocp.constraints.idxbu = np.arange(nu)

    # Soft collision avoidance constraints between quadrotors
    n_pairs = n_quad * (n_quad - 1) // 2
    if n_pairs > 0:
        ocp.constraints.lh = np.zeros(n_pairs)
        ocp.constraints.uh = np.full(n_pairs, 1e3)
        ocp.constraints.idxsh = np.arange(n_pairs, dtype=int)
        ocp.constraints.lsh = np.zeros(n_pairs)
        ocp.constraints.ush = np.full(n_pairs, 1e3)

        soft_weight = 5e3
        ocp.cost.Zl = 2.0 * soft_weight * np.ones(n_pairs)
        ocp.cost.Zu = 2.0 * soft_weight * np.ones(n_pairs)
        ocp.cost.zl = 2.0 * soft_weight * np.ones(n_pairs)
        ocp.cost.zu = 2.0 * soft_weight * np.ones(n_pairs)

    # Initial state (ENU inertial frame)
    x0 = np.zeros(nx)

    # Load initial position and velocity
    x0[0:3] = [0.0, 0.0, 2.0]   # starting position
    x0[3:6] = [0.0, 0.0, 0.0]   # initial velocity

    # Load initial attitude (unit quaternion)
    x0[6:10] = [1.0, 0.0, 0.0, 0.0]  # qL

    # Load initial angular velocity
    x0[10:13] = [0.0, 0.0, 0.0]

    # Cable initial direction (vertically downward)
    for i in range(n_quad):
        x0[13 + 6*i : 13 + 6*i + 3] = [0.0, 0.0, -1.0]  # qi
        x0[13 + 6*i + 3 : 13 + 6*i + 6] = [0.0, 0.0, 0.0]  # omega_i

    # Quadrotor initial state (attitude, thrust, angular velocity, angular acceleration)
    for i in range(n_quad):
        base = quad_state_start + 11 * i
        x0[base : base + 4] = [1.0, 0.0, 0.0, 0.0]
        x0[base + 4] = thrust_hover
        x0[base + 5 : base + 11] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    ocp.constraints.x0 = x0

    # Set solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0

    # Set prediction horizon
    N = 20   # prediction steps
    T = 2.0  # prediction horizon (seconds)
    ocp.dims.N = N
    ocp.solver_options.tf = T

    # Create solver
    print("Creating NMPC solver...")
    acados_ocp = AcadosOcpSolver(ocp, json_file='multi_payload_ocp.json')
    acados_sim = AcadosSimSolver(ocp, json_file='multi_payload_ocp.json')
    print("Solver created successfully!")

    return acados_ocp, acados_sim, params


def nmpc_sim():
    """Run NMPC simulation"""

    # Create solver
    ocp_solver, integrator, params = create_nmpc_solver()

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    n_quad = params['n_quad']
    quad_state_start = 13 + 6 * n_quad
    g = params['g']
    m_L = params['m_L']
    m_i = params['m_i']
    total_mass = m_L + n_quad * m_i
    thrust_hover = (total_mass * g) / float(n_quad)

    # Initial state
    x0 = np.zeros(nx)
    x0[0:3] = [0.0, 0.0, 2.0]        # load initial position
    x0[6:10] = [1.0, 0.0, 0.0, 0.0]  # load initial attitude

    # Cable initial direction (vertically downward)
    for i in range(n_quad):
        x0[13 + 6*i : 13 + 6*i + 3] = [0.0, 0.0, -1.0]

    # Quadrotor initial state (attitude, thrust, angular velocity, angular acceleration)
    for i in range(n_quad):
        base = quad_state_start + 11 * i
        x0[base : base + 4] = [1.0, 0.0, 0.0, 0.0]
        x0[base + 4] = thrust_hover

    # Simulation settings
    Nsim = 100  # simulation steps
    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))

    t_preparation = np.zeros(Nsim)
    t_feedback = np.zeros(Nsim)

    simX[0, :] = x0

    print(f"\nStarting simulation ({Nsim} steps)...")

    # Simulation loop
    for i in range(Nsim):

        # Phase 1: Preparation phase - linearization
        ocp_solver.options_set('rti_phase', 1)
        status = ocp_solver.solve()
        t_preparation[i] = ocp_solver.get_stats('time_tot')

        if status not in [0, 2, 5]:
            print(f"Warning: Step {i} preparation returned status {status}")

        # Update initial state constraint
        ocp_solver.set(0, "lbx", simX[i, :])
        ocp_solver.set(0, "ubx", simX[i, :])

        # Phase 2: Feedback phase - solve QP
        ocp_solver.options_set('rti_phase', 2)
        status = ocp_solver.solve()
        t_feedback[i] = ocp_solver.get_stats('time_tot')

        if status not in [0, 2, 5]:
            print(f"Warning: Step {i} feedback returned status {status}")

        # Get control input
        simU[i, :] = ocp_solver.get(0, "u")

        # Simulate next state
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])

        # Print progress
        if (i + 1) % 30 == 0:
            print(f"  Step: {i+1}/{Nsim} - Load position: [{simX[i+1, 0]:.3f}, {simX[i+1, 1]:.3f}, {simX[i+1, 2]:.3f}]")

    # Compute time statistics
    t_feedback *= 1000
    t_preparation *= 1000

    print("\n" + "="*70)
    print("Solution time statistics (milliseconds):")
    print("="*70)
    print(f"Preparation: min {np.min(t_preparation):.3f} | median {np.median(t_preparation):.3f} | max {np.max(t_preparation):.3f}")
    print(f"Feedback:    min {np.min(t_feedback):.3f} | median {np.median(t_feedback):.3f} | max {np.max(t_feedback):.3f}")
    print("="*70)

    # Print final state
    print("\n" + "="*70)
    print("Final state:")
    print("="*70)
    print(f"Load position: [{simX[-1, 0]:.4f}, {simX[-1, 1]:.4f}, {simX[-1, 2]:.4f}] m")
    print(f"Load velocity: [{simX[-1, 3]:.4f}, {simX[-1, 4]:.4f}, {simX[-1, 5]:.4f}] m/s")
    print(f"Load attitude: [{simX[-1, 6]:.4f}, {simX[-1, 7]:.4f}, {simX[-1, 8]:.4f}, {simX[-1, 9]:.4f}]")
    print("="*70 + "\n")

    return simX, simU, params


def quaternion_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix"""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    # Normalize
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])

    return R


def visualize_multi_payload(simX, simU, params, interval=50, save_animation=False):
    """
    Visualize 3D animation of 3 quadrotors carrying rigid-body payload
    """

    # Extract parameters
    L_cable = params['L_cable']
    r_b = params['r_b']
    Lx, Ly, Lz = params['dims']
    n_quad = params['n_quad']
    quad_state_start = 13 + 6 * n_quad
    thrust_indices = [quad_state_start + 11 * i + 4 for i in range(n_quad)]

    # Create figure
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Extract load position and attitude
    load_pos = simX[:, 0:3]  # (x, y, z)
    load_quat = simX[:, 6:10]  # (qw, qx, qy, qz)

    # Extract cable directions
    q1 = simX[:, 13:16]
    q2 = simX[:, 19:22]
    q3 = simX[:, 25:28]

    # Compute quadrotor positions
    # Quadrotor i position = load position + R_L * r_i - L_i * q_i
    n_steps = len(simX)
    drone_pos = np.zeros((n_steps, 3, 3))  # (time steps, quad number, xyz)

    for t in range(n_steps):
        R_L = quaternion_to_rotation_matrix(load_quat[t])

        # Three quadrotors
        for i in range(3):
            r_i = r_b[i]
            q_i = simX[t, 13 + 6*i : 13 + 6*i + 3]
            L_i = L_cable[i]

            # x_i = x_L + R_L * r_i - L_i * q_i
            drone_pos[t, i, :] = load_pos[t] + R_L @ r_i - L_i * q_i

    # Set figure boundaries
    all_x = np.concatenate([load_pos[:, 0], drone_pos[:, 0, 0], drone_pos[:, 1, 0], drone_pos[:, 2, 0]])
    all_y = np.concatenate([load_pos[:, 1], drone_pos[:, 0, 1], drone_pos[:, 1, 1], drone_pos[:, 2, 1]])
    all_z = np.concatenate([load_pos[:, 2], drone_pos[:, 0, 2], drone_pos[:, 1, 2], drone_pos[:, 2, 2]])

    max_range = np.array([all_x.max() - all_x.min(),
                          all_y.max() - all_y.min(),
                          all_z.max() - all_z.min()]).max() / 2.0

    mid_x = (all_x.max() + all_x.min()) * 0.5
    mid_y = (all_y.max() + all_y.min()) * 0.5
    mid_z = (all_z.max() + all_z.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(max(0, mid_z - max_range), mid_z + max_range)

    # Set labels
    ax.set_xlabel('East (m)', fontsize=11)
    ax.set_ylabel('North (m)', fontsize=11)
    ax.set_zlabel('Up (m)', fontsize=11)
    ax.set_title('Three-Quadrotor Rigid-Body Payload System', fontsize=14, fontweight='bold')

    # Plot trajectories
    ax.plot(load_pos[:, 0], load_pos[:, 1], load_pos[:, 2],
            'b-', alpha=0.3, linewidth=2, label='Load trajectory')

    colors = ['red', 'green', 'orange']
    for i in range(3):
        ax.plot(drone_pos[:, i, 0], drone_pos[:, i, 1], drone_pos[:, i, 2],
                '--', color=colors[i], alpha=0.3, linewidth=1, label=f'Quad {i+1} trajectory')

    # Initialize animation elements
    # Quadrotors
    drone_points = []
    for i in range(3):
        pt, = ax.plot([], [], [], 'o', color=colors[i], markersize=12, label=f'Quad {i+1}')
        drone_points.append(pt)

    # Load (represented by body)
    load_body = None

    # Cables
    rope_lines = []
    for i in range(3):
        line, = ax.plot([], [], [], '-', color=colors[i], linewidth=2.5)
        rope_lines.append(line)

    # Time text
    time_text = ax.text2D(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                          fontweight='bold', verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

    # State info text
    state_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes, fontsize=9,
                          verticalalignment='top', family='monospace',
                          bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

    ax.legend(loc='upper right', fontsize=9)

    # Animation update function
    def update(frame):
        nonlocal load_body

        # Update quadrotor positions
        for i in range(3):
            drone_points[i].set_data([drone_pos[frame, i, 0]], [drone_pos[frame, i, 1]])
            drone_points[i].set_3d_properties([drone_pos[frame, i, 2]])

        # Update cables
        R_L = quaternion_to_rotation_matrix(load_quat[frame])
        for i in range(3):
            attach_pt = load_pos[frame] + R_L @ r_b[i]
            rope_lines[i].set_data([drone_pos[frame, i, 0], attach_pt[0]],
                                  [drone_pos[frame, i, 1], attach_pt[1]])
            rope_lines[i].set_3d_properties([drone_pos[frame, i, 2], attach_pt[2]])

        # Update load (draw rectangular box)
        if load_body is not None:
            load_body.remove()

        # Box vertices (in load body frame)
        hx, hy, hz = 0.5 * Lx, 0.5 * Ly, 0.5 * Lz
        corners_body = np.array(
            [
                [hx, hy, hz],
                [hx, hy, -hz],
                [hx, -hy, hz],
                [hx, -hy, -hz],
                [-hx, hy, hz],
                [-hx, hy, -hz],
                [-hx, -hy, hz],
                [-hx, -hy, -hz],
            ]
        )

        corners_world = [load_pos[frame] + R_L @ corner for corner in corners_body]

        face_indices = [
            [0, 1, 3, 2],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 2, 6, 4],
            [1, 3, 7, 5],
        ]

        faces = [[corners_world[idx] for idx in face] for face in face_indices]

        load_body = Poly3DCollection(faces, alpha=0.3, facecolor='cyan', edgecolor='blue', linewidths=2)
        ax.add_collection3d(load_body)

        # Update time
        dt = 0.02  # Assume 20ms time step
        time_text.set_text(f'Time: {frame * dt:.2f} s')

        # Update state info
        state_str = '--- System State ---\n'
        state_str += f'Load pos: ({load_pos[frame, 0]:5.2f}, {load_pos[frame, 1]:5.2f}, {load_pos[frame, 2]:5.2f}) m\n'
        for i in range(3):
            state_str += f'Quad {i+1}:   ({drone_pos[frame, i, 0]:5.2f}, {drone_pos[frame, i, 1]:5.2f}, {drone_pos[frame, i, 2]:5.2f}) m\n'

        thrust_vals = [simX[frame, idx] for idx in thrust_indices]
        thrust_str = ", ".join([f"{val:.1f}" for val in thrust_vals])
        state_str += f'\nThrust: [{thrust_str}] N'

        state_text.set_text(state_str)

        return drone_points + rope_lines + [time_text, state_text]

    # Create animation
    anim = FuncAnimation(fig, update, frames=len(simX), interval=interval, blit=False, repeat=True)

    # Save animation
    if save_animation:
        print("Saving animation to multi_payload_animation.gif...")
        anim.save('multi_payload_animation.gif', writer='pillow', fps=20)
        print("Animation saved!")

    plt.tight_layout()
    plt.show()

    return anim


if __name__ == '__main__':
    # Run simulation
    print("="*70)
    print("  Multi-Quadrotor Rigid-Body Payload NMPC Simulation")
    print("="*70)

    simX, simU, params = nmpc_sim()

    # Visualize results
    print("\nGenerating animation...")
    visualize_multi_payload(simX, simU, params, interval=50, save_animation=True)
