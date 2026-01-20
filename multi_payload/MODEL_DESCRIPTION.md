# Multi-UAV Payload Transport Dynamics Model

## Overview

This implementation creates an acados dynamics model for a system of **3 UAVs cooperatively transporting a rigid body payload** via rigid cables, based on the paper:

**"Geometric Control of Quadrotor UAVs Transporting a Cable-Suspended Rigid Body"**
by Taeyoung Lee, IEEE Transactions on Control Systems Technology, 2018

## System Configuration

- **3 UAVs** (quadrotors) arranged in a triangular configuration
- **1 Rigid body payload** (triangular prism)
- **3 Rigid cables** connecting each UAV to an attachment point on the payload
- Attachment points form an equilateral triangle on the payload

## State Variables (43 total)

### Payload State (13 dimensions)
| Variable | Dimension | Description |
|----------|-----------|-------------|
| `p` | 3 | Position in inertial frame (x, y, z) [m] |
| `v` | 3 | Velocity in inertial frame (vx, vy, vz) [m/s] |
| `qL` | 4 | Attitude quaternion (qw, qx, qy, qz) |
| `omegaL` | 3 | Angular velocity in body frame (ωx, ωy, ωz) [rad/s] |

### Cable State (18 dimensions = 6 per cable × 3 cables)
| Variable | Dimension | Description |
|----------|-----------|-------------|
| `si` | 3 | Cable direction (unit vector from UAV to payload) |
| `ri` | 3 | Cable angular velocity [rad/s] |

### UAV Attitudes (12 dimensions = 4 per UAV × 3 UAVs)
| Variable | Dimension | Description |
|----------|-----------|-------------|
| `qi` | 4 | UAV attitude quaternion (qw, qx, qy, qz) |

## Control Inputs (12 total)

### Per UAV (4 inputs × 3 UAVs)
| Variable | Dimension | Description | Bounds |
|----------|-----------|-------------|--------|
| `Ti` | 1 | Thrust magnitude [N] | [0, 30] |
| `omegai` | 3 | Angular velocity command [rad/s] | [-3, 3] |

## Key Equations from Paper

### 1. Payload Translational Dynamics (Eq. 5)
```
M_q * (p̈ - g*e3) = Σ u_i^∥
```

where:
- `M_q = M*I + Σ m_i * s_i * s_i^T` is the effective mass matrix
- `u_i^∥` is the thrust component parallel to cable direction

### 2. Payload Rotational Dynamics (Eq. 6)
```
J_eff * Ω̇_L + Ω̂_L * J_eff * Ω_L = Σ ρ̂_i * R_L^T * u_i^∥
```

where:
- `J_eff` is the effective inertia tensor
- `ρ_i` are attachment points in payload body frame

### 3. Cable Dynamics (Eq. 7)
```
ω̇_i = (1/l_i) * ŝ_i * a_i - (1/(m_i*l_i)) * ŝ_i * u_i^⊥
```

where:
- `a_i` is acceleration of attachment point
- `u_i^⊥` is thrust component perpendicular to cable

### 4. Kinematic Equations
```
ṗ = v
q̇_L = 0.5 * Ω(ω_L) * q_L
ṡ_i = r_i × s_i
q̇_i = 0.5 * Ω(ω_i) * q_i
```

## Implementation Details

### Quaternion Representation
- Attitude is represented using quaternions `q = [qw, qx, qy, qz]` instead of rotation matrices
- Avoids singularities (gimbal lock) of Euler angles
- Quaternion kinematics: `q̇ = 0.5 * Ω(ω) * q`

### Control Simplification
- **Paper's full model**: UAV moment dynamics `J_i * Ω̇_i + Ω_i × J_i * Ω_i = M_i`
- **Our implementation**: Direct angular velocity control (inner loop assumed)
- Thrust magnitude `T_i` and angular velocity `ω_i` are direct control inputs

### Coordinate Frames
- **Inertial frame**: NED convention (North-East-Down), e3 points downward
- **Payload body frame**: Origin at center of mass, attached to payload
- **UAV body frames**: Origin at UAV center of mass, e3 points downward

### Physical Parameters

```python
# Payload
M = 5.0 kg          # Mass
J_L = diag([0.5, 0.5, 0.8]) kg·m²  # Inertia

# UAVs (identical)
m_i = 1.5 kg        # Mass per UAV
J_i = 0.1 * I       # Inertia per UAV

# Cables
l_i = 1.0 m         # Cable length

# Attachment points (equilateral triangle)
r = 1.0 m           # Center to corner distance
```

## Usage

### Create Model
```python
from multi_payload_model import multi_payload_model

model = multi_payload_model()
# Returns AcadosModel with 43 states and 12 inputs
```

### State Vector Layout
```python
x = [p(3), v(3), qL(4), omegaL(3),
     s1(3), r1(3), s2(3), r2(3), s3(3), r3(3),
     q1(4), q2(4), q3(4)]  # 43 total
```

### Control Vector Layout
```python
u = [T1, omega1(3), T2, omega2(3), T3, omega3(3)]  # 12 total
```

## Differences from Paper

1. **Quaternions vs Rotation Matrices**: We use quaternions for numerical stability
2. **Direct Angular Velocity Control**: Simplified UAV dynamics (inner loop assumed)
3. **No Disturbances**: Model excludes disturbance terms for simplicity
4. **Rigid Cables**: Uses rigid cable assumption (not flexible cables)

## References

- Taeyoung Lee, "Geometric Control of Quadrotor UAVs Transporting a Cable-Suspended Rigid Body," IEEE Transactions on Control Systems Technology, vol. 26, no. 1, pp. 255-264, Jan. 2018.
- Equations referenced by number (Eq. 1, 5, 6, 7, etc.) from the paper
