# 工作记录（NMPC + ESDF 避障）

> 用途：给下次接手的 Codex / AI 助手快速恢复上下文。

## 当前整体结构
- `plan_env`：构建 ESDF 地图，写入共享内存（距离 + 梯度），发布 `grid_map/esdf` 与 `grid_map/esdf_metadata`。
- `px4`：`px4_nmpc.py` 使用 acados 生成 NMPC 求解器（quadrotor 模型 + ESDF 非线性约束）。
- `px4_nmpc`：`nmpc_controller.cpp` 读取 ESDF 共享内存，通过 acados 求解控制输入，Body Rate Offboard 控制 PX4。

## ESDF / 地图相关修改（已完成）
- **只关心雷达点云建图，不使用深度图像做占据更新。**
- `GridMap::cloudCallback`：
  - 不再调用 `resetBuffer()`，当前帧扫描到的点云在 `occupancy_buffer_inflate_` 上“累积”，不清空为 free。
  - 仍然在 `local_update_range` 内对雷达点进行栅格膨胀，并写入 `occupancy_buffer_inflate_`。
  - `md_.local_bound_min_/max_` 每帧设为 `camera_pos ± local_update_range`（固定尺寸局部盒），ESDF 始终在这块上重算。
- `updateESDF3d()`：
  - 使用累积的 `occupancy_buffer_inflate_`（局部裁剪到 `local_bound`）作为障碍源，计算 3D ESDF（正/负距离 + gradient），写入共享内存。

## acados / NMPC 控制相关修改（已完成）
- `px4/px4_nmpc.py`：
  - 定义 ESDF 约束：使用一阶泰勒展开的距离 `d_approx`，约束 `d_approx - safety_distance >= 0`。
  - 将该约束设置为**软约束**：
    - 使用 `idxsh/lsh/ush` 配置 slack；
    - 在 `Zl/Zu/zl/zu` 中对 slack 加大权重（约 `1e4` 量级）。
  - ESDF 运行时参数 `p = [d_ref, grad_x, grad_y, grad_z, x_ref, y_ref, z_ref]`，由 C++ 端更新。
- `src/px4_nmpc/src/nmpc_controller.cpp`：
  - 使用 `ESDFMapReader` 从共享内存读取距离和梯度。
  - **内层多次求解 + 碰撞检查**：
    - 每个控制周期内最多进行 3 次 `px4_model_acados_solve`。
    - 每次求解后扫描整条预测轨迹，如果任何点的 ESDF 距离 < `safety_distance_check`（~0.4m），认为“预测碰撞”：
      - 使用当前预测轨迹重新线性化 ESDF（调用 `getDistanceAndGradient`，更新所有阶段参数 `p`），再解一次；
      - 若 3 次后仍碰撞，发警告但使用最后一次解。
  - 新目标点回调 `goalPoseCallback`：
    - 重置 `first_solve_done_ = false`；
    - 调用 `resetAcadosStateForNewGoal()`：
      - `px4_model_acados_reset` 重置 solver 内部状态；
      - 所有阶段参数初始化为“远离障碍”的默认值：`dist=10, grad=0, x_ref=当前机体位置`。

## 尚未实现但计划中的改动（Trust Region）
- 想引入**“位置信任域”代价**，只对 `x,y,z` 三轴位置加惩罚，限制轨迹每次修正幅度：
  - 在 `px4_nmpc.py` 中扩展 OCP 的输出维度 `ny`，在 LINEAR_LS 代价里增加一块用于 `(p - p_trust)` 的二次项。
  - 新的 `y` 结构（示意）：`y = [x(10); u(4); p(3)]`，`ny = 17`。
  - 权重矩阵 `W = block_diag(Q_track(10x10), R(4x4), Q_trust_pos(3x3))`，其中 `Q_trust_pos` 对 `px,py,pz` 施加较小但非零的信任域权重。
  - `Vx/Vu` 设置：
    - 行 0..9：`Vx[i,i] = 1`（状态跟踪，保持现有逻辑）。
    - 行 10..13：`Vu[10+i, i] = 1`（控制输入跟踪）。
    - 行 14..16：`Vx[14,0]=1, Vx[15,1]=1, Vx[16,2]=1`（只对位置分量添加 trust term）。
  - `yref` 每阶段：`[x_goal(10); u_hover(4); p_trust(3)]`，其中 `p_trust` 在 C++ 中设置：
    - 初期可简单用当前测量的位置 `x0[0:3]`；
    - 理想方案：用上一轮预测轨迹的位置 `x_pred_last[i][0:3]` 作为每个阶段的 trust 中心。
- **C++ 端的配合（尚未改）**：
  - 把 `solveNMPC()` 中的 `yref` 长度从 14 调整为 17，并在设置 yref 时填上最后 3 维的位置信任中心。

## 使用/调试建议（给未来自己）
- 若后续修改 `px4_nmpc.py`（尤其是 cost 结构）：
  - 记得重新运行 `python3 px4_nmpc.py` 生成新的 acados 代码与 `quadrotor_ocp.json`；
  - 再重编 `px4_nmpc` 包：`colcon build --packages-select px4_nmpc`。
- 调试优先顺序：
  1. 确认 ESDF 地图合理（`grid_map/esdf` 可视化，看圆柱是否完整、下方是否被误判 free）。
  2. 查看 `nmpc_controller` 日志中内层迭代与碰撞检测是否按预期触发。
  3. 在实现 trust region 后，重点观察轨迹是否从“突然大幅绕路”变成“沿障碍边缘小步绕行”。  

读取本文件可以快速恢复当前设计状态，然后再对应查看：
- 地图相关：`src/plan_env/src/grid_map.cpp`
- NMPC 控制：`src/px4_nmpc/src/nmpc_controller.cpp`
- OCP 建模：`px4/px4_nmpc.py`
