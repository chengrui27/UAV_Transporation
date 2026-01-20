# PX4 Payload NMPC Controller

基于论文《Nonlinear Backstepping Control of a Quadrotor-Slung Load System》的四旋翼悬挂负载非线性模型预测控制器。

## 🎯 功能特性

- ✅ **100Hz实时控制** - 高频NMPC求解和控制指令发送
- ✅ **完整状态估计** - 读取PX4和Gazebo负载状态，计算16维NMPC状态变量
- ✅ **坐标系转换** - 自动处理ENU(ROS/Gazebo)和NED(PX4)坐标系转换
- ✅ **Offboard模式** - Body Rate控制，2秒延迟确保NMPC准备就绪
- ✅ **acados集成** - 调用高性能NMPC求解器

## 📋 系统要求

- **ROS2**: Humble
- **PX4**: v1.13.3
- **MAVROS**: ROS2版本
- **acados**: 安装并配置
- **Eigen3**: C++线性代数库

## 🔧 安装步骤

### 1. 安装依赖

```bash
# 安装MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# 安装Eigen3
sudo apt install libeigen3-dev

# 安装GeographicLib数据集 (MAVROS需要)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 2. 安装acados

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
```

添加环境变量（在`~/.bashrc`）：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
export ACADOS_SOURCE_DIR=$HOME/acados
```

### 3. 生成NMPC求解器

```bash
cd ~/px4_ws/px4_payload
python3 px4_payload_nmpc.py
```

这将生成C代码到 `~/px4_ws/px4_payload/c_generated_code/`

### 4. 编译功能包

```bash
cd ~/px4_ws
colcon build --packages-select px4_payload_nmpc
source install/setup.bash
```

## 🚀 使用方法

### 完整启动流程

#### 终端1: 启动PX4 SITL Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

等待Gazebo加载完成，应该看到带悬挂负载的iris四旋翼。

#### 终端2: 启动MAVROS

```bash
cd ~/px4_ws
source install/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557
```

#### 终端3: 启动NMPC控制器

```bash
cd ~/px4_ws
source install/setup.bash
ros2 launch px4_payload_nmpc nmpc_controller.launch.py
```

### 控制流程

1. **数据收集阶段** (启动后)
   - 等待接收PX4位置、速度、姿态数据
   - 等待接收Gazebo负载位置、速度数据
   - 所有数据就绪后进入准备阶段

2. **准备阶段** (2秒)
   - 发送心跳消息保持连接
   - NMPC求解器warm-up
   - 倒计时提示

3. **控制阶段**
   - 自动切换到Offboard模式
   - 自动解锁飞机
   - 100Hz频率运行NMPC控制循环
   - 实时统计求解成功率和耗时

### 监控状态

**查看节点日志**:
```bash
ros2 topic echo /rosout | grep nmpc_controller
```

**查看控制指令**:
```bash
ros2 topic echo /mavros/setpoint_raw/attitude
```

**查看MAVROS状态**:
```bash
ros2 topic echo /mavros/state
```

## ⚙️ 配置参数

在launch文件中可以配置以下参数：

```python
ros2 launch px4_payload_nmpc nmpc_controller.launch.py \
  cable_length:=1.0 \
  quad_mass:=1.5 \
  payload_mass:=0.3 \
  control_frequency:=100.0 \
  offboard_delay:=2.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cable_length` | 1.0 | 绳长 (m) |
| `quad_mass` | 1.5 | 四旋翼质量 (kg) |
| `payload_mass` | 0.3 | 负载质量 (kg) |
| `control_frequency` | 100.0 | 控制频率 (Hz) |
| `offboard_delay` | 2.0 | Offboard延迟 (s) |

## 📊 状态变量说明

### NMPC状态向量 (16维)

基于论文公式(8)-(12)：

```
x = [pL, vL, q, ω, quaternion]^T ∈ R^16

其中:
- pL (3D): 负载位置 (NED坐标系)
- vL (3D): 负载速度 (NED坐标系)
- q (3D): 绳子方向单位向量
- ω (3D): 绳子角速度
- quaternion (4D): 四旋翼姿态四元数 (w,x,y,z)
```

### 控制输入 (4维)

```
u = [T, Ωx, Ωy, Ωz]^T ∈ R^4

其中:
- T: 推力 (N)
- Ω: 机体角速度 (rad/s, body frame)
```

## 🔄 坐标系转换

### ENU → NED 转换

```cpp
NED_North = ENU_y
NED_East  = ENU_x
NED_Down  = -ENU_z
```

### 订阅话题坐标系

| 话题 | 坐标系 | 说明 |
|------|--------|------|
| `/mavros/local_position/pose` | ENU | PX4位置(MAVROS转换后) |
| `/mavros/local_position/velocity_local` | ENU | PX4速度 |
| `/payload/odom` | ENU | Gazebo负载状态 |

### 发布话题坐标系

| 话题 | 坐标系 | 说明 |
|------|--------|------|
| `/mavros/setpoint_raw/attitude` | Body | Body Rate控制 |

## 🐛 故障排除

### 问题1: 编译错误 - 找不到acados

**错误信息**:
```
Could not find acados
```

**解决方案**:
1. 确认acados已安装: `ls ~/acados/lib/libacados.so`
2. 修改CMakeLists.txt中的`ACADOS_INSTALL_DIR`路径
3. 添加环境变量: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib`

### 问题2: 运行时错误 - 找不到生成的求解器

**错误信息**:
```
Acados generated solver not found!
```

**解决方案**:
1. 运行Python脚本生成求解器:
   ```bash
   cd ~/px4_ws/px4_payload
   python3 px4_payload_nmpc.py
   ```
2. 检查生成的文件:
   ```bash
   ls ~/px4_ws/px4_payload/c_generated_code/
   ```
3. 重新编译功能包

### 问题3: Offboard模式切换失败

**现象**: 节点启动但无法进入Offboard模式

**解决方案**:
1. 确认MAVROS已连接: `ros2 topic echo /mavros/state`
2. 检查PX4参数:
   ```bash
   # 在PX4控制台
   param set COM_RCL_EXCEPT 4  # 允许无遥控器
   param set NAV_RCL_ACT 0     # 失去遥控器时不做任何事
   ```
3. 确认心跳消息发送正常

### 问题4: NMPC求解失败

**现象**: 日志显示 "NMPC solver failed"

**解决方案**:
1. 检查初始状态是否合理
2. 调整NMPC权重矩阵 (修改`px4_payload_nmpc.py`)
3. 增加求解器迭代次数
4. 检查约束条件是否过严

## 📖 相关论文

```bibtex
@article{yu2019nonlinear,
  title={Nonlinear backstepping control of a quadrotor-slung load system},
  author={Yu, Gan and Cabecinhas, David and Cunha, Rita and Silvestre, Carlos},
  journal={IEEE/ASME Transactions on Mechatronics},
  volume={24},
  number={5},
  pages={2304--2315},
  year={2019},
  publisher={IEEE}
}
```

## 📝 许可证

MIT License

## 👥 贡献者

PX4 Payload NMPC Team
