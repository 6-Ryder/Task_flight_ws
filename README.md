# 四旋翼无人机圆筒侦察任务仿真系统

> PX4 + ROS2 + Gazebo  —  micro-XRCE-DDS 直连架构

---

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 场地布局](#2-场地布局)
- [3. 文件结构](#3-文件结构)
- [4. 通信架构](#4-通信架构)
- [5. 坐标系](#5-坐标系)
- [6. 依赖与环境](#6-依赖与环境)
- [7. 构建与启动](#7-构建与启动)
- [8. 任务流程（状态机）](#8-任务流程状态机)
- [9. 关键代码解析](#9-关键代码解析)
- [10. 参数配置](#10-参数配置)
- [11. 平台适配记录](#11-平台适配记录)
- [12. 故障排查](#12-故障排查)

---

## 1. 项目概述

本系统在 Gazebo 仿真环境中，控制一架 X500 四旋翼无人机执行**圆筒侦察任务**：

- 从 H 起降点起飞，巡航高度 5m
- 飞至投放区，在 2 个不同尺寸的圆筒上方降高至 2m 悬停 5s（模拟投放/识别）
- 飞至侦察区，在 5m 高度依次穿越 5 个圆筒
- 返回 H 点精准着陆

控制器通过 **micro-XRCE-DDS** 直连 PX4 飞控，不经过 MAVROS 中间层。

---

## 2. 场地布局

```
  场地总尺寸: 70m (长, x轴) × 8m (宽, y轴)

  x=0                   10              40      45              65      70
  ├───────┼──────────────┼───────────────┼───────┼───────────────┼───────┤
  │ 准备区 │   起降区      │    投放区      │ 间隔区 │    侦察区      │       │
  │ (绿)  │   (深灰)     │  (3筒 15/20/25)│ (原色) │  (5筒 Ø20)   │       │
  │ 0~7m  │   7~40m      │   40~45m       │45~65m │   65~70m      │       │
  │       │   H@x=10,y=4 │               │       │  橙色底色     │       │
  └───────┴──────────────┴───────────────┴───────┴───────────────┴───────┘
                                         起飞线@x=12 (虚线)
```

| 区域 | x 范围 | 说明 |
|---|---|---|
| 准备区 | 0 ~ 7m | 绿色地面 |
| 起降区 | 7 ~ 40m | H 起降点 (x=10, y=4, Ø80cm)，起飞线 (x=12) |
| 投放区 | 40 ~ 45m | 3 个圆筒 (Ø15/20/25cm, h=30cm)，A/B 有效区 |
| 间隔区 | 45 ~ 65m | 地面原色 |
| 侦察区 | 65 ~ 70m | 5 个圆筒 (Ø20cm, h=15cm)，含危险品标识板 |

---

## 3. 文件结构

```
Task_flight_ws/
│
├── README.md                          # 本文件
├── drone_field.kne                    # 场地 SDF 源文件 (XML)
├── drone_field_meshes/                # 场地 3D 模型与纹理
│   ├── cyl_r0075_h030.stl             # 圆筒 Ø7.5cm × h30cm (投放区1号)
│   ├── cyl_r0100_h030.stl             # 圆筒 Ø10cm × h30cm (投放区2号 + 侦察区)
│   ├── cyl_r0125_h030.stl             # 圆筒 Ø12.5cm × h30cm (投放区3号, 未使用)
│   ├── cyl_r0100_h015.stl             # 圆筒 Ø10cm × h15cm (侦察区5筒)
│   └── textures/
│       ├── hazard_flammable.png       # 易燃标识 (橙色)
│       ├── hazard_health.png          # 健康危害标识 (蓝色)
│       └── hazard_danger.png          # 危险标识 (红色)
│
├── PX4-Autopilot -> /path/to/PX4-Autopilot   # PX4 软链接 (setup_project.sh 自动创建)
│
└── src/
    ├── px4_msgs/                      # PX4 uORB → ROS2 消息定义
    │   ├── CMakeLists.txt             # CMake 构建 (rosidl_generate_interfaces)
    │   ├── package.xml
    │   ├── msg/                       # 150+ 个 .msg 文件
    │   └── srv/
    │       └── VehicleCommand.srv     # VehicleCommand / VehicleCommandAck
    │
    └── px4_control_dds/               # 任务控制器包
        ├── setup.py                   # Python 包入口 (entry_points → console_script)
        ├── setup.cfg
        ├── package.xml                # 依赖: rclpy, px4_msgs
        ├── resource/
        │   └── px4_control_dds        # ament 索引标记
        ├── config/
        │   └── mission_params.yaml    # 运行时参数
        ├── launch/
        │   └── mission_control.launch.py   # ROS2 Launch 文件
        ├── scripts/
        │   └── start_mission_dds.sh   # 一键启动脚本 (主入口)
        └── px4_control_dds/
            ├── __init__.py
            └── mission_control_node.py # 任务控制器主逻辑 (~1300 行)
```

### 核心文件说明

| 文件 | 用途 |
|---|---|
| `start_mission_dds.sh` | **主入口脚本** — 按顺序启动 MicroXRCEAgent → PX4+Gazebo → QGC → 控制器 |
| `mission_control_node.py` | **任务控制器** — 19 状态状态机，20Hz 控制循环，DDS 直连 PX4 |
| `mission_control.launch.py` | ROS2 Launch 描述 — 加载 YAML 参数，启动控制器节点 |
| `mission_params.yaml` | 可调参数 — 高度、容差、悬停时长、超时等 |
| `drone_field.sdf` | Gazebo 世界文件 — 70m×8m 场地，圆筒、标线、危险品标识板 |

---

## 4. 通信架构

### 4.1 整体架构

```
┌──────────────────────────────────────────────────────────────────┐
│ PX4 飞控 (SITL)                                                  │
│   uORB 话题                        uxrce_dds_client 模块          │
│   ┌──────────┐                    ┌──────────────────┐           │
│   │ odometry │──→ 发布 ──→       │                  │           │
│   │ status   │──→ 发布 ──→       │  XRCE/UDP Client │           │
│   │ command  │←── 订阅 ←──       │  (port 8888)     │           │
│   │ setpoint │←── 订阅 ←──       │                  │           │
│   └──────────┘                    └────────┬─────────┘           │
└────────────────────────────────────────────┼──────────────────────┘
                                             │ XRCE 协议 over UDP
┌────────────────────────────────────────────┼──────────────────────┐
│ MicroXRCEAgent (独立进程)                  │                      │
│   XRCE/UDP Server (监听 :8888)             │                      │
│   → 为每个 uORB 话题创建 DDS Topic/Reader/Writer                  │
└────────────────────────────────────────────┼──────────────────────┘
                                             │ DDS/RTPS
┌────────────────────────────────────────────┼──────────────────────┐
│ ROS2 + 本控制器                            │                      │
│   订阅: /fmu/out/vehicle_odometry          │                      │
│   订阅: /fmu/out/vehicle_status_v4         │                      │
│   发布: /fmu/in/trajectory_setpoint        │                      │
│   发布: /fmu/in/offboard_control_mode      │                      │
│   发布: /fmu/in/vehicle_command            │                      │
└────────────────────────────────────────────┴──────────────────────┘
```

### 4.2 DDS vs MAVROS

| 特性 | MAVROS 方案 | DDS 方案 (本项目) |
|---|---|---|
| 中间层 | MAVROS 进程 (MAVLink ↔ ROS2) | MicroXRCEAgent (XRCE ↔ DDS) |
| 额外依赖 | mavros, mavros_msgs | 仅需 px4_msgs |
| 延迟 | 较高 (协议转换) | 较低 (DDS 直连) |
| 话题命名 | `/mavros/...` | `/fmu/out/...` `/fmu/in/...` |
| 注册机制 | 无 | 需要外部组件注册 (4 步握手) |

### 4.3 QoS 配置

PX4 的 `uxrce_dds_client` 使用 **BEST_EFFORT + TRANSIENT_LOCAL** QoS 发布传感器数据：

- `BEST_EFFORT`: 不重传丢失的数据包，适合高频传感器数据
- `TRANSIENT_LOCAL`: 新订阅者加入时能收到最新一帧缓存数据
- `depth=10`: 缓存最近 10 条消息

> ⚠️ **关键**: 订阅方必须使用相同的 QoS 配置，否则 DDS 中间件会静默丢弃所有消息。

### 4.4 外部组件注册协议

PX4 通过 DDS 拒绝未注册组件的 VehicleCommand 指令。注册流程：

```
1. 控制器 → PX4: RegisterExtComponentRequest
   (register_arming_check=True, px4_ros2_api_version=LATEST)

2. PX4 → 控制器: RegisterExtComponentReply
   (success=True, arming_check_id=N)

3. PX4 → 控制器: ArmingCheckRequest (解锁前广播)
   (valid_registrations_mask 指示哪些组件需要响应)

4. 控制器 → PX4: ArmingCheckReply
   (can_arm_and_run=True, registration_id=N)
```

---

## 5. 坐标系

这是**最容易出错的部分**，务必理解。

### 5.1 三种坐标系

| 坐标系 | x | y | z | 用途 |
|---|---|---|---|---|
| **Gazebo 世界** | 东 (场地长边 70m) | 北 (场地宽边 8m) | 上 | Gazebo 仿真 |
| **PX4 NED** | 北 = Gazebo +y | 东 = Gazebo +x | 下 = Gazebo -z | PX4 飞控内部 |
| **PX4 NED Local** | 北 (相对 H 点) | 东 (相对 H 点) | 下 (负值=向上) | 控制器航点 |

### 5.2 转换公式

H 起降点 Gazebo 世界坐标: `(10, 4, 0)`

```
NED_x (north) = Gazebo_y - 4
NED_y (east)  = Gazebo_x - 10
NED_z (down)  = -Gazebo_z
```

### 5.3 控制器航点坐标 (PX4 NED Local, 原点=H)

```python
TAKEOFF:       ( 0.0,  0.0, -5.0)   # H 点正上方 5m
TRANSIT_DROP:  ( 0.0, 32.5, -5.0)   # 投放区中心 (Gazebo x=42.5, y=4)
HOVER_C1:      (-1.0, 31.5, -2.0)   # 筒1 Ø15cm, 降高 2m
HOVER_C2:      ( 1.5, 33.0, -2.0)   # 筒2 Ø20cm, 降高 2m
TRANSIT_RECON: ( 0.0, 57.5, -5.0)   # 侦察区中心 (Gazebo x=67.5, y=4)
RECON_C1:      (-2.0, 56.0, -5.0)   # 侦察筒1
RECON_C2:      ( 1.5, 57.5, -5.0)   # 侦察筒2
RECON_C3:      ( 3.0, 58.5, -5.0)   # 侦察筒3
RECON_C4:      (-0.5, 56.5, -5.0)   # 侦察筒4
RECON_C5:      (-2.5, 59.0, -5.0)   # 侦察筒5
RETURN:        ( 0.0,  0.0, -5.0)   # 返回 H 点 5m
```

### 5.4 偏航角

- PX4 约定: yaw=0 → 朝北 (Gazebo +y, 场地窄边)
- 本控制器: yaw=π/2 → **朝东** (Gazebo +x, 场地长边/飞行方向)

---

## 6. 依赖与环境

### 6.1 系统要求

| 组件 | 版本 | 安装方式 |
|---|---|---|
| Ubuntu | 22.04 (Jammy) 或 24.04 (Noble) | — |
| ROS2 | Humble (22.04) / Jazzy (24.04) | `apt install ros-<distro>-desktop` |
| Gazebo | Harmonic (gz-harmonic) | `apt install gz-harmonic` |
| PX4-Autopilot | v1.15+ | `git clone` + `make px4_sitl` |
| colcon | 通用构建工具 | `apt install python3-colcon-common-extensions` |
| MicroXRCEAgent | v3.0+ | 从源码编译 (见下方) |

### 6.2 MicroXRCEAgent 安装

```bash
# 1. 安装构建依赖
sudo apt-get install -y rapidjson-dev libasio-dev libfastcdr-dev libfastrtps-dev libspdlog-dev

# 2. 克隆并编译
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DUAGENT_SUPERBUILD=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 6.3 Python 依赖

```
rclpy
px4_msgs        # 本项目的消息包
numpy
```

---

## 7. 构建与启动

### 7.1 首次构建

```bash
cd ~/Task_flight_ws

# 一键初始化 (修复所有路径 + 编译)
./setup_project.sh

# 或手动编译
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs px4_control_dds
```

### 7.2 一键启动

```bash
cd ~/Task_flight_ws
./src/px4_control_dds/scripts/start_mission_dds.sh
```

脚本会自动完成以下步骤：

| Step | 操作 | 说明 |
|---|---|---|
| 0 | 清理旧进程 | 杀掉残留的 Agent/Gazebo/PX4 |
| 1 | 启动 MicroXRCEAgent | UDP 8888 端口，后台运行 |
| 2 | 启动 PX4 SITL + Gazebo | 加载 `drone_field` 世界，x500 模型置于 H 点 |
| 3 | 启动 QGroundControl (可选) | 可视化地面站 |
| 4 | 启动任务控制器 | 当前终端实时日志，Ctrl+C 停止 |

### 7.3 手动启动（调试用）

```bash
# 终端 1: Agent
MicroXRCEAgent udp4 -p 8888

# 终端 2: PX4 + Gazebo
cd ~/PX4-Autopilot/build/px4_sitl_default
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=drone_field
export PX4_GZ_MODEL_POSE="10,4,0.3,0,0,0"
./bin/px4 -i 0

# 终端 3: 控制器
source /opt/ros/humble/setup.bash
source ~/Task_flight_ws/install/setup.bash
ros2 launch px4_control_dds mission_control.launch.py
```

### 7.4 停止

按 `Ctrl+C` 停止控制器，脚本的 `trap EXIT` 会自动清理 Agent、Gazebo 和 PX4 进程。也可以手动清理：

```bash
pkill -f "MicroXRCEAgent"
pkill -f "gz sim"
pkill -f "bin/px4"
```

---

## 8. 任务流程（状态机）

控制器使用 **19 状态状态机**，每个控制周期 (20Hz) 执行当前状态的处理函数。

```
Phase 0: INIT              等待里程计数据稳定 (≥50条)
Phase 1: REGISTER          等待 PX4 外部组件注册确认
Phase 2: STREAM_SETPOINTS  预热 setpoint 流 (发送100个地面定位点)
Phase 3: ARM               切换 OFFBOARD + 解锁 (force arm)
Phase 4: TAKEOFF           H 点起飞 → 巡航高度 5m
Phase 5: TRANSIT_DROP      飞向投放区中心 (32.5m 东)
Phase 6: HOVER_C1          筒1 (Ø15cm) 降高至 2m 悬停 5s
Phase 7: ASCEND_C1         筒1 上方回升至 5m
Phase 8: HOVER_C2          筒2 (Ø20cm) 降高至 2m 悬停 5s
Phase 9: ASCEND_C2         筒2 上方回升至 5m
Phase 10: TRANSIT_RECON    飞向侦察区中心 (57.5m 东)
Phase 11: RECON_C1         侦察筒1 穿越 (5m)
Phase 12: RECON_C2         侦察筒2 穿越 (5m)
Phase 13: RECON_C3         侦察筒3 穿越 (5m)
Phase 14: RECON_C4         侦察筒4 穿越 (5m)
Phase 15: RECON_C5         侦察筒5 穿越 (5m)
Phase 16: RETURN           返回 H 位置 (5m 高度)
Phase 17: LAND             垂直下降 → 停止 Offboard → 上锁
Phase 18: COMPLETE         任务完成
```

### 状态转移

```mermaid
INIT → REGISTER → STREAM_SETPOINTS → ARM → TAKEOFF
  → TRANSIT_DROP → HOVER_C1 → ASCEND_C1 → HOVER_C2 → ASCEND_C2
  → TRANSIT_RECON → RECON_C1→C5
  → RETURN → LAND → COMPLETE
```

### 时间线参考

| 阶段 | 约耗时 |
|---|---|
| 初始化 (0→3) | ~15s |
| 起飞 | ~8s |
| 投放区 (2筒悬停) | ~14s |
| 侦察区 (5筒穿越) | ~12s |
| 返航+着陆 | ~12s |
| **总计** | **~60s** |

---

## 9. 关键代码解析

### 9.1 `start_mission_dds.sh` — 启动脚本

```bash
# 自动检测项目根目录
PROJ="$(cd "$(dirname "$0")/../../.." && pwd)"

# 自动编译 (首次运行)
if [ ! -f "$PROJ/install/setup.bash" ]; then
    colcon build --packages-select px4_msgs px4_control_dds
fi

# PX4 环境变量
export PX4_SIM_MODEL="gz_x500"       # 使用 X500 四旋翼模型
export PX4_GZ_WORLD="drone_field"    # 加载 drone_field 世界
export PX4_GZ_MODEL_POSE="10,4,0.3,0,0,0"  # 放置于 H 点 (x,y,z,roll,pitch,yaw)

# 健康检查: 等待 Gazebo world 就绪
if gz topic -l 2>/dev/null | grep -q "/world/drone_field/clock"; then
    log "Gazebo world ready"
fi

# 健康检查: 等待 DDS 话题出现
if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
    log "DDS topics ready"
fi
```

### 9.2 `mission_control_node.py` — 控制器核心

**类结构**: `MissionController(Node)` — 继承 `rclpy.node.Node`

**定时器**:

| 频率 | 方法 | 功能 |
|---|---|---|
| 20Hz | `_control_loop()` | 主状态机驱动 |
| 10Hz | `_heartbeat()` | 维持 Offboard 模式 (发布 ControlMode + Setpoint) |
| 1Hz | `_registration_timer()` | 重试组件注册直到成功 |

**DDS 话题**:

| 方向 | 话题 | 消息类型 | 频率 |
|---|---|---|---|
| PX4→ROS2 | `/fmu/out/vehicle_odometry` | `VehicleOdometry` | ~30-50Hz |
| PX4→ROS2 | `/fmu/out/vehicle_status_v4` | `VehicleStatus` | ~10Hz |
| PX4→ROS2 | `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` | ~10Hz |
| PX4→ROS2 | `/fmu/out/register_ext_component_reply_v1` | `RegisterExtComponentReply` | 按需 |
| PX4→ROS2 | `/fmu/out/arming_check_request_v1` | `ArmingCheckRequest` | 解锁前 |
| ROS2→PX4 | `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | 10Hz |
| ROS2→PX4 | `/fmu/in/offboard_control_mode` | `OffboardControlMode` | 10Hz |
| ROS2→PX4 | `/fmu/in/vehicle_command` | `VehicleCommand` | 按需 |
| ROS2→PX4 | `/fmu/in/register_ext_component_request_v2` | `RegisterExtComponentRequest` | 1Hz |
| ROS2→PX4 | `/fmu/in/arming_check_reply_v1` | `ArmingCheckReply` | 按需 |

**着陆策略** (关键设计):

1. Setpoint 锁定 H 点 (0, 0, 0)，无人机 Offboard 下降
2. 当 z > -0.2 (离地 < 20cm): 停止 `OffboardControlMode.position` → PX4 检测到失去位置控制 → 自动退出 Offboard (~0.5s)
3. 退出 Offboard 后持续发送 `force_disarm` (param2=21196, bypass 检查) → 螺旋桨停止

> 为什么这样设计: PX4 不允许在 Offboard 模式下上锁。通过停止位置控制声明让 PX4 自行退出 Offboard，比发送 VEHICLE_CMD_DO_SET_MODE 更可靠。

**VehicleCommand 防抖**: 指令间隔 ≥2-3s，避免 PX4 commander 模块过载。

### 9.3 `mission_control.launch.py` — Launch 文件

```python
def generate_launch_description():
    pkg_share = get_package_share_directory('px4_control_dds')
    param_file = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    return LaunchDescription([
        Node(
            package='px4_control_dds',
            executable='mission_control_dds_node',  # setup.py entry_points 注册
            name='mission_controller_dds',
            output='screen',
            emulate_tty=True,
            parameters=[param_file],
        )
    ])
```

---

## 10. 参数配置

所有参数在 `config/mission_params.yaml` 中定义，可通过命令行覆盖：

```yaml
/mission_controller_dds:
  ros__parameters:
    takeoff_height: 5.0         # 巡航飞行高度 (m)
    waypoint_tolerance: 0.5     # 航点到达容差 (m)
    takeoff_tolerance: 0.3      # 起飞到达容差 (m)
    hover_duration: 5.0         # 投放区悬停时长 (s)
    control_frequency: 20.0     # 控制循环频率 (Hz), 必须 >2Hz
    max_error_count: 5          # 最大错误次数, 超过则中止任务
    waypoint_timeout: 120.0     # 单航点超时 (s)
    land_altitude: 0.15         # 着陆确认高度 (m)
```

命令行覆盖示例:

```bash
ros2 launch px4_control_dds mission_control.launch.py \
  --ros-args -p takeoff_height:=10.0 -p hover_duration:=10.0
```

---

## 11. 平台适配记录

本项目原为 **Ubuntu 24.04 + ROS2 Jazzy** 编写，已适配至 **Ubuntu 22.04 + ROS2 Humble**。

### 已修改项

| 文件 | 修改 | 原因 |
|---|---|---|
| `start_mission_dds.sh` L25 | `/opt/ros/jazzy` → `/opt/ros/humble` | ROS2 发行版适配 |
| `drone_field.sdf` | `/home/llc/drone_field_meshes/` → `/home/ryder/Task_flight_ws/drone_field_meshes/` | 用户路径适配 |
| `drone_field.sdf` L920-928 | 删除静态 `<include>model://x500</include>` 模型 | 避免与 PX4 生成的飞行器重叠导致返航碰撞 |
| `mission_control_node.py` L498 | `self._yaw = 0.0` → `self._yaw = math.pi / 2` | 机头朝东对齐飞行方向 (North→East) |
| — | 编译安装 MicroXRCEAgent 至 `/usr/local/bin` | 系统缺失此依赖 |

### 其他平台适配时需检查

> **推荐**: 使用自动化脚本 `./setup_project.sh` 一键完成以下所有适配。
> 手动检查清单仅作为备用参考。

```bash
# 一键适配 (推荐)
cd ~/Task_flight_ws
./setup_project.sh

# 如果 PX4-Autopilot 不在标准位置
./setup_project.sh --px4-dir /custom/path/PX4-Autopilot
```

`setup_project.sh` 自动完成:
1. 检测并修复 PX4-Autopilot 软链接
2. 修复 `drone_field.sdf` 中所有 `file://` 路径
3. 部署 SDF 到 PX4 worlds 目录
4. 检测 ROS2 发行版并修复启动脚本
5. 编译工作空间
6. 生成 VS Code 配置
7. 检查 MicroXRCEAgent 是否已安装

**手动检查清单** (不使用自动化脚本时):

1. ROS2 安装路径: `/opt/ros/<distro>/setup.bash`
2. PX4-Autopilot 软链接: `ln -s /path/to/PX4-Autopilot ~/Task_flight_ws/PX4-Autopilot`
3. `drone_field_meshes/` 路径: 需更新 SDF 中所有 `file://` URI
4. `drone_field.sdf` 需复制到 PX4: `cp drone_field.sdf $PX4_DIR/Tools/simulation/gz/worlds/`
5. QGroundControl AppImage: 可选，放 `$HOME` 下或确保 `qgroundcontrol` 在 PATH 中
6. 密码: 如果 sudo 需要密码，`export SUDO_PASSWORD="your_password"`

---

## 12. 故障排查

### 12.1 多进程冲突

**症状**: 无人机起飞后无法爬升，高度卡在 ~0.5m

**原因**: 多个 PX4 或 Gazebo 实例同时运行，DDS 话题冲突

**解决**:

```bash
sudo pkill -9 -f "px4"
sudo pkill -9 -f "gz"
sudo pkill -9 -f "MicroXRCE"
```

确保无残留进程后重新启动。

### 12.2 DDS 话题不可见

**症状**: 控制器日志显示 "等待 PX4 odometry 数据..." 超时

**排查**:

```bash
# 检查 Agent 是否在监听
ss -tuln | grep 8888

# 检查 DDS 话题
source /opt/ros/humble/setup.bash
ros2 topic list | grep "/fmu/"
```

### 12.3 注册失败

**症状**: "注册超时 (30s)!"

**解决**: 确认 MicroXRCEAgent 先于 PX4 启动，PX4 的 uxrce_dds_client 模块已启用 (默认启用)。

### 12.4 解锁失败

**症状**: ARM 阶段反复重试，迟迟不解锁

**可能原因**:
1. GCS 连接检查 — PX4 默认要求 GCS 心跳，使用 `force_arm` (param2=21196) 绕过
2. EKF 未收敛 — 等待里程计数据稳定 (≥50 条)
3. 组件未注册 — 确认 `arming_check_id` 已获取

### 12.5 着陆时卡在 Offboard 模式

**症状**: 无人机在 H 点上方悬停，不下降

**解决**: 控制器在 z > -0.2m 时停止 OffboardControlMode.position，PX4 检测到后自动退出。若此机制失效，可在 LAND 状态中增加 `VEHICLE_CMD_DO_SET_MODE` 到 POSCTL 的备用逻辑。

---

## 日志

每次运行在 `/tmp/px4_dds_mission_<timestamp>/` 下生成：

| 文件 | 内容 |
|---|---|
| `agent.log` | MicroXRCEAgent 输出 |
| `px4.log` | PX4 SITL 输出 |
| `mission.log` | 控制器实时日志 |
| `qgc.log` | QGroundControl 输出 |
| `*.pid` | 各进程 PID |

---

## 许可

MIT License
