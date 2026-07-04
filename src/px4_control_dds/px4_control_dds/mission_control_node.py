#!/usr/bin/env python3
"""
PX4 + ROS2 + Gazebo — 四旋翼无人机圆筒侦察任务控制器 (micro-XRCE-DDS 版)
======================================================================

═══════════════════════════════════════════════════════════════════════════
                    通 信 架 构 对 比
═══════════════════════════════════════════════════════════════════════════

  MAVROS 版 (drone_mission_ws):
    PX4 ←──MAVLink UDP──→ MAVROS 进程 ←──ROS2 Topic/Service──→ 控制器
    中间多一层 MAVROS 桥接, 需要额外安装 mavros 和 mavros_msgs

  DDS 版 (本项目 drone_mission_dds_ws):
    PX4 ←──XRCE/UDP :8888──→ MicroXRCEAgent ←──DDS/RTPS──→ 控制器
    PX4 内部 uxrce_dds_client 直接将 uORB 话题映射为 DDS, 无需 MAVROS

═══════════════════════════════════════════════════════════════════════════
                micro-XRCE-DDS 数 据 流 详 解
═══════════════════════════════════════════════════════════════════════════

  ┌─────────────────────────────────────────────────────────────┐
  │ PX4 飞控 (SITL)                                             │
  │   uORB 话题                     uxrce_dds_client 模块       │
  │   ┌──────────┐                 ┌──────────────────┐        │
  │   │ odometry │──→ 发布 ──→    │                  │        │
  │   │ status   │──→ 发布 ──→    │  XRCE/UDP Client │        │
  │   │ command  │←── 订阅 ←──    │  (port 8888)     │        │
  │   │ setpoint │←── 订阅 ←──    │                  │        │
  │   └──────────┘                 └────────┬─────────┘        │
  └─────────────────────────────────────────┼───────────────────┘
                                            │ XRCE 协议 over UDP
                                            │ (eProsima Micro XRCE-DDS)
  ┌─────────────────────────────────────────┼───────────────────┐
  │ MicroXRCEAgent (独立进程)               │                   │
  │   ┌─────────────────────────────────────┴─────────┐        │
  │   │  XRCE/UDP Server (监听 :8888)                 │        │
  │   │  接收 PX4 的 XRCE 流 → 创建 DDS Participant   │        │
  │   │  为每个 uORB 话题创建 DDS Topic/Reader/Writer  │        │
  │   └───────────────────┬───────────────────────────┘        │
  └───────────────────────┼────────────────────────────────────┘
                          │ DDS/RTPS (Real-Time Publish-Subscribe)
                          │ 基于 UDP Multicast/Unicast
  ┌───────────────────────┼────────────────────────────────────┐
  │ ROS2 + 本控制器       │                                    │
  │   ┌───────────────────┴──────────────────────────┐        │
  │   │  ROS2 Middleware (rmw_fastrtps / rmw_cyclonedds)│      │
  │   │  自动发现 Agent 创建的 DDS 话题               │        │
  │   │  话题命名: /fmu/out/* (PX4→ROS2)             │        │
  │   │            /fmu/in/*  (ROS2→PX4)             │        │
  │   └───────────────────┬──────────────────────────┘        │
  │                       │ ROS2 Topic 订阅/发布               │
  │   ┌───────────────────┴──────────────────────────┐        │
  │   │  本控制器 (mission_control_node.py)          │        │
  │   │  - 订阅 /fmu/out/vehicle_odometry (位置)     │        │
  │   │  - 订阅 /fmu/out/vehicle_status_v4 (状态)    │        │
  │   │  - 发布 /fmu/in/trajectory_setpoint (航点)   │        │
  │   │  - 发布 /fmu/in/vehicle_command (指令)       │        │
  │   └──────────────────────────────────────────────┘        │
  └────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════════
                    QoS 配 置 说 明
═══════════════════════════════════════════════════════════════════════════

  reliability = BEST_EFFORT:
    传感器数据以高频率发布, 丢失个别样本不影响控制.
    PX4 uxrce_dds_client 以 BEST_EFFORT 发布, 订阅方必须匹配否则收不到数据.
    (ROS2 默认是 RELIABLE, 不匹配会导致静默丢弃所有消息!)

  durability = TRANSIENT_LOCAL:
    PX4 延迟加入 DDS 网络时 (如控制器先于 PX4 启动), 也能收到最新一帧数据.
    避免因启动顺序导致的初始数据缺失.

  depth = 10:
    队列深度, 缓存最近 10 条消息. BEST_EFFORT 下溢出时丢弃旧消息.

═══════════════════════════════════════════════════════════════════════════
                    DDS 话 题 命 名 与 版 本 后 缀
═══════════════════════════════════════════════════════════════════════════

  PX4 uxrce_dds_client 在 ROS2 话题名后添加版本后缀 (如 _v1, _v2, _v4).
  这是因为 PX4 的 uORB 消息定义有版本号, 不同版本消息结构不兼容.

  常见后缀:
    vehicle_status    → /fmu/out/vehicle_status_v4
    vehicle_command   → /fmu/in/vehicle_command  (无后缀)
    register_ext_component_request → ..._v2
    register_ext_component_reply   → ..._v1
    arming_check_request  → ..._v1
    arming_check_reply     → ..._v1

  注意: 后缀随 PX4 版本变化, 需要与当前 PX4 固件匹配.

═══════════════════════════════════════════════════════════════════════════
               外 部 组 件 注 册 协 议 (DDS 特 有)
═══════════════════════════════════════════════════════════════════════════

  PX4 通过 DDS 拒绝未注册组件的 VehicleCommand 指令. 注册流程:

  1. 控制器 → PX4: RegisterExtComponentRequest
     - register_arming_check = True  (参与解锁检查)
     - px4_ros2_api_version = LATEST

  2. PX4 → 控制器: RegisterExtComponentReply
     - success = True
     - arming_check_id = N  (PX4 分配的回调 ID)

  3. PX4 → 控制器: ArmingCheckRequest (解锁前广播)
     - valid_registrations_mask: 位掩码, 表示哪些组件需要响应

  4. 控制器 → PX4: ArmingCheckReply
     - can_arm_and_run = True  (允许解锁)
     - registration_id = N     (与 arming_check_id 对应)

  注: 注册请求每 1s 重发直到成功. 成功注册后停止重发,
      避免 arming_check_id 变动导致 PX4 超时.

═══════════════════════════════════════════════════════════════════════════
                Offboard 控 制 流 程
═══════════════════════════════════════════════════════════════════════════

  PX4 Offboard 模式需要外部计算机持续提供位置/速度指令. 流程:

  1. 发布 OffboardControlMode (声明控制类型, 如"位置控制")
  2. 发布 TrajectorySetpoint (目标位置, NED 坐标)
  3. 发送 VehicleCommand VEHICLE_CMD_DO_SET_MODE (切换至 Offboard)
  4. PX4 确认进入 Offboard (nav_state=14) 后, 开始追踪 setpoint

  关键:
  - OffboardControlMode + TrajectorySetpoint 必须以 >2Hz 频率持续发布
  - 若停止发布超过 ~0.5s, PX4 自动退出 Offboard 并切换至故障保护模式
  - 本控制器以 10Hz (心跳) + 20Hz (控制循环) 双频率发布, 确保可靠性
  - 着陆阶段: 将 OffboardControlMode.position 设为 False,
    PX4 检测到失去位置控制后自动退出 Offboard, 然后可以安全上锁

═══════════════════════════════════════════════════════════════════════════
                VehicleCommand 指 令 参 数 说 明
═══════════════════════════════════════════════════════════════════════════

  VEHICLE_CMD_COMPONENT_ARM_DISARM (400):
    param1=1.0 → 解锁 (ARM)
    param1=0.0 → 上锁 (DISARM)
    param2=21196 (SITL magic number) → 强制解锁/上锁, 绕过所有安全检查
    注: Force arm/disarm 的 magic number 仅在 SITL 仿真中有效

  VEHICLE_CMD_DO_SET_MODE (176):
    param1=1.0, param2=6.0  → OFFBOARD 模式
    param1=1.0, param2=2.0  → POSCTL (位置保持)
    param1=1.0, param2=4.0  → AUTO.LOITER (悬停)
    param1=1.0, param2=15.0 → STABILIZED (增稳)
    param1=6.0, param2=5.0  → AUTO.LAND (自动降落, 参数编码因 PX4 版本而异)

═══════════════════════════════════════════════════════════════════════════
                    坐 标 系 (关 键!)
═══════════════════════════════════════════════════════════════════════════

  Gazebo 世界坐标:  x=东 (场地 70m 长边), y=北 (8m 宽边), z=上

  PX4 NED (TrajectorySetpoint.position):
    [0] = x = North (北) = Gazebo  +y (场地宽边, 无人机左右)
    [1] = y = East  (东) = Gazebo  +x (场地长边, 飞向投放区方向!)
    [2] = z = Down  (下) = Gazebo  -z (负值 = 向上/离地高度)

  原点 = 起飞点 H: Gazebo(10, 4, 0)
  转换公式: NED_x = world_y - 4
            NED_y = world_x - 10
            NED_z = -world_z

  重要: PX4 的 NED 坐标系中 position[1] (east) 才是场地长边方向!
        position[0] (north) 是场地宽边方向.
        航点定义必须使用 (north, east, down) 顺序, 否则无人机飞错方向.

═══════════════════════════════════════════════════════════════════════════
                    任 务 流 程
═══════════════════════════════════════════════════════════════════════════

  H 标识处起飞 (5m) → 投放区中心 → 识别 2 个较小圆筒
  (降高至 2m, 悬停 5s, 回升至 5m) → 侦察区中心
  → 依次飞过 5 个圆筒 (5m) → 返回 H 精准降落

  投放区 (x:40~45):  3 个圆筒 (15cm/20cm/25cm), 只访问前 2 个
  侦察区 (x:65~70):  5 个 20cm 圆筒, 5m 高度穿越不悬停
"""

import math
import time
import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ==========================================================================
# px4_msgs — PX4 uORB 消息的 ROS2 等价定义
#
# 这些消息由 px4_msgs 包通过 rosidl_generate_interfaces() 从 .msg 文件
# 编译生成。消息结构与 PX4 内部 uORB 消息一一对应。
#
# 路径: drone_mission_dds_ws/src/px4_msgs/msg/*.msg
# 来源: PX4-Autopilot 仓库的 uORB 定义, 通过脚本同步
# ==========================================================================
from px4_msgs.msg import (
    OffboardControlMode,    # 声明 Offboard 控制类型 (位置/速度/加速度)
    TrajectorySetpoint,     # 目标轨迹设定点 (NED 位置+速度+加速度+偏航)
    VehicleCommand,         # 通用车辆指令 (解锁/上锁/模式切换)
    VehicleOdometry,        # 里程计 (位置+姿态四元数+速度)
    VehicleStatus,          # 飞行状态 (解锁状态/导航状态/系统状态)
    VehicleLandDetected,    # 着陆检测 (PX4 land detector 输出)
    RegisterExtComponentRequest,   # 外部组件注册请求
    RegisterExtComponentReply,     # 注册确认 (含 arming_check_id)
    ArmingCheckReply,        # 解锁检查回复
    ArmingCheckRequest,      # 解锁检查请求
)


# ==========================================================================
# MissionState — 任务状态枚举
#
# 使用 Python Enum 定义所有飞行阶段, state machine 通过 match/case 分发.
# auto() 自动分配数值, 状态转移通过 NEXT_STATE 映射定义.
# ==========================================================================
class MissionState(Enum):
    """19 状态任务状态机"""

    # --- 初始化阶段 ---
    INIT = auto()               # 0: 等待里程计数据稳定 (≥50条)
    REGISTER = auto()           # 1: 等待 PX4 外部组件注册确认
    STREAM_SETPOINTS = auto()   # 2: 预热 setpoint 流 (发送 100 个地面定位点)

    # --- 解锁阶段 ---
    ARM = auto()                # 3: 切换 OFFBOARD + 解锁

    # --- 投放区 (圆筒识别与悬停) ---
    TAKEOFF = auto()            # 4: H 点起飞至巡航高度 5m
    TRANSIT_DROP = auto()       # 5: 飞向投放区中心 (32.5m 东)
    HOVER_C1 = auto()           # 6: 筒1 (15cm) 降高至 2m 悬停 5s
    ASCEND_C1 = auto()          # 7: 筒1 上方回升至 5m
    HOVER_C2 = auto()           # 8: 筒2 (20cm) 降高至 2m 悬停 5s
    ASCEND_C2 = auto()          # 9: 筒2 上方回升至 5m

    # --- 侦察区 (5 筒依次穿越) ---
    TRANSIT_RECON = auto()      # 10: 飞向侦察区中心 (57.5m 东)
    RECON_C1 = auto()           # 11: 侦察筒1 穿越
    RECON_C2 = auto()           # 12: 侦察筒2 穿越
    RECON_C3 = auto()           # 13: 侦察筒3 穿越
    RECON_C4 = auto()           # 14: 侦察筒4 穿越
    RECON_C5 = auto()           # 15: 侦察筒5 穿越

    # --- 返航与着陆 ---
    RETURN = auto()             # 16: 返回 H 位置 (5m)
    LAND = auto()               # 17: 垂直下降 → 停止 Offboard → 上锁
    COMPLETE = auto()           # 18: 任务完成


# ==========================================================================
# MissionController — 任务控制器主类 (ROS2 Node)
# ==========================================================================
class MissionController(Node):
    """
    micro-XRCE-DDS 圆筒侦察任务控制器.

    通过 DDS 直连 PX4 (不经过 MAVROS), 以 20Hz 控制循环执行完整无人机任务:
    起飞 → 投放区圆筒识别悬停 → 侦察区穿越 → 返航降落.
    """

    # ======================================================================
    # 航点列表 — PX4 本地 NED 坐标系
    #
    # 坐标系: position[0]=North(Gazebo+y), position[1]=East(Gazebo+x),
    #          position[2]=Down(-Gazebo+z)
    # 原点 = H 起飞点 Gazebo(10,4,0):
    #   NED_x = world_y - 4
    #   NED_y = world_x - 10
    #   NED_z = -world_z
    #
    # 投放区 (Gazebo x:40~45): 3 个圆筒, 只访问 2 个较小的
    # 侦察区 (Gazebo x:65~70): 5 个 20cm 圆筒, 全部穿越
    # ======================================================================
    WAYPOINTS = {
        MissionState.TAKEOFF:       (0.0,   0.0,  -5.0),   # H 点起飞 5m
        MissionState.TRANSIT_DROP:  (0.0,  32.5,  -5.0),   # 投放区中心 (w:42.5,4)
        MissionState.HOVER_C1:      (-1.0, 31.5,  -2.0),   # 筒1 15cm (w:41.5,3.0)
        MissionState.ASCEND_C1:     (-1.0, 31.5,  -5.0),   # 筒1 回升 5m
        MissionState.HOVER_C2:      (1.5,  33.0,  -2.0),   # 筒2 20cm (w:43.0,5.5)
        MissionState.ASCEND_C2:     (1.5,  33.0,  -5.0),   # 筒2 回升 5m
        MissionState.TRANSIT_RECON: (0.0,  57.5,  -5.0),   # 侦察区中心 (w:67.5,4)
        MissionState.RECON_C1:      (-2.0, 56.0,  -5.0),   # 侦察筒1 (w:66.0,2.0)
        MissionState.RECON_C2:      (1.5,  57.5,  -5.0),   # 侦察筒2 (w:67.5,5.5)
        MissionState.RECON_C3:      (3.0,  58.5,  -5.0),   # 侦察筒3 (w:68.5,7.0)
        MissionState.RECON_C4:      (-0.5, 56.5,  -5.0),   # 侦察筒4 (w:66.5,3.5)
        MissionState.RECON_C5:      (-2.5, 59.0,  -5.0),   # 侦察筒5 (w:69.0,1.5)
        MissionState.RETURN:        (0.0,   0.0,  -5.0),   # 返回 H 5m
    }

    # 投放区圆筒悬停时长 (s)
    HOVER_DURATION = {
        MissionState.HOVER_C1: 5.0,
        MissionState.HOVER_C2: 5.0,
    }

    # 圆筒标签 (用于日志)
    CYL_LABELS = {
        MissionState.HOVER_C1: ('1', '15cm'),
        MissionState.HOVER_C2: ('2', '20cm'),
    }

    # 状态转移映射 — 定义每个航点完成后跳转到哪个状态
    NEXT_STATE = {
        MissionState.TAKEOFF:       MissionState.TRANSIT_DROP,
        MissionState.TRANSIT_DROP:  MissionState.HOVER_C1,
        MissionState.HOVER_C1:      MissionState.ASCEND_C1,
        MissionState.ASCEND_C1:     MissionState.HOVER_C2,
        MissionState.HOVER_C2:      MissionState.ASCEND_C2,
        MissionState.ASCEND_C2:     MissionState.TRANSIT_RECON,
        MissionState.TRANSIT_RECON: MissionState.RECON_C1,
        MissionState.RECON_C1:      MissionState.RECON_C2,
        MissionState.RECON_C2:      MissionState.RECON_C3,
        MissionState.RECON_C3:      MissionState.RECON_C4,
        MissionState.RECON_C4:      MissionState.RECON_C5,
        MissionState.RECON_C5:      MissionState.RETURN,
        MissionState.RETURN:        MissionState.LAND,
    }

    # ======================================================================
    # __init__ — ROS2 节点初始化
    # ======================================================================
    def __init__(self):
        super().__init__('mission_controller_dds')

        # ------------------------------------------------------------------
        # ROS2 参数 — 可通过 YAML 文件 (config/mission_params.yaml) 或
        # 命令行 (--ros-args -p param:=value) 覆盖默认值
        # ------------------------------------------------------------------
        self.declare_parameter('takeoff_height', 5.0)         # 巡航高度 (m)
        self.declare_parameter('waypoint_tolerance', 0.5)     # 航点到达容差 (m)
        self.declare_parameter('takeoff_tolerance', 0.3)      # 起飞到达容差 (m)
        self.declare_parameter('hover_duration', 5.0)         # 投放区悬停时长 (s)
        self.declare_parameter('control_frequency', 20.0)     # 控制频率 (Hz)
        self.declare_parameter('max_error_count', 5)          # 最大容错次数
        self.declare_parameter('waypoint_timeout', 120.0)     # 单航点超时 (s)
        self.declare_parameter('land_altitude', 0.15)         # 着陆确认高度 (m)

        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.takeoff_tolerance = self.get_parameter('takeoff_tolerance').value
        self.hover_duration = self.get_parameter('hover_duration').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_error_count = self.get_parameter('max_error_count').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.land_altitude = self.get_parameter('land_altitude').value

        # ------------------------------------------------------------------
        # QoS 配置
        #
        # PX4 uxrce_dds_client 以 BEST_EFFORT + TRANSIENT_LOCAL 发布数据.
        # ROS2 订阅方必须使用相同 QoS, 否则 DDS 中间件静默丢弃消息.
        #
        # BEST_EFFORT vs RELIABLE:
        #   - BEST_EFFORT: 不重传丢失的数据包, 适合高频传感器数据
        #   - RELIABLE: 重传直到确认, 增加延迟, PX4 不使用
        #
        # TRANSIENT_LOCAL vs VOLATILE:
        #   - TRANSIENT_LOCAL: 新订阅者加入时收到最新一帧缓存数据
        #   - VOLATILE: 新订阅者只能收到发布后的数据
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ==================================================================
        # [下行] DDS 订阅 — PX4 发布, 控制器订阅
        #
        # 数据流: PX4 uORB → uxrce_dds_client → XRCE/UDP → Agent → DDS → ROS2
        # 话题前缀: /fmu/out/ (from PX4, output)
        # ==================================================================

        # 里程计: 约 30-50Hz, PX4 EKF 融合 IMU+GPS 后的位置估计
        # VehicleOdometry.position[3]: NED 坐标 (x=north, y=east, z=down)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self._cb_odom, qos)

        # 飞行状态: 约 10Hz, 包含 arming_state 和 nav_state
        # 注意版本后缀 _v4: PX4 uxrce_dds_client 给不同版本的消息加后缀
        # vehicle_status 当前版本为 v4, 若 PX4 版本升级可能变为 v5
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)

        # 着陆检测: PX4 land detector 输出, 分三个状态:
        #   ground_contact → maybe_landed → landed
        # 用于精确判断无人机是否已触地
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._cb_land_detected, qos)

        # ==================================================================
        # [上行] DDS 发布 — 控制器发布, PX4 订阅
        #
        # 数据流: ROS2 → DDS → Agent → XRCE/UDP → uxrce_dds_client → PX4 uORB
        # 话题前缀: /fmu/in/ (to PX4, input)
        # ==================================================================

        # Offboard 控制模式声明:
        #   告诉 PX4 我们要用哪种控制模式 (位置/速度/加速度/姿态/推力).
        #   本控制器使用纯位置控制 (position=True, 其余 False).
        #   必须 >2Hz 持续发布, 否则 PX4 退出 Offboard.
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)

        # 轨迹设定点:
        #   目标位置 (NED)、速度、加速度、偏航角.
        #   velocity/acceleration/jerk 设为 NaN 表示不控制该维度.
        #   必须 >2Hz 与 OffboardControlMode 一起发布.
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        # 车辆指令:
        #   通用 MAVLink 风格指令 (解锁/上锁/模式切换).
        #   通过 VehicleCommand 话题发送, PX4 commander 模块处理.
        #   常用指令:
        #     VEHICLE_CMD_COMPONENT_ARM_DISARM (400): 解锁/上锁
        #     VEHICLE_CMD_DO_SET_MODE (176): 飞行模式切换
        self._cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ==================================================================
        # 外部组件注册 — DDS 模式特有的解锁前提
        #
        # PX4 安全机制: 通过 DDS 发送 VehicleCommand 的外部控制器必须
        # 先在 PX4 注册为"外部组件", 否则 PX4 commander 拒绝指令.
        #
        # 注册流程 (4 步握手):
        #   1. 控制器 → PX4: RegisterExtComponentRequest
        #      (声明: 我参与解锁检查, 模式=arming_check)
        #   2. PX4 → 控制器: RegisterExtComponentReply
        #      (确认注册, 返回 arming_check_id)
        #   3. PX4 → 控制器: ArmingCheckRequest (解锁前 PX4 主动询问)
        #      (valid_registrations_mask 指示哪些组件需要回复)
        #   4. 控制器 → PX4: ArmingCheckReply
        #      (can_arm_and_run=True, 告诉 PX4 可以解锁)
        #
        # 话题版本后缀:
        #   register_ext_component_request: _v2 (ROS2→PX4)
        #   register_ext_component_reply:   _v1 (PX4→ROS2)
        #   arming_check_request:           _v1 (PX4→ROS2)
        #   arming_check_reply:             _v1 (ROS2→PX4)
        # ==================================================================
        self._registered = False           # 注册是否成功
        self._arming_check_id = 0          # PX4 分配的回调 ID
        self._reg_request_id = int(time.time() * 1000) & 0xFFFFFFFF

        self._reg_pub = self.create_publisher(
            RegisterExtComponentRequest,
            '/fmu/in/register_ext_component_request_v2', qos)
        self.create_subscription(
            RegisterExtComponentReply,
            '/fmu/out/register_ext_component_reply_v1',
            self._cb_reg_reply, qos)
        self.create_subscription(
            ArmingCheckRequest,
            '/fmu/out/arming_check_request_v1',
            self._cb_arming_check, qos)
        self._arming_check_pub = self.create_publisher(
            ArmingCheckReply,
            '/fmu/in/arming_check_reply_v1', qos)

        # 构造函数中立即发起首次注册请求
        self._register_component()

        # ------------------------------------------------------------------
        # 内部状态变量
        # ------------------------------------------------------------------
        # PX4 遥测数据 (通过 DDS 订阅更新)
        self._pos = [0.0, 0.0, 0.0]          # NED 位置 [north, east, down]
        self._pos_count = 0                    # 已接收位置消息数 (用于初始化)
        self._armed = False                    # PX4 arming_state (True=已解锁)
        self._nav_state = 0                    # PX4 nav_state (14=OFFBOARD)
        self._landed = False                   # PX4 land detector (True=已着陆)

        # 状态机
        self._state = MissionState.INIT        # 当前状态
        self._state_enter_time = time.time()   # 进入当前状态的时间戳
        self._setpoint_counter = 0             # setpoint 发送计数

        # 指令防抖与重试
        self._last_action_time = 0.0           # 上次发送指令时间 (防抖)
        self._arm_attempts = 0                 # 解锁尝试次数
        self._error_count = 0                  # 错误计数
        self._diag_counter = 0                 # 诊断日志输出间隔计数器
        self._land_requested = False           # 着陆流程是否已启动
        self._disarm_count = 0                 # 上锁指令发送次数

        # setpoint 目标 (被 _heartbeat 以 10Hz 发布到 PX4)
        self._target = [0.0, 0.0, 0.0]        # NED 目标位置
        self._yaw = math.pi / 2                  # 目标偏航角 (rad) — π/2=朝东(Gazebo +x), 与航点飞行方向一致

        # 着陆标志: True 时 _heartbeat 停止发送 OffboardControlMode.position,
        # PX4 检测到失去位置控制 → 自动退出 Offboard → 可以安全上锁
        self._stop_offboard = False

        # ------------------------------------------------------------------
        # 定时器
        #   - 20Hz 控制循环: 状态机驱动, setpoint 更新
        #   - 10Hz 心跳: 维持 Offboard 模式 (OffboardControlMode + Setpoint)
        #   - 1Hz 注册刷新: 重试组件注册直到成功
        # ------------------------------------------------------------------
        dt = 1.0 / self.control_frequency
        self.create_timer(dt, self._control_loop)
        self.create_timer(0.1, self._heartbeat)
        self.create_timer(1.0, self._registration_timer)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Mission Controller (micro-XRCE-DDS) v2 启动')
        self.get_logger().info(f'  巡航高度: {self.takeoff_height}m  '
                               f'悬停: {self.hover_duration}s')
        self.get_logger().info('  任务: 投放区2筒(降高2m) + 侦察区5筒(5m穿越)')
        self.get_logger().info('  通信: micro-XRCE-DDS (Agent UDP :8888)')
        self.get_logger().info('  等待 PX4 odometry 数据...')
        self.get_logger().info('=' * 60)

    # ======================================================================
    # DDS 回调函数 — 接收 PX4 → Agent → DDS → ROS2 的数据
    # ======================================================================

    def _cb_odom(self, msg: VehicleOdometry):
        """
        /fmu/out/vehicle_odometry 回调 (~30-50Hz).

        PX4 EKF (扩展卡尔曼滤波器) 输出的融合位置估计.
        msg.position[0/1/2] = NED x(north) / y(east) / z(down) (米).

        数据来源链: IMU+GPS+磁力计 → EKF2 → vehicle_odometry uORB
        → uxrce_dds_client → Agent → DDS → ROS2 → 本回调
        """
        self._pos = [float(msg.position[0]), float(msg.position[1]),
                     float(msg.position[2])]
        self._pos_count += 1

    def _cb_status(self, msg: VehicleStatus):
        """
        /fmu/out/vehicle_status_v4 回调 (~10Hz).

        PX4 commander 模块发布的飞行状态.
        msg.arming_state: 1=DISARMED, 2=ARMED (已解锁)
        msg.nav_state: 当前导航状态
          0=MANUAL, 2=POSCTL, 3=AUTO_MISSION, 4=AUTO_LOITER,
          14=OFFBOARD (外部控制), 17=AUTO_TAKEOFF, 18=AUTO_LAND

        数据来源链: commander → vehicle_status uORB
        → uxrce_dds_client → Agent → DDS → ROS2 → 本回调

        注意话题名后缀 _v4: PX4 uORB 消息有版本号, 不同版本结构不兼容,
        uxrce_dds_client 自动添加后缀以避免版本冲突.
        """
        self._armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self._nav_state = msg.nav_state

    def _cb_land_detected(self, msg: VehicleLandDetected):
        """
        /fmu/out/vehicle_land_detected 回调 (~10Hz).

        PX4 land detector 模块输出的着陆检测状态:
          freefall → ground_contact → maybe_landed → landed
        msg.landed: True 表示无人机已稳定着陆 (第三阶段).

        用途: 着陆确认, 只有 land detector 确认着陆后才发送上锁指令,
        避免空中误上锁或地面弹跳导致上锁失败.
        """
        self._landed = msg.landed

    # ======================================================================
    # 外部组件注册协议 — DDS 模式特有的解锁前提
    #
    # PX4 安全机制要求通过 DDS 控制的外部计算机必须先注册.
    # 未注册的组件发送的 VehicleCommand 会被 PX4 commander 拒绝.
    # ======================================================================

    def _register_component(self):
        """
        向 PX4 发起外部组件注册请求.

        发送 RegisterExtComponentRequest 到 /fmu/in/register_ext_component_request_v2.
        msg.name: 组件名称 (char[25], 用于 PX4 日志标识)
        msg.register_arming_check: True (注册解锁检查回调能力)
        msg.register_mode: False (不注册自定义飞行模式)
        msg.px4_ros2_api_version: 与 PX4 固件编译时的 API 版本对齐

        PX4 收到后返回 RegisterExtComponentReply (含 arming_check_id).
        """
        msg = RegisterExtComponentRequest()
        msg.timestamp = self._now_us()
        msg.request_id = self._reg_request_id
        # char[25] 字段需要用 uint8 数组填充 (ROS2 IDL 不支持固定长度字符串)
        name_arr = np.zeros(25, dtype=np.uint8)
        name_str = 'mission_controller_dds'
        for i, c in enumerate(name_str[:25]):
            name_arr[i] = ord(c)
        msg.name = name_arr
        msg.px4_ros2_api_version = \
            RegisterExtComponentRequest.LATEST_PX4_ROS2_API_VERSION
        msg.register_arming_check = True
        msg.register_mode = False
        msg.register_mode_executor = False
        msg.enable_replace_internal_mode = False
        msg.activate_mode_immediately = False
        self._reg_pub.publish(msg)

    def _registration_timer(self):
        """
        1Hz 定时器: 重发注册请求.

        注册成功后停止重发 (self._registered = True 时跳过).
        原因: 每次注册 PX4 可能分配新的 arming_check_id,
        旧 ID 失效会导致 PX4 解锁检查超时.
        """
        if not self._registered:
            self._register_component()

    def _cb_reg_reply(self, msg: RegisterExtComponentReply):
        """
        /fmu/out/register_ext_component_reply_v1 回调.

        PX4 对注册请求的回复.
        msg.request_id: 与发送的请求匹配
        msg.success: True 表示注册成功
        msg.arming_check_id: PX4 分配的唯一回调 ID,
          后续 ArmingCheckReply 中需要携带此 ID 以证明身份
        """
        if msg.request_id == self._reg_request_id and msg.success:
            if not self._registered:
                self._arming_check_id = msg.arming_check_id
                self._registered = True
                self.get_logger().info(
                    f'Registration confirmed! arming_check_id='
                    f'{msg.arming_check_id}')

    def _cb_arming_check(self, msg: ArmingCheckRequest):
        """
        /fmu/out/arming_check_request_v1 回调.

        PX4 在尝试解锁前向所有已注册外部组件广播 ArmingCheckRequest.
        每个组件需要回复 ArmingCheckReply 声明"可以解锁".

        msg.valid_registrations_mask: 位掩码, bit N 对应 arming_check_id=N.
        只有掩码中标记为有效的组件才需要回复.

        回复设置:
          can_arm_and_run = True: 允许 PX4 解锁
          mode_req_* = False: 不要求特定飞行模式
          registration_id = self._arming_check_id: 身份验证
        """
        # 检查 PX4 是否认为我们的注册 ID 有效
        if not (msg.valid_registrations_mask & (1 << self._arming_check_id)):
            return
        reply = ArmingCheckReply()
        reply.timestamp = self._now_us()
        reply.request_id = msg.request_id
        reply.registration_id = self._arming_check_id
        reply.can_arm_and_run = True        # 关键: 允许解锁
        reply.health_component_index = 0
        reply.health_component_is_present = False
        reply.health_component_warning = False
        reply.health_component_error = False
        # 不要求任何特定飞行模式 (由控制器自行管理)
        reply.mode_req_angular_velocity = False
        reply.mode_req_attitude = False
        reply.mode_req_local_alt = False
        reply.mode_req_local_position = False
        reply.mode_req_local_position_relaxed = False
        reply.mode_req_global_position = False
        reply.mode_req_global_position_relaxed = False
        reply.mode_req_mission = False
        reply.mode_req_home_position = False
        reply.mode_req_prevent_arming = False
        reply.mode_req_manual_control = False
        reply.num_events = 0
        self._arming_check_pub.publish(reply)

    # ======================================================================
    # 工具函数
    # ======================================================================

    def _now_us(self) -> int:
        """
        获取当前微秒级时间戳.

        PX4 内部使用微秒时间戳 (uint64), 所有 VehicleCommand 和
        OffboardControlMode/TrajectorySetpoint 的 timestamp 字段
        都需要微秒单位的 PX4 时间.
        """
        return int(self.get_clock().now().nanoseconds / 1000)

    def _now(self) -> float:
        """获取当前时间 (秒), 用于状态机计时和超时判断."""
        return time.time()

    def _elapsed_in_state(self) -> float:
        """当前状态已持续的时间 (秒)."""
        return self._now() - self._state_enter_time

    def _dist(self, x: float, y: float, z: float) -> float:
        """
        计算当前位置到目标点 (x,y,z) 的三维欧氏距离.

        用于航点到达判断. x/y/z 为 NED 坐标.
        """
        return math.sqrt(
            (x - self._pos[0]) ** 2 +
            (y - self._pos[1]) ** 2 +
            (z - self._pos[2]) ** 2
        )

    def _reached(self, x: float, y: float, z: float,
                 tol: float | None = None) -> bool:
        """
        判断是否到达目标点 (距离 < 容差).

        Args:
            tol: 容差 (米), 默认使用 waypoint_tolerance 参数 (0.5m)
        """
        t = tol if tol is not None else self.waypoint_tolerance
        return self._dist(x, y, z) < t

    def _set_target(self, x: float, y: float, z: float) -> None:
        """
        更新 setpoint 目标位置 (NED 坐标).

        此目标会在下一个心跳周期 (10Hz) 被发布到
        /fmu/in/trajectory_setpoint, PX4 内部位置控制器追踪此目标.
        """
        self._target = [float(x), float(y), float(z)]

    def _transition(self, new: MissionState) -> None:
        """
        状态机切换.

        记录旧状态、新状态、进入时间, 重置重试计数器.
        状态转移日志通过 [STATE] 前缀输出, 便于追踪任务进度.
        """
        old = self._state
        self._state = new
        self._state_enter_time = self._now()
        self._arm_attempts = 0
        self.get_logger().info(f'[STATE] {old.name} → {new.name}')

    # ======================================================================
    # VehicleCommand 发送 — ROS2 → DDS → Agent → XRCE → PX4
    #
    # VehicleCommand 是通用 MAVLink 风格指令接口.
    # PX4 commander 模块订阅 vehicle_command uORB 话题并处理指令.
    #
    # from_external=True: 标记指令来自外部计算机 (非机载).
    # 配合组件注册机制, PX4 只接受已注册组件的指令.
    # ======================================================================

    def _send_cmd(self, command: int, p1: float = 0.0, p2: float = 0.0):
        """
        发送 VehicleCommand 到 PX4.

        通过 /fmu/in/vehicle_command DDS 话题发布, Agent 将其转发为
        XRCE 流 → uxrce_dds_client 接收 → 发布到 vehicle_command uORB
        → PX4 commander 模块处理.

        Args:
            command: MAVLink 指令 ID (如 400=ARM_DISARM, 176=SET_MODE)
            p1: param1 (语义由 command 决定)
            p2: param2 (语义由 command 决定)
        """
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        # 目标与来源: 1 表示飞控本身
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True  # 关键: 标记为外部计算机发送
        self._cmd_pub.publish(msg)

    def _arm(self):
        """
        标准解锁.

        VEHICLE_CMD_COMPONENT_ARM_DISARM (400):
          param1=1.0 → Arm (解锁, 电机开始转)
          param2=0.0 → 正常解锁 (需要所有 preflight checks 通过)

        PX4 preflight checks 包括:
          - 传感器校准 (gyro/accel/mag/baro)
          - EKF 收敛
          - GCS 连接 (可通过 NAV_DLL_ACT=0 禁用)
          - 安全开关 (SITL 默认关闭)
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _force_arm(self):
        """
        SITL 强制解锁 — 绕过所有 preflight checks.

        VEHICLE_CMD_COMPONENT_ARM_DISARM (400):
          param1=1.0 → Arm
          param2=21196 → PX4 SITL magic number

        当 param2=21196 时, PX4 commander 跳过以下检查:
          - GCS 心跳检查
          - RC 遥控器检查
          - EKF 收敛检查
          - 传感器校准检查
          - 安全开关检查

        ⚠️ 仅在 SITL 仿真中有效, 真实硬件上此 magic number 被忽略.
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                       1.0, 21196.0)

    def _disarm(self):
        """
        标准上锁.

        VEHICLE_CMD_COMPONENT_ARM_DISARM (400):
          param1=0.0 → Disarm (上锁, 电机停止)

        PX4 上锁条件:
          - 无人机必须已着陆 (land detector 确认)
          - 不能处于 OFFBOARD 模式 (需先退出)
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def _force_disarm(self):
        """
        强制上锁 — 绕过所有上锁前检查.

        VEHICLE_CMD_COMPONENT_ARM_DISARM (400):
          param1=0.0 → Disarm
          param2=21196 → SITL magic number (跳过模式/着陆检查)

        用在着陆完成后的最后一步, 确保螺旋桨立即停止.
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                       0.0, 21196.0)

    def _request_offboard(self):
        """
        请求切换至 Offboard 模式.

        VEHICLE_CMD_DO_SET_MODE (176):
          param1=1.0 → MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
          param2=6.0 → PX4_CUSTOM_MAIN_MODE_OFFBOARD

        PX4 收到后尝试切换至 Offboard (nav_state=14).
        前提: OffboardControlMode + TrajectorySetpoint 正在持续发布.
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def _request_land(self):
        """
        请求自动降落 (AUTO.LAND).

        VEHICLE_CMD_DO_SET_MODE (176):
          param1=6.0 → 包含 MAV_MODE_FLAG_SAFETY_ARMED
          param2=5.0 → PX4 AUTO.LAND 子模式

        注: AUTO.LAND 模式切换参数编码因 PX4 版本而异,
        当前参数在 PX4 v1.15 中可能不完全正确, 作为备选方案使用.
        """
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 6.0, 5.0)

    # ======================================================================
    # 心跳 (10Hz 定时器) — 维持 Offboard 模式的必需组件
    #
    # PX4 要求以 >2Hz 频率持续收到 OffboardControlMode + TrajectorySetpoint.
    # 如果停止发布超过 ~0.5s (PX4 COM_OF_LOSS_T 参数), PX4 自动退出 Offboard
    # 并切换到 COM_FLTMODE 指定的故障保护模式.
    #
    # 本控制器以 10Hz 发布心跳, 20Hz (控制循环) 更新 setpoint 值,
    # 两个频率独立运行, 确保 Offboard 控制信号不会意外中断.
    # ======================================================================

    def _heartbeat(self):
        """
        10Hz 心跳 — 发布 OffboardControlMode + TrajectorySetpoint.

        OffboardControlMode 声明控制类型:
          position=True: 使用位置控制 (PX4 mc_pos_control 追踪位置目标)
          其余字段 False: 不使用速度/加速度/姿态/推力控制

        TrajectorySetpoint 设定目标:
          position: NED 目标位置 [north, east, down]
          velocity/acceleration/jerk: NaN 表示不控制 (PX4 忽略 NaN 字段)
          yaw: 目标偏航角 (rad)
          yawspeed: NaN 表示不控制偏航速率

        着陆特殊处理:
          当 self._stop_offboard=True 时, ocm.position=False.
          PX4 检测到位置控制停止 → 退出 Offboard → 进入安全模式.
          然后控制器可以发送上锁指令 (Offboard 模式下 PX4 拒绝上锁).
        """
        # --- OffboardControlMode ---
        ocm = OffboardControlMode()
        ocm.timestamp = self._now_us()
        # 着陆时停止位置控制声明, PX4 将自动退出 Offboard
        ocm.position = not self._stop_offboard
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self._offboard_pub.publish(ocm)

        # --- TrajectorySetpoint ---
        tsp = TrajectorySetpoint()
        tsp.timestamp = self._now_us()
        tsp.position = [float(self._target[0]), float(self._target[1]),
                        float(self._target[2])]
        # NaN = 不控制该维度 (PX4 内部忽略 NaN 值)
        tsp.velocity = [float('nan')] * 3
        tsp.acceleration = [float('nan')] * 3
        tsp.jerk = [float('nan')] * 3
        tsp.yaw = float(self._yaw)
        tsp.yawspeed = float('nan')
        self._traj_pub.publish(tsp)

    # ======================================================================
    # 控制循环 (20Hz 定时器) — 主状态机
    #
    # 每个控制周期 (0.05s) 执行一次:
    #   1. 检查位置数据是否就绪
    #   2. 根据当前状态执行对应处理函数
    #   3. 处理函数更新 _target (setpoint), 心跳线程负责发布
    # ======================================================================

    def _control_loop(self):
        """
        20Hz 主状态机.

        每个 tick 执行当前状态的处理函数. 状态转移由各处理函数
        调用 _transition() 触发, 下一个 tick 自动执行新状态.
        """
        self._setpoint_counter += 1
        self._diag_counter += 1

        # 等待至少 10 条里程计消息 (确保位置数据已开始流入)
        if self._pos_count < 10:
            return

        # 状态机分发: match/case 确保所有状态都被处理
        match self._state:
            case MissionState.INIT:
                self._do_init()
            case MissionState.REGISTER:
                self._do_register()
            case MissionState.STREAM_SETPOINTS:
                self._do_stream()
            case MissionState.ARM:
                self._do_arm()
            case MissionState.TAKEOFF:
                self._do_takeoff()
            case MissionState.TRANSIT_DROP:
                self._do_transit()
            case MissionState.HOVER_C1:
                self._do_hover()
            case MissionState.ASCEND_C1:
                self._do_transit()
            case MissionState.HOVER_C2:
                self._do_hover()
            case MissionState.ASCEND_C2:
                self._do_transit()
            case MissionState.TRANSIT_RECON:
                self._do_transit()
            case MissionState.RECON_C1:
                self._do_transit()
            case MissionState.RECON_C2:
                self._do_transit()
            case MissionState.RECON_C3:
                self._do_transit()
            case MissionState.RECON_C4:
                self._do_transit()
            case MissionState.RECON_C5:
                self._do_transit()
            case MissionState.RETURN:
                self._do_return()
            case MissionState.LAND:
                self._do_land()
            case MissionState.COMPLETE:
                pass  # 任务完成, 空转

    # ======================================================================
    # 状态处理函数 — 每个状态的具体逻辑
    # ======================================================================

    def _do_init(self):
        """
        Phase 0: INIT — 等待里程计数据稳定.

        PX4 启动后 EKF 需要数秒收敛, 期间位置数据可能跳变.
        等待 50 条消息 (~2.5s @ 20Hz) 确保数据稳定.
        setpoint 保持在地面位置 (z=0, 即当前高度).
        """
        self._set_target(self._pos[0], self._pos[1], 0.0)

        if self._pos_count >= 50:
            self.get_logger().info(
                f'Odometry 稳定 ({self._pos_count} msgs), '
                f'等待 PX4 注册...')
            self._transition(MissionState.REGISTER)

    def _do_register(self):
        """
        Phase 1: REGISTER — 等待外部组件注册完成.

        PX4 需要确认外部组件的注册才能接受 VehicleCommand.
        注册请求在 __init__ 中已发送, _registration_timer 每 1s 重试.
        最长等待 30s, 超时则报错.
        """
        self._set_target(self._pos[0], self._pos[1], 0.0)

        if self._registered:
            self.get_logger().info('组件注册成功, 预热 setpoint 流...')
            self._transition(MissionState.STREAM_SETPOINTS)
        elif self._elapsed_in_state() > 30.0:
            self.get_logger().error(
                '注册超时 (30s)! 请检查 MicroXRCEAgent 和 PX4 是否正常运行.')
            self._error_count += 1
            if self._error_count > self.max_error_count:
                self._transition(MissionState.COMPLETE)

    def _do_stream(self):
        """
        Phase 2: STREAM_SETPOINTS — 预热 setpoint 流.

        PX4 Offboard 模式要求切换之前 setpoint 流已经在发布.
        否则切换时会因为 "无 setpoint" 立即退出 Offboard.

        做法: 在地面位置发送 100 个 setpoint (5s @ 20Hz),
        确保 PX4 收到稳定的 setpoint 流后再进入解锁阶段.
        """
        if self._setpoint_counter < 100:
            if self._setpoint_counter == 1:
                self.get_logger().info('流式发送 setpoint (当前位置)...')
            self._set_target(self._pos[0], self._pos[1], 0.0)
            return

        # 诊断输出 (约每 5s)
        if self._diag_counter >= 100:
            self._diag_counter = 0
            self.get_logger().info(
                f'[DIAG] pos=({self._pos[0]:.1f},{self._pos[1]:.1f},'
                f'{self._pos[2]:.2f}) nav_state={self._nav_state} '
                f'armed={self._armed} registered={self._registered}')

        self._transition(MissionState.ARM)

    def _do_arm(self):
        """
        Phase 3: ARM — 切换 Offboard 模式并解锁.

        流程:
          第 1 步: 发送 VEHICLE_CMD_DO_SET_MODE 进入 Offboard (nav_state=14)
          第 2 步: Offboard 就绪后, 发送解锁指令
                   - 首次尝试 force arm (param2=21196, 绕过所有检查)
                   - 失败则回退到标准 arm
          第 3 步: 解锁成功 → TAKEOFF, 开始飞行

        防抖: 指令间隔 ≥2-3s, 避免 PX4 commander 过载.

        设计决策: 在 OFFBOARD 模式下直接解锁 (而非 POSCTL→解锁→OFFBOARD).
        旧方案 POSCTL 过渡会导致 PX4 sys_status 从 STANDBY(3) 降到 UNINIT(0),
        触发 EKF 重置. 新方案绕过此 PX4 bug.
        """
        now = self._now()

        # setpoint 保持在地面
        self._set_target(self._pos[0], self._pos[1], 0.0)

        # 诊断 (约每 4s)
        if self._diag_counter >= 80:
            self._diag_counter = 0
            self.get_logger().info(
                f'[ARM] nav_state={self._nav_state} armed={self._armed} '
                f'attempts={self._arm_attempts}')

        # 第 1 步: 确保进入 Offboard 模式
        if self._nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if now - self._last_action_time > 3.0:
                self.get_logger().info(
                    f'切 OFFBOARD (当前 nav_state={self._nav_state})...')
                self._request_offboard()
                self._last_action_time = now
                self._arm_attempts += 1
            return

        # 第 2 步: 已在 Offboard, 发送解锁指令
        if not self._armed and now - self._last_action_time > 2.0:
            if self._arm_attempts <= 1:
                # 首次尝试 force arm (绕过 GCS/EKF/RC 等检查)
                self.get_logger().info(
                    'Force arm (MAV_CMD=400 param2=21196)...')
                self._force_arm()
            else:
                self.get_logger().info(
                    f'标准解锁 (attempt {self._arm_attempts})...')
                self._arm()
            self._last_action_time = now
            self._arm_attempts += 1
            return

        # 第 3 步: 解锁成功
        if self._armed:
            self.get_logger().info('解锁成功! 开始航点任务...')
            self._transition(MissionState.TAKEOFF)

        # 超时保护: 120s 后放弃
        if self._elapsed_in_state() > 120.0:
            self.get_logger().error(
                f'解锁超时 (120s)! nav_state={self._nav_state} '
                f'armed={self._armed}')
            self._transition(MissionState.COMPLETE)

    def _do_takeoff(self):
        """
        Phase 4: TAKEOFF — H 点起飞至巡航高度 5m.

        从 H 位置 (0, 0) 垂直上升到 -5.0m (NED z=down, 负值=向上).
        使用 takeoff_tolerance (0.3m) 作为到达判断.
        """
        tx, ty, tz = self.WAYPOINTS[MissionState.TAKEOFF]
        self._set_target(tx, ty, tz)

        if self._diag_counter >= 100:
            self._diag_counter = 0
            self.get_logger().info(
                f'[TAKEOFF] → ({tx:.1f},{ty:.1f},{tz:.1f}) '
                f'pos=({self._pos[0]:.1f},{self._pos[1]:.1f},{self._pos[2]:.2f}) '
                f'dist={self._dist(tx,ty,tz):.2f}m')

        if self._reached(tx, ty, tz, self.takeoff_tolerance):
            self.get_logger().info(
                f'到达 {self.takeoff_height}m, 飞向投放区中心...')
            self._transition(MissionState.TRANSIT_DROP)

    def _do_transit(self):
        """
        通用航点穿越 — 到达当前航点后自动前进到 NEXT_STATE 定义的下一状态.

        用于所有非悬停航点: TRANSIT_DROP, ASCEND_C1, ASCEND_C2,
        TRANSIT_RECON, RECON_C1~C5.

        setpoint 设为当前状态对应的 WAYPOINTS 坐标.
        PX4 位置控制器自动规划速度曲线飞向目标.
        到达 (距离 < waypoint_tolerance) 后自动转移.

        超时保护: waypoint_timeout (120s) 后跳过当前航点继续.
        连续超时达到 max_error_count (5 次) 后强制降落.
        """
        tx, ty, tz = self.WAYPOINTS.get(self._state, (0.0, 0.0, 0.0))
        self._set_target(tx, ty, tz)

        if self._diag_counter >= 100:
            self._diag_counter = 0
            self.get_logger().info(
                f'[TRANSIT] {self._state.name} '
                f'→ ({tx:.1f},{ty:.1f},{tz:.1f}) '
                f'dist={self._dist(tx,ty,tz):.2f}m')

        if self._reached(tx, ty, tz):
            next_state = self.NEXT_STATE.get(self._state)
            if next_state:
                self.get_logger().info(
                    f'{self._state.name} 到达 → {next_state.name}')
                self._transition(next_state)

        # 超时: 跳过当前航点
        if self._elapsed_in_state() > self.waypoint_timeout:
            self.get_logger().error(
                f'航点 {self._state.name} 超时 '
                f'({self.waypoint_timeout}s)! dist='
                f'{self._dist(tx,ty,tz):.2f}m')
            self._error_count += 1
            if self._error_count > self.max_error_count:
                self.get_logger().error('错误次数过多, 中止任务!')
                self._transition(MissionState.LAND)
            else:
                next_state = self.NEXT_STATE.get(self._state)
                if next_state:
                    self.get_logger().warn(
                        f'跳过 {self._state.name} → {next_state.name}')
                    self._transition(next_state)

    def _do_hover(self):
        """
        投放区圆筒降高悬停 — 到达后在圆筒正上方悬停指定时长.

        用于 HOVER_C1 (筒1, 15cm) 和 HOVER_C2 (筒2, 20cm).
        悬停时长: HOVER_DURATION 中定义 (默认 5s).
        悬停高度: WAYPOINTS 中定义 (z=-2.0, 即离地 2m).
        悬停完成后自动回升 (NEXT_STATE 指向 ASCEND_C1/C2).
        """
        tx, ty, tz = self.WAYPOINTS.get(self._state, (0.0, 0.0, 0.0))
        self._set_target(tx, ty, tz)

        elapsed = self._elapsed_in_state()
        duration = self.HOVER_DURATION.get(self._state, self.hover_duration)
        cyl_num, cyl_diam = self.CYL_LABELS.get(self._state, ('?', '?'))

        if self._diag_counter >= 100:
            self._diag_counter = 0
            self.get_logger().info(
                f'[HOVER] 投放筒{cyl_num} ({cyl_diam}) '
                f'elapsed={elapsed:.0f}s / {duration:.0f}s '
                f'dist={self._dist(tx,ty,tz):.2f}m '
                f'z={self._pos[2]:.2f}m')

        # 到达航点 + 悬停时间到 → 前进 (回升)
        if elapsed >= duration and self._reached(tx, ty, tz):
            self.get_logger().info(
                f'投放筒 {cyl_num} ({cyl_diam}) 悬停完成 ({duration}s) → 回升!')
            next_state = self.NEXT_STATE.get(self._state)
            if next_state:
                self._transition(next_state)

        if elapsed > self.waypoint_timeout:
            self.get_logger().error(f'悬停 {self._state.name} 超时!')
            next_state = self.NEXT_STATE.get(self._state)
            if next_state:
                self._transition(next_state)

    def _do_return(self):
        """
        Phase 16: RETURN — 返回 H 起飞点 (0, 0, -5.0).

        从侦察区飞回 H 位置, 保持 5m 高度.
        到达后自动切入 LAND 状态.
        """
        tx, ty, tz = self.WAYPOINTS[MissionState.RETURN]
        self._set_target(tx, ty, tz)

        if self._diag_counter >= 100:
            self._diag_counter = 0
            self.get_logger().info(
                f'[RETURN] dist={self._dist(tx,ty,tz):.2f}m')

        if self._reached(tx, ty, tz):
            self.get_logger().info('已返回 H (5m), 开始降落...')
            self._transition(MissionState.LAND)

    def _do_land(self):
        """
        Phase 17: LAND — 降落至 H 点并上锁.

        策略:
          1. setpoint 锁定 H 点 (0, 0), z=0 (地面), 无人机在 Offboard 下下降
          2. 当 z > -0.2 (离地 <20cm): 停止 OffboardControlMode.position
             → PX4 检测到失去位置控制 → 自动退出 Offboard (约 0.5s)
             → 无人机进入安全模式 (通常为 LOITER)
          3. 退出 Offboard 后持续发送 force_disarm (magic number 21196)
             → 绕过 PX4 上锁前检查 → 螺旋桨立即停止

        为什么这样设计:
          - PX4 不允许在 Offboard 模式下上锁 (安全机制)
          - 通过停止 OffboardControlMode 让 PX4 自行退出 Offboard
            比发送 SET_MODE 指令更可靠
          - force_disarm (param2=21196) 绕过所有检查确保螺旋桨停止
        """
        # 始终定位在 H 点正上方
        self._set_target(0.0, 0.0, 0.0)

        if not self._land_requested:
            # 阶段 1: 仍在下降
            if self._pos[2] < -0.2:
                if self._diag_counter >= 100:
                    self._diag_counter = 0
                    self.get_logger().info(
                        f'[LAND] 下降至 H 点 z={self._pos[2]:.2f}m')
                return

            # 阶段 2: 已接近地面, 触发着陆
            self._land_requested = True
            self._stop_offboard = True  # 停止 Offboard 位置控制
            self.get_logger().info(
                f'触地 (z={self._pos[2]:.2f}m), 停止 Offboard 并上锁...')

        # 阶段 3: 持续发送强制上锁直到成功
        if self._armed:
            self._disarm_count += 1
            self._force_disarm()  # 每 tick (0.05s) 发送一次
            if self._disarm_count % 50 == 1:  # 约每 2.5s 日志一次
                self.get_logger().info(
                    f'等待上锁... (count={self._disarm_count}, '
                    f'nav_state={self._nav_state}, z={self._pos[2]:.2f}m)')
        else:
            # 上锁成功: _armed 变为 False
            self.get_logger().info(
                f'着陆完成! 螺旋桨已停止 (z={self._pos[2]:.2f}m)')
            self._transition(MissionState.COMPLETE)

        # 超时: 10s 后强制结束 (即使 _armed 检查未通过)
        if self._elapsed_in_state() > 10.0:
            self.get_logger().error('上锁超时, 强制停止.')
            self._transition(MissionState.COMPLETE)


# ==========================================================================
# ROS2 节点入口 — 由 setup.py entry_points 注册为 console_script
#
# 启动方式:
#   ros2 run px4_control_dds mission_control_dds_node
#   ros2 launch px4_control_dds mission_control.launch.py (推荐, 加载 YAML)
# ==========================================================================
def main(args=None):
    """ROS2 节点入口函数."""
    rclpy.init(args=args)

    node = MissionController()

    try:
        # rclpy.spin() 阻塞运行, 内部驱动定时器和回调
        # 控制循环以 20Hz 在定时器中执行
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C → 尝试安全降落
        node.get_logger().info('Ctrl+C — 尝试 AUTO.LAND...')
        try:
            node._request_land()
            time.sleep(0.5)
            node._disarm()
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
