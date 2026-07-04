#!/usr/bin/env bash
# ==============================================================================
# PX4+ROS2+Gazebo — 四旋翼圆筒侦察任务 (micro-XRCE-DDS 版)
#
# 架构: PX4 ←XRCE(UDP 8888)→ MicroXRCEAgent ←DDS→ 控制器
#
# 启动顺序:
#   1. MicroXRCEAgent  (PX4 ↔ ROS2 桥梁)
#   2. PX4 + Gazebo    (直接启动, 不通过 gnome-terminal)
#   3. Mission Controller (DDS 直连 PX4)
# ==============================================================================
set -eo pipefail

PROJ="$(cd "$(dirname "$0")/../../.." && pwd)"

# PX4: 优先跟随项目软链接，找不到则回退到 $HOME
if [ -L "$PROJ/PX4-Autopilot" ]; then
    PX4_DIR="$(readlink -f "$PROJ/PX4-Autopilot")"
elif [ -d "$HOME/PX4-Autopilot" ]; then
    PX4_DIR="$HOME/PX4-Autopilot"
else
    echo "ERROR: PX4-Autopilot not found. Run setup_project.sh first."
    exit 1
fi
PX4_BLD="$PX4_DIR/build/px4_sitl_default"
LOG="/tmp/px4_dds_mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG"

log()  { echo -e "\033[0;32m[INFO]\033[0m  $*"; }
log_step() { echo -e "\n\033[0;34m==== $* ====\033[0m"; }
trap 'log "cleanup..."; kill %1 %2 %3 2>/dev/null; pkill -f "gz sim" 2>/dev/null; pkill -f "bin/px4" 2>/dev/null' EXIT

# --- 环境: 自动检测 ROS2 发行版 ---
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
else
    echo "ERROR: ROS2 not found in /opt/ros/"
    exit 1
fi

# 确保 DDS 工作空间已编译
if [ ! -f "$PROJ/install/setup.bash" ]; then
    log "首次编译 px4_msgs + px4_control_dds..."
    cd "$PROJ"
    colcon build --packages-select px4_msgs px4_control_dds
fi
source "$PROJ/install/setup.bash"

# ====================================================================
# Step 0: 清理旧进程
# ====================================================================
log_step "0/4 清理旧进程"
pkill -f "MicroXRCEAgent" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "bin/px4" 2>/dev/null || true
sleep 3
log "清理完成"

# ====================================================================
# Step 1: MicroXRCEAgent (后台直接启动, 不开新终端)
# ====================================================================
log_step "1/4 启动 MicroXRCEAgent"
MicroXRCEAgent udp4 -p 8888 > "$LOG/agent.log" 2>&1 &
echo $! > "$LOG/agent.pid"
sleep 2
if ss -tuln 2>/dev/null | grep -q 8888; then
    log "MicroXRCEAgent ready (UDP 8888)"
else
    log "WARNING: MicroXRCEAgent may not have started"
fi

# ====================================================================
# Step 2: PX4 SITL + Gazebo (直接后台启动, 不通过 gnome-terminal)
# ====================================================================
log_step "2/4 启动 PX4 SITL + Gazebo"

cd "$PX4_BLD"
export PX4_SIM_MODEL="gz_x500"
export PX4_GZ_WORLD="drone_field"
export PX4_GZ_MODEL_POSE="10,4,0.3,0,0,0"
export GZ_IP="127.0.0.1"
export ROS_DOMAIN_ID=0

./bin/px4 -i 0 > "$LOG/px4.log" 2>&1 &
PX4_PID=$!
echo $PX4_PID > "$LOG/px4.pid"
log "PX4 PID: $PX4_PID"

# 等 Gazebo world 就绪
for i in $(seq 1 60); do
    kill -0 "$PX4_PID" 2>/dev/null || { log "ERROR: PX4 crashed early"; tail -40 "$LOG/px4.log"; exit 1; }
    if gz topic -l 2>/dev/null | grep -q "/world/drone_field/clock"; then
        log "Gazebo world ready ($((i))s)"
        break
    fi
    sleep 2
done

# 等 DDS 话题
log "Waiting for DDS topics..."
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        log "DDS topics ready ($((i))s)"
        break
    fi
    sleep 1
done

# ====================================================================
# Step 3: QGroundControl (可选)
# ====================================================================
log_step "3/4 QGroundControl (可选)"
if command -v qgroundcontrol &>/dev/null; then
    qgroundcontrol > "$LOG/qgc.log" 2>&1 &
    echo $! > "$LOG/qgc.pid"
    log "QGC started (PID $(cat $LOG/qgc.pid))"
elif [ -f "$HOME/QGroundControl-x86_64.AppImage" ]; then
    "$HOME/QGroundControl-x86_64.AppImage" > "$LOG/qgc.log" 2>&1 &
    echo $! > "$LOG/qgc.pid"
    log "QGC started (PID $(cat $LOG/qgc.pid))"
else
    log "QGC not found, skipping"
fi

# ====================================================================
# Step 4: 任务控制器 (当前终端运行, 可看实时日志)
# ====================================================================
log_step "4/4 启动任务控制器"
log "=========================================="
log " 全部启动完成"
log "   MicroXRCEAgent: UDP 8888"
log "   Gazebo:         drone_field 世界"
log "   PX4:            gz_x500 @ H 位置 (10,4)"
log "   Controller:     v2 — 投放区2筒 + 侦察区5筒"
log " 日志: $LOG"
log " Ctrl+C 停止全部"
log "=========================================="

ros2 launch px4_control_dds mission_control.launch.py 2>&1 | tee "$LOG/mission.log"
