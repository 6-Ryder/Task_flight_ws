#!/usr/bin/env bash
# ==============================================================================
# Task_flight_ws 自动化测试脚本
# 用法: ./test_mission.sh
# 流程: 清理旧进程 → 编译(如需) → 启动仿真 → 完成后仅关 gz+QGC
# ==============================================================================
set -eo pipefail

# --- 清理旧进程 (仅清理当前用户进程，无需 sudo) ---
echo "=== 清理旧进程 ==="
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "MicroXRCE" 2>/dev/null || true
pkill -9 -f "mission_control" 2>/dev/null || true
pkill -9 -f "QGroundControl" 2>/dev/null || true
# QGC AppImage 进程名特殊，pkill 可能匹配不到，按 PID 补杀
QGC_PIDS=$(ps aux | grep -i qground | grep -v grep | awk '{print $2}')
[ -n "$QGC_PIDS" ] && kill -9 $QGC_PIDS 2>/dev/null || true
sleep 2
echo "清理完成"

# --- 编译: 自动检测 ROS2 发行版 ---
cd "$(dirname "$0")"
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
if [ ! -f install/setup.bash ]; then
    echo "=== 首次编译 ==="
    colcon build --packages-select px4_msgs px4_control_dds
fi

# --- 启动仿真 ---
echo "=== 启动仿真 ==="
exec ./src/px4_control_dds/scripts/start_mission_dds.sh
