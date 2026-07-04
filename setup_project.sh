#!/usr/bin/env bash
# ==============================================================================
# Task_flight_ws 项目初始化脚本
# 用法: ./setup_project.sh [--px4-dir /path/to/PX4-Autopilot]
#
# 自动完成:
#   1. 检测工作空间路径
#   2. 检测 PX4-Autopilot 路径并创建/修复软链接
#   3. 用绝对路径重新生成 drone_field.sdf 中的 mesh/texture 引用
#   4. 将 drone_field.sdf 部署到 PX4 worlds 目录
#   5. 检测 ROS2 发行版并修复启动脚本
#   6. 编译 px4_msgs + px4_control_dds
#   7. 生成 .vscode/settings.json
#   8. 检查 MicroXRCEAgent 是否已安装
# ==============================================================================
set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
step()  { echo -e "\n${BLUE}==== $* ====${NC}"; }

# --- 检测项目根目录 (脚本所在目录) ---
PROJ_ROOT="$(cd "$(dirname "$0")" && pwd)"
info "项目根目录: $PROJ_ROOT"

# ====================================================================
# 1. 解析参数 + 检测 PX4-Autopilot
# ====================================================================
step "1/8 检测 PX4-Autopilot"

PX4_DIR=""
if [ "$1" = "--px4-dir" ] && [ -n "$2" ]; then
    PX4_DIR="$(realpath "$2")"
    shift 2
fi

if [ -z "$PX4_DIR" ]; then
    # 自动检测: 先看软链接, 再搜常见位置
    if [ -L "$PROJ_ROOT/PX4-Autopilot" ]; then
        PX4_DIR="$(readlink -f "$PROJ_ROOT/PX4-Autopilot")"
    fi
    if [ ! -d "$PX4_DIR/Tools/simulation/gz/worlds" ]; then
        for candidate in \
            "$HOME/PX4-Autopilot" \
            "$PROJ_ROOT/../PX4-Autopilot" \
            "$HOME/px4/PX4-Autopilot"; do
            if [ -d "$candidate/Tools/simulation/gz/worlds" ]; then
                PX4_DIR="$candidate"
                break
            fi
        done
    fi
fi

if [ -z "$PX4_DIR" ] || [ ! -d "$PX4_DIR/Tools/simulation/gz/worlds" ]; then
    warn "未找到 PX4-Autopilot"
    warn "请指定: ./setup_project.sh --px4-dir /path/to/PX4-Autopilot"
    warn "跳过 PX4 相关配置 (SDF 部署、软链接)"
    PX4_DIR=""
else
    info "PX4-Autopilot: $PX4_DIR"
fi

# ====================================================================
# 2. 创建/修复 PX4-Autopilot 软链接
# ====================================================================
step "2/8 修复 PX4-Autopilot 软链接"

if [ -n "$PX4_DIR" ]; then
    if [ -L "$PROJ_ROOT/PX4-Autopilot" ]; then
        CURRENT="$(readlink "$PROJ_ROOT/PX4-Autopilot")"
        if [ "$(readlink -f "$PROJ_ROOT/PX4-Autopilot")" != "$PX4_DIR" ]; then
            rm "$PROJ_ROOT/PX4-Autopilot"
            ln -s "$PX4_DIR" "$PROJ_ROOT/PX4-Autopilot"
            info "软链接已更新: -> $PX4_DIR"
        else
            info "软链接正确: -> $PX4_DIR"
        fi
    elif [ ! -e "$PROJ_ROOT/PX4-Autopilot" ]; then
        ln -s "$PX4_DIR" "$PROJ_ROOT/PX4-Autopilot"
        info "软链接已创建: -> $PX4_DIR"
    else
        warn "PX4-Autopilot 存在但不是软链接，跳过"
    fi
else
    warn "跳过 (PX4 未找到)"
fi

# ====================================================================
# 3. 修复 drone_field.sdf 中的 mesh/texture 路径
# ====================================================================
step "3/8 修复 drone_field.sdf 路径"

SDF_FILE="$PROJ_ROOT/drone_field.sdf"
MESH_DIR="$PROJ_ROOT/drone_field_meshes"

if [ ! -f "$SDF_FILE" ]; then
    error "drone_field.sdf 未找到!"
fi

# 替换所有旧的 /home/<user>/Task_flight_ws/ 为当前项目路径
OLD_COUNT=$(grep -c "file:///home/" "$SDF_FILE" 2>/dev/null || echo 0)
sed -i "s|file:///home/[^/]*/Task_flight_ws/drone_field_meshes/|file://${PROJ_ROOT}/drone_field_meshes/|g" "$SDF_FILE"
NEW_COUNT=$(grep -c "file:///home/" "$SDF_FILE" 2>/dev/null || echo 0)

if [ "$OLD_COUNT" -gt 0 ]; then
    info "已替换 $OLD_COUNT 处旧路径 -> file://${PROJ_ROOT}/drone_field_meshes/"
elif [ "$NEW_COUNT" -eq 0 ]; then
    # 检查是否已经是当前路径
    if grep -q "file://${PROJ_ROOT}/drone_field_meshes/" "$SDF_FILE"; then
        info "SDF 路径已正确，无需修改"
    fi
else
    warn "仍有 $NEW_COUNT 处未识别的 /home/ 路径，请手动检查"
fi

# ====================================================================
# 4. 部署 drone_field.sdf 到 PX4 worlds 目录
# ====================================================================
step "4/8 部署 drone_field.sdf 到 PX4"

if [ -n "$PX4_DIR" ]; then
    PX4_WORLD="$PX4_DIR/Tools/simulation/gz/worlds/drone_field.sdf"
    cp "$SDF_FILE" "$PX4_WORLD"
    info "已部署: $PX4_WORLD"
else
    warn "跳过 (PX4 未找到, PX4_GZ_WORLD=drone_field 将无法加载此世界)"
fi

# ====================================================================
# 5. 检测 ROS2 发行版 + 修复脚本
# ====================================================================
step "5/8 检测 ROS2 发行版"

ROS2_DISTRO=""
for d in humble jazzy iron rolling; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
        ROS2_DISTRO="$d"
        break
    fi
done

if [ -z "$ROS2_DISTRO" ]; then
    error "未检测到 ROS2 (检查了 /opt/ros/{humble,jazzy,iron,rolling})"
fi
info "ROS2: $ROS2_DISTRO"
source "/opt/ros/$ROS2_DISTRO/setup.bash"

# 修复 start_mission_dds.sh
START_SCRIPT="$PROJ_ROOT/src/px4_control_dds/scripts/start_mission_dds.sh"
if [ -f "$START_SCRIPT" ]; then
    sed -i "s|source /opt/ros/[a-z]*/setup.bash|source /opt/ros/$ROS2_DISTRO/setup.bash|" "$START_SCRIPT"
    info "start_mission_dds.sh -> $ROS2_DISTRO"
fi

# 修复 test_mission.sh
TEST_SCRIPT="$PROJ_ROOT/test_mission.sh"
if [ -f "$TEST_SCRIPT" ]; then
    sed -i "s|source /opt/ros/[a-z]*/setup.bash|source /opt/ros/$ROS2_DISTRO/setup.bash|" "$TEST_SCRIPT"
    info "test_mission.sh -> $ROS2_DISTRO"
fi

# ====================================================================
# 6. 编译工作空间
# ====================================================================
step "6/8 编译工作空间"

cd "$PROJ_ROOT"
colcon build --packages-select px4_msgs px4_control_dds --symlink-install
info "编译完成"

# ====================================================================
# 7. 生成 .vscode/settings.json
# ====================================================================
step "7/8 生成 VS Code 配置"

PYTHON_VERSION=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")
VSCODE_DIR="$PROJ_ROOT/.vscode"
mkdir -p "$VSCODE_DIR"

cat > "$VSCODE_DIR/settings.json" << EOF
{
    "ROS2.distro": "$ROS2_DISTRO",
    "python.autoComplete.extraPaths": [
        "$PROJ_ROOT/install/px4_control_dds/lib/$PYTHON_VERSION/site-packages",
        "$PROJ_ROOT/install/px4_msgs/local/lib/$PYTHON_VERSION/dist-packages",
        "/opt/ros/$ROS2_DISTRO/lib/$PYTHON_VERSION/site-packages",
        "/opt/ros/$ROS2_DISTRO/local/lib/$PYTHON_VERSION/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "$PROJ_ROOT/install/px4_control_dds/lib/$PYTHON_VERSION/site-packages",
        "$PROJ_ROOT/install/px4_msgs/local/lib/$PYTHON_VERSION/dist-packages",
        "/opt/ros/$ROS2_DISTRO/lib/$PYTHON_VERSION/site-packages",
        "/opt/ros/$ROS2_DISTRO/local/lib/$PYTHON_VERSION/dist-packages"
    ]
}
EOF
info ".vscode/settings.json 已生成 (Python: $PYTHON_VERSION)"

# ====================================================================
# 8. 检查 MicroXRCEAgent
# ====================================================================
step "8/8 检查 MicroXRCEAgent"

if command -v MicroXRCEAgent &>/dev/null; then
    info "MicroXRCEAgent: $(which MicroXRCEAgent)"
else
    warn "MicroXRCEAgent 未安装!"
    echo ""
    echo "  ┌─────────────────────────────────────────────────────────┐"
    echo "  │  MicroXRCEAgent 安装步骤:                               │"
    echo "  │                                                         │"
    echo "  │  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git │"
    echo "  │  cd Micro-XRCE-DDS-Agent                                │"
    echo "  │  mkdir build && cd build                                │"
    echo "  │  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \\          │"
    echo "  │           -DUAGENT_SUPERBUILD=ON                        │"
    echo "  │  make -j\$(nproc)                                       │"
    echo "  │  sudo make install && sudo ldconfig                     │"
    echo "  └─────────────────────────────────────────────────────────┘"
    echo ""
fi

# ====================================================================
# 完成
# ====================================================================
echo ""
echo "┌─────────────────────────────────────────────────────────────┐"
echo "│              项目初始化完成!                                │"
echo "├─────────────────────────────────────────────────────────────┤"
echo "│  项目:     $PROJ_ROOT"
if [ -n "$PX4_DIR" ]; then
echo "│  PX4:      $PX4_DIR"
else
echo "│  PX4:      (未配置)"
fi
echo "│  ROS2:     $ROS2_DISTRO"
echo "│  Python:   $PYTHON_VERSION"
echo "├─────────────────────────────────────────────────────────────┤"
echo "│  启动仿真:                                                  │"
echo "│    ./src/px4_control_dds/scripts/start_mission_dds.sh       │"
echo "│                                                             │"
echo "│  或使用测试脚本:                                            │"
echo "│    ./test_mission.sh                                        │"
echo "└─────────────────────────────────────────────────────────────┘"
