#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
GAZEBO_SETUP="${GAZEBO_SETUP:-/usr/share/gazebo/setup.sh}"
if [[ -z "${LIVOX_SETUP:-}" ]]; then
  if [[ -n "${LIVOX_WS:-}" ]]; then
    LIVOX_SETUP="${LIVOX_WS}/install/setup.bash"
  else
    for candidate in "${ROOT_DIR}/third_party/ws_livox" "${ROOT_DIR}/../livox/ws_livox"; do
      if [[ -f "${candidate}/install/setup.bash" ]]; then
        LIVOX_WS="${candidate}"
        LIVOX_SETUP="${candidate}/install/setup.bash"
        break
      fi
    done
  fi
fi
LIVOX_SETUP="${LIVOX_SETUP:-}"

GUI="${GUI:-true}"
RVIZ="${RVIZ:-true}"
WORLD_FILE="${WORLD_FILE:-mid360_fast_lio_world.world}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-mid360_fast_lio_mapping.rviz}"
FAST_LIO_MODE="${FAST_LIO_MODE:-stub}"
BUILD_FAST_LIO="${BUILD_FAST_LIO:-auto}"
if [[ "${BUILD_FAST_LIO}" == "auto" ]]; then
  if [[ "${FAST_LIO_MODE}" == "real" ]]; then
    BUILD_FAST_LIO="true"
  else
    BUILD_FAST_LIO="false"
  fi
fi
ENABLE_MAPPING_DRIVE="${ENABLE_MAPPING_DRIVE:-false}"
ENABLE_NAVIGATION="${ENABLE_NAVIGATION:-true}"
NAVIGATION_BACKEND="${NAVIGATION_BACKEND:-nav2}"
START_X="${START_X:--10.0}"
START_Y="${START_Y:--10.0}"
if [[ -z "${START_YAW+x}" ]]; then
  START_YAW="$(python3 - "${START_X}" "${START_Y}" <<'PY'
import math
import sys

x = float(sys.argv[1])
y = float(sys.argv[2])
if abs(x) < 1e-9 and abs(y) < 1e-9:
    yaw = 0.0
else:
    yaw = math.atan2(-y, -x)
print(f'{yaw:.6f}')
PY
)"
fi
GOAL_X="${GOAL_X:-10.0}"
GOAL_Y="${GOAL_Y:-10.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
WAYPOINTS="${WAYPOINTS:-[]}"
SKIP_BUILD="${SKIP_BUILD:-false}"
ALLOW_MULTIPLE_LAUNCHES="${ALLOW_MULTIPLE_LAUNCHES:-false}"
AUTO_CLEAN_OLD_PROCESSES="${AUTO_CLEAN_OLD_PROCESSES:-true}"
CAR_LAUNCH_PATTERN="ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py"

source_setup() {
  set +u
  source "$1"
  set -u
}

ensure_symlink() {
  local link_path="$1"
  local target_path="$2"
  if [[ ! -e "${link_path}" && -e "${target_path}" ]]; then
    ln -s "${target_path}" "${link_path}"
  fi
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS 环境文件: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${WS_DIR}" ]]; then
  echo "找不到 ROS2 工作区: ${WS_DIR}" >&2
  exit 1
fi

source_setup "${ROS_SETUP}"
if [[ -f "${GAZEBO_SETUP}" ]]; then
  source_setup "${GAZEBO_SETUP}"
fi

if [[ "${FAST_LIO_MODE}" == "real" || "${BUILD_FAST_LIO}" == "true" ]]; then
  if [[ ! -f "${LIVOX_SETUP}" ]]; then
    echo "找不到 livox_ros_driver2 依赖环境: ${LIVOX_SETUP}" >&2
    echo "FAST_LIO_MODE=real 或 BUILD_FAST_LIO=true 时需要 Livox 工作区。" >&2
    echo "可以把 Livox 工作区放到 ./third_party/ws_livox，或通过 LIVOX_WS=/path/to/ws_livox 指定。" >&2
    echo "如果只是运行 Gazebo 仿真演示，请使用默认 FAST_LIO_MODE=stub，不需要 Livox。" >&2
    exit 1
  fi
  source_setup "${LIVOX_SETUP}"
elif [[ -f "${LIVOX_SETUP}" ]]; then
  source_setup "${LIVOX_SETUP}"
else
  echo "未找到 livox_ros_driver2 环境，当前 FAST_LIO_MODE=${FAST_LIO_MODE}，将使用 stub 仿真模式跳过 Livox/fast_lio 构建。"
  echo "需要真实 FAST-LIO 时请运行: FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car.sh"
fi

if [[ "${ALLOW_MULTIPLE_LAUNCHES}" != "true" && "${AUTO_CLEAN_OLD_PROCESSES}" == "true" ]]; then
  echo "启动前清理旧的小车仿真/Gazebo/RViz 进程，避免 Gazebo 端口冲突..."
  "${ROOT_DIR}/stop_car_arm.sh" || true
fi

if [[ "${ALLOW_MULTIPLE_LAUNCHES}" != "true" ]]; then
  EXISTING_LAUNCHES="$(pgrep -af "${CAR_LAUNCH_PATTERN}" || true)"
  if [[ -n "${EXISTING_LAUNCHES}" ]]; then
    echo "检测到已有小车仿真正在运行，请先关闭旧窗口/旧终端里的仿真。" >&2
    echo "${EXISTING_LAUNCHES}" >&2
    echo "如果旧终端找不到了，可以执行：" >&2
    echo "  ./stop_car_arm.sh" >&2
    echo "确认清干净后再运行 ./start_car.sh。" >&2
    exit 1
  fi
fi

REBOT_SRC="${WS_DIR}/src/_external_rebot/reBotArmController_ROS2/src"
if [[ -d "${REBOT_SRC}" ]]; then
  ensure_symlink "${WS_DIR}/src/rebotarm_msgs" "_external_rebot/reBotArmController_ROS2/src/rebotarm_msgs"
  ensure_symlink "${WS_DIR}/src/rebotarmcontroller" "_external_rebot/reBotArmController_ROS2/src/rebotarmcontroller"
  ensure_symlink "${WS_DIR}/src/rebotarm_bringup" "_external_rebot/reBotArmController_ROS2/src/rebotarm_bringup"
  ensure_symlink "${WS_DIR}/src/rebotarm_moveit_config" "_external_rebot/reBotArmController_ROS2/src/rebotarm_moveit_config"
  ensure_symlink "${WS_DIR}/src/rebotarm_moveit_demos" "_external_rebot/reBotArmController_ROS2/src/rebotarm_moveit_demos"
fi

if [[ "${ENABLE_NAVIGATION}" == "true" && "${NAVIGATION_BACKEND}" == "nav2" ]]; then
  if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    echo "当前系统没有安装 ROS2 Nav2，但 NAVIGATION_BACKEND=nav2。" >&2
    echo "请先安装：" >&2
    echo "  sudo apt update" >&2
    echo "  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
    echo "安装后重新运行 ./start_3d_fastlio.sh" >&2
    echo "临时退回旧算法可用：NAVIGATION_BACKEND=custom ./start_3d_fastlio.sh" >&2
    exit 1
  fi
fi

cd "${WS_DIR}"

if [[ "${SKIP_BUILD}" != "true" ]]; then
  build_packages=(
    rebotarm_msgs
    rebotarmcontroller
    rebotarm_bringup
    rebotarm_moveit_config
    version_car_sim
  )
  if [[ "${BUILD_FAST_LIO}" == "true" ]]; then
    build_packages=(fast_lio "${build_packages[@]}")
  fi
  echo "开始构建 ros2_ws: ${build_packages[*]}"
  colcon build --symlink-install --packages-select "${build_packages[@]}"
fi

source_setup "${WS_DIR}/install/setup.bash"

echo "启动 MID360 + FAST-LIO 三维建图..."
launch_args=(
  "gui:=${GUI}"
  "rviz:=${RVIZ}"
  "world_file:=${WORLD_FILE}"
  "rviz_config_file:=${RVIZ_CONFIG_FILE}"
  "fast_lio_mode:=${FAST_LIO_MODE}"
  "enable_mapping_drive:=${ENABLE_MAPPING_DRIVE}"
  "enable_navigation:=${ENABLE_NAVIGATION}"
  "navigation_backend:=${NAVIGATION_BACKEND}"
  "start_x:=${START_X}"
  "start_y:=${START_Y}"
  "start_yaw:=${START_YAW}"
  "goal_x:=${GOAL_X}"
  "goal_y:=${GOAL_Y}"
  "goal_yaw:=${GOAL_YAW}"
)

if [[ "${WAYPOINTS}" != "[]" ]]; then
  launch_args+=("waypoints:=${WAYPOINTS}")
fi

ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py "${launch_args[@]}"
