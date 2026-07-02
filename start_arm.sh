#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

RVIZ="${RVIZ:-true}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
DEMO_MODE="${DEMO_MODE:-circle_points}"
START_TASK="${START_TASK:-true}"
SKIP_BUILD="${SKIP_BUILD:-false}"

LEAK_X="${LEAK_X:-0.50}"
LEAK_Y="${LEAK_Y:-0.0}"
LEAK_Z="${LEAK_Z:-0.28}"
LEAK_STANDOFF="${LEAK_STANDOFF:-0.12}"
SPRAY_DURATION_SEC="${SPRAY_DURATION_SEC:-4.0}"

START_DELAY_SEC="${START_DELAY_SEC:-8.0}"
FRAME_ID="${FRAME_ID:-rebot_base_link}"
CENTER_X="${CENTER_X:-0.35}"
CENTER_Y="${CENTER_Y:-0.0}"
CENTER_Z="${CENTER_Z:-0.05}"
RADIUS="${RADIUS:-0.25}"
NUM_POINTS="${NUM_POINTS:-6}"
LOOP_COUNT="${LOOP_COUNT:-0}"
HOLD_SEC="${HOLD_SEC:-1.0}"
TIP_LINK="${TIP_LINK:-spray_tip_link}"

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

require_ros_package() {
  local package_name="$1"
  local install_hint="$2"
  if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    echo "缺少 ROS2 包: ${package_name}" >&2
    echo "${install_hint}" >&2
    exit 1
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

REBOT_SRC="${WS_DIR}/src/_external_rebot/reBotArmController_ROS2/src"
if [[ -d "${REBOT_SRC}" ]]; then
  ensure_symlink "${WS_DIR}/src/rebotarm_msgs" "_external_rebot/reBotArmController_ROS2/src/rebotarm_msgs"
  ensure_symlink "${WS_DIR}/src/rebotarmcontroller" "_external_rebot/reBotArmController_ROS2/src/rebotarmcontroller"
  ensure_symlink "${WS_DIR}/src/rebotarm_bringup" "_external_rebot/reBotArmController_ROS2/src/rebotarm_bringup"
  ensure_symlink "${WS_DIR}/src/rebotarm_moveit_config" "_external_rebot/reBotArmController_ROS2/src/rebotarm_moveit_config"
  ensure_symlink "${WS_DIR}/src/rebotarm_moveit_demos" "_external_rebot/reBotArmController_ROS2/src/rebotarm_moveit_demos"
fi

for package_dir in \
  rebotarm_msgs \
  rebotarmcontroller \
  rebotarm_bringup \
  rebotarm_moveit_config \
  rebotarm_moveit_demos \
  rebot_gas_spray_demo; do
  if [[ ! -e "${WS_DIR}/src/${package_dir}" ]]; then
    echo "找不到 ${package_dir}。" >&2
    echo "当前脚本只启动独立 reBot B601-DM 机械臂仿真，请确认该包已在 ${WS_DIR}/src 下。" >&2
    exit 1
  fi
done

source_setup "${ROS_SETUP}"

require_ros_package "moveit_ros_move_group" \
  "请先安装: sudo apt install ros-humble-moveit"
require_ros_package "controller_manager" \
  "请先安装: sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers"
require_ros_package "xacro" \
  "请先安装: sudo apt install ros-humble-xacro"

cd "${WS_DIR}"

if [[ "${SKIP_BUILD}" != "true" ]]; then
  echo "开始构建 ros2_ws: reBot B601-DM 独立机械臂仿真..."
  colcon build --symlink-install --packages-select \
    rebotarm_msgs \
    rebotarmcontroller \
    rebotarm_bringup \
    rebotarm_moveit_config \
    rebotarm_moveit_demos \
    rebot_gas_spray_demo
fi

source_setup "${WS_DIR}/install/setup.bash"

case "${DEMO_MODE}" in
  circle|circle_points|points)
    echo "启动独立 reBot Arm B601-DM 圆形 6 点位运动仿真..."
    ros2 launch rebot_gas_spray_demo rebot_dm_circle_points_demo.launch.py \
      "model:=dm" \
      "rviz:=${RVIZ}" \
      "use_sim_time:=${USE_SIM_TIME}" \
      "start_task:=${START_TASK}" \
      "start_delay_sec:=${START_DELAY_SEC}" \
      "frame_id:=${FRAME_ID}" \
      "center_x:=${CENTER_X}" \
      "center_y:=${CENTER_Y}" \
      "center_z:=${CENTER_Z}" \
      "radius:=${RADIUS}" \
      "num_points:=${NUM_POINTS}" \
      "loop_count:=${LOOP_COUNT}" \
      "hold_sec:=${HOLD_SEC}" \
      "tip_link:=${TIP_LINK}"
    ;;
  spray|gas_spray)
    echo "启动独立 reBot Arm B601-DM 喷药/注射仿真..."
    ros2 launch rebot_gas_spray_demo rebot_dm_spray_demo.launch.py \
      "model:=dm" \
      "rviz:=${RVIZ}" \
      "use_sim_time:=${USE_SIM_TIME}" \
      "leak_x:=${LEAK_X}" \
      "leak_y:=${LEAK_Y}" \
      "leak_z:=${LEAK_Z}" \
      "leak_standoff:=${LEAK_STANDOFF}" \
      "spray_duration_sec:=${SPRAY_DURATION_SEC}" \
      "start_task:=${START_TASK}"
    ;;
  *)
    echo "未知 DEMO_MODE: ${DEMO_MODE}" >&2
    echo "可用值: circle_points, spray" >&2
    exit 1
    ;;
esac
