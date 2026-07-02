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
FAST_LIO_MODE="${FAST_LIO_MODE:-stub}"
BUILD_FAST_LIO="${BUILD_FAST_LIO:-auto}"
if [[ "${BUILD_FAST_LIO}" == "auto" ]]; then
  if [[ "${FAST_LIO_MODE}" == "real" ]]; then
    BUILD_FAST_LIO="true"
  else
    BUILD_FAST_LIO="false"
  fi
fi
WORLD_FILE="${WORLD_FILE:-mid360_fast_lio_world.world}"
CAR_MODEL_FILE="${CAR_MODEL_FILE:-version_car_mid360_rebot_b601_dm.sdf}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-mid360_fast_lio_mapping.rviz}"
START_X="${START_X:--10.0}"
START_Y="${START_Y:--10.0}"
START_YAW="${START_YAW:-0.785398}"
SPAWN_CAR="${SPAWN_CAR:-auto}"

LEAK_X="${LEAK_X:-10.0}"
LEAK_Y="${LEAK_Y:-10.0}"
LEAK_Z="${LEAK_Z:-0.3}"
# 固定的小车安全半径，不作为部署时调参项。
VEHICLE_SAFETY_RADIUS="0.452548"
# 可调项：小车外壳/安全半径外侧到目标点的净距离。机械臂喷洒姿态需要更自然的工作距离。
TARGET_CLEARANCE="${TARGET_CLEARANCE:-0.80}"
WORK_DISTANCE="${WORK_DISTANCE:-${TARGET_CLEARANCE}}"
SPRAY_STANDOFF="${SPRAY_STANDOFF:-0.15}"
MIN_SPRAY_TIP_Z="${MIN_SPRAY_TIP_Z:-0.24}"
GROUND_CLEARANCE="${GROUND_CLEARANCE:-0.10}"
SPRAY_TIP_WORK_HEIGHT="${SPRAY_TIP_WORK_HEIGHT:-0.30}"
SPRAY_AIM_HEIGHT="${SPRAY_AIM_HEIGHT:-0.24}"
SPRAY_MAX_DOWNWARD_Z="${SPRAY_MAX_DOWNWARD_Z:--0.20}"
SPRAY_MIN_RANGE="${SPRAY_MIN_RANGE:-0.25}"
SPRAY_MAX_RANGE="${SPRAY_MAX_RANGE:-0.78}"
BASE_LINK_GROUND_Z="${BASE_LINK_GROUND_Z:-0.0}"
ALLOW_POSITION_ONLY_SPRAY_FALLBACK="${ALLOW_POSITION_ONLY_SPRAY_FALLBACK:-false}"
ALLOW_DIRECT_SPRAY_JOINT_FALLBACK="${ALLOW_DIRECT_SPRAY_JOINT_FALLBACK:-false}"
ARM_SPEED_MULTIPLIER="${ARM_SPEED_MULTIPLIER:-20.0}"
MIN_ARM_TRAJECTORY_DURATION="${MIN_ARM_TRAJECTORY_DURATION:-0.5}"
MAX_ARM_TRAJECTORY_DURATION="${MAX_ARM_TRAJECTORY_DURATION:-5.0}"
HOME_TO_INITIAL_DURATION="${HOME_TO_INITIAL_DURATION:-2.0}"
INITIAL_TO_TARGET_DURATION="${INITIAL_TO_TARGET_DURATION:-3.0}"
NAVIGATION_GOAL_TOLERANCE="${NAVIGATION_GOAL_TOLERANCE:-0.50}"
NAVIGATION_GOAL_HOLD_SEC="${NAVIGATION_GOAL_HOLD_SEC:-2.0}"
SIMULATION_FAST_MODE="${SIMULATION_FAST_MODE:-true}"
REAL_HARDWARE_MODE="${REAL_HARDWARE_MODE:-false}"
ARM_MAX_VELOCITY_SCALING_FACTOR="${ARM_MAX_VELOCITY_SCALING_FACTOR:-1.0}"
ARM_MAX_ACCELERATION_SCALING_FACTOR="${ARM_MAX_ACCELERATION_SCALING_FACTOR:-1.0}"
SPRAY_DURATION_SEC="${SPRAY_DURATION_SEC:-5.0}"
RETURN_HOME_AFTER_SPRAY="${RETURN_HOME_AFTER_SPRAY:-false}"
ENABLE_GAZEBO_ARM_MIRROR="${ENABLE_GAZEBO_ARM_MIRROR:-true}"
GAZEBO_ARM_MIRROR_DURATION_SEC="${GAZEBO_ARM_MIRROR_DURATION_SEC:-3.0}"
START_TASK="${START_TASK:-true}"
ARM_CONTROL_MODE="${ARM_CONTROL_MODE:-rl}"
DEFAULT_GAZEBO_RL_POLICY="${WS_DIR}/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/policy.pt"
DEFAULT_RL_POLICY="${WS_DIR}/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian/policy.pt"
if [[ -z "${RL_POLICY_PATH:-}" ]]; then
  if [[ -f "${DEFAULT_GAZEBO_RL_POLICY}" ]]; then
    RL_POLICY_PATH="${DEFAULT_GAZEBO_RL_POLICY}"
  else
    RL_POLICY_PATH="${DEFAULT_RL_POLICY}"
  fi
fi
RL_ENABLE_TEACHER_FALLBACK="${RL_ENABLE_TEACHER_FALLBACK:-true}"
RL_MAX_STEPS="${RL_MAX_STEPS:-360}"
RL_TIMEOUT_SEC="${RL_TIMEOUT_SEC:-360.0}"
RL_SUCCESS_TOLERANCE="${RL_SUCCESS_TOLERANCE:-0.06}"
RL_TF_SUCCESS_TOLERANCE="${RL_TF_SUCCESS_TOLERANCE:-0.07}"
RL_MAX_ACTION_DELTA="${RL_MAX_ACTION_DELTA:-0.04}"
RL_CONTROL_RATE_HZ="${RL_CONTROL_RATE_HZ:-8.0}"
RL_TRAJECTORY_DURATION_SEC="${RL_TRAJECTORY_DURATION_SEC:-0.32}"
RL_TRAJECTORY_MIN_DURATION_SEC="${RL_TRAJECTORY_MIN_DURATION_SEC:-0.22}"
RL_TRAJECTORY_MAX_DURATION_SEC="${RL_TRAJECTORY_MAX_DURATION_SEC:-0.55}"
RL_TRAJECTORY_NOMINAL_JOINT_SPEED="${RL_TRAJECTORY_NOMINAL_JOINT_SPEED:-0.18}"
RL_TRAJECTORY_WAYPOINTS="${RL_TRAJECTORY_WAYPOINTS:-6}"
RL_ACTION_LOW_PASS_ALPHA="${RL_ACTION_LOW_PASS_ALPHA:-0.55}"

ARM_MOUNT_X="${ARM_MOUNT_X:-0.0}"
ARM_MOUNT_Y="${ARM_MOUNT_Y:-0.0}"
ARM_MOUNT_Z="${ARM_MOUNT_Z:-0.08}"
ARM_MOUNT_ROLL="${ARM_MOUNT_ROLL:-0.0}"
ARM_MOUNT_PITCH="${ARM_MOUNT_PITCH:-0.0}"
ARM_MOUNT_YAW="${ARM_MOUNT_YAW:-0.0}"

SKIP_BUILD="${SKIP_BUILD:-false}"
ALLOW_MULTIPLE_LAUNCHES="${ALLOW_MULTIPLE_LAUNCHES:-false}"
AUTO_CLEAN_OLD_PROCESSES="${AUTO_CLEAN_OLD_PROCESSES:-true}"
CAR_ARM_LAUNCH_PATTERN="ros2 launch version_car_sim gas_leak_car_arm_demo.launch.py"

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
  echo "需要真实 FAST-LIO 时请运行: FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car_arm.sh"
fi

if [[ "${ALLOW_MULTIPLE_LAUNCHES}" != "true" && "${AUTO_CLEAN_OLD_PROCESSES}" == "true" ]]; then
  echo "启动前清理旧的小车+机械臂仿真进程，确保加载最新代码..."
  "${ROOT_DIR}/stop_car_arm.sh" || true
fi

if [[ "${ALLOW_MULTIPLE_LAUNCHES}" != "true" ]]; then
  EXISTING_LAUNCHES="$(pgrep -af "${CAR_ARM_LAUNCH_PATTERN}" || true)"
  if [[ -n "${EXISTING_LAUNCHES}" ]]; then
    echo "检测到已有小车+机械臂联合仿真正在运行，请先关闭旧窗口/旧终端里的仿真。" >&2
    echo "${EXISTING_LAUNCHES}" >&2
    echo "如果旧终端找不到了，可以执行：" >&2
    echo "  ./stop_car_arm.sh" >&2
    echo "确认清干净后再运行 ./start_car_arm.sh。" >&2
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

if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
  echo "当前系统没有安装 ROS2 Nav2，请先安装：" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
  exit 1
fi

cd "${WS_DIR}"

if [[ "${SKIP_BUILD}" != "true" ]]; then
  echo "开始构建 ros2_ws: 小车 + reBot B601-DM 联合仿真..."
  build_packages=(
    rebotarm_msgs
    rebotarmcontroller
    rebotarm_bringup
    rebotarm_moveit_config
    rebotarm_moveit_demos
    rebot_gas_spray_demo
    version_car_sim
  )
  if [[ "${BUILD_FAST_LIO}" == "true" ]]; then
    build_packages=(fast_lio "${build_packages[@]}")
  fi
  colcon build --symlink-install --packages-select "${build_packages[@]}"
fi

source_setup "${WS_DIR}/install/setup.bash"
export GAZEBO_MODEL_PATH="${WS_DIR}/install/rebotarm_bringup/share:${GAZEBO_MODEL_PATH:-}"

echo "启动 小车底盘 + MID360 + FAST-LIO + Nav2 + reBot B601-DM 机械臂 联合仿真..."
echo "泄漏点: map(${LEAK_X}, ${LEAK_Y}, ${LEAK_Z})，小车外壳到泄漏点净距离: ${WORK_DISTANCE} m"
echo "小车安全半径: ${VEHICLE_SAFETY_RADIUS} m，实际 base_link 中心停车距离 = 半径 + 净距离"
echo "机械臂控制模式: ${ARM_CONTROL_MODE}"
echo "安全强化学习策略: ${RL_POLICY_PATH}"
echo "喷头安全高度: 地面以上 >= ${MIN_SPRAY_TIP_Z} m，喷洒前距 ${SPRAY_STANDOFF} m"
echo "喷头姿态约束: tip_height=${SPRAY_TIP_WORK_HEIGHT} m, aim_height=${SPRAY_AIM_HEIGHT} m, max_downward_z=${SPRAY_MAX_DOWNWARD_Z}"
echo "机械臂速度: MoveIt vel_scale=${ARM_MAX_VELOCITY_SCALING_FACTOR}, acc_scale=${ARM_MAX_ACCELERATION_SCALING_FACTOR}; arm_speed_multiplier=${ARM_SPEED_MULTIPLIER}, min_traj_duration=${MIN_ARM_TRAJECTORY_DURATION}s, max_traj_duration=${MAX_ARM_TRAJECTORY_DURATION}s, home_to_initial=${HOME_TO_INITIAL_DURATION}s, initial_to_target=${INITIAL_TO_TARGET_DURATION}s; RL max_delta=${RL_MAX_ACTION_DELTA} rad, step_duration=${RL_TRAJECTORY_DURATION_SEC}s"
ros2 launch version_car_sim gas_leak_car_arm_demo.launch.py \
  "model:=dm" \
  "gui:=${GUI}" \
  "rviz:=${RVIZ}" \
  "fast_lio_mode:=${FAST_LIO_MODE}" \
  "world_file:=${WORLD_FILE}" \
  "car_model_file:=${CAR_MODEL_FILE}" \
  "rviz_config_file:=${RVIZ_CONFIG_FILE}" \
  "start_x:=${START_X}" \
  "start_y:=${START_Y}" \
  "start_yaw:=${START_YAW}" \
  "spawn_car:=${SPAWN_CAR}" \
  "leak_x:=${LEAK_X}" \
  "leak_y:=${LEAK_Y}" \
  "leak_z:=${LEAK_Z}" \
  "work_distance:=${WORK_DISTANCE}" \
  "vehicle_safety_radius:=${VEHICLE_SAFETY_RADIUS}" \
  "spray_standoff:=${SPRAY_STANDOFF}" \
  "min_spray_tip_z:=${MIN_SPRAY_TIP_Z}" \
  "ground_clearance:=${GROUND_CLEARANCE}" \
  "spray_tip_work_height:=${SPRAY_TIP_WORK_HEIGHT}" \
  "spray_aim_height:=${SPRAY_AIM_HEIGHT}" \
  "spray_max_downward_z:=${SPRAY_MAX_DOWNWARD_Z}" \
  "spray_min_range:=${SPRAY_MIN_RANGE}" \
  "spray_max_range:=${SPRAY_MAX_RANGE}" \
  "base_link_ground_z:=${BASE_LINK_GROUND_Z}" \
  "allow_position_only_spray_fallback:=${ALLOW_POSITION_ONLY_SPRAY_FALLBACK}" \
  "allow_direct_spray_joint_fallback:=${ALLOW_DIRECT_SPRAY_JOINT_FALLBACK}" \
  "arm_speed_multiplier:=${ARM_SPEED_MULTIPLIER}" \
  "min_arm_trajectory_duration:=${MIN_ARM_TRAJECTORY_DURATION}" \
  "max_arm_trajectory_duration:=${MAX_ARM_TRAJECTORY_DURATION}" \
  "home_to_initial_duration:=${HOME_TO_INITIAL_DURATION}" \
  "initial_to_target_duration:=${INITIAL_TO_TARGET_DURATION}" \
  "navigation_goal_tolerance:=${NAVIGATION_GOAL_TOLERANCE}" \
  "navigation_goal_hold_sec:=${NAVIGATION_GOAL_HOLD_SEC}" \
  "simulation_fast_mode:=${SIMULATION_FAST_MODE}" \
  "real_hardware_mode:=${REAL_HARDWARE_MODE}" \
  "arm_max_velocity_scaling_factor:=${ARM_MAX_VELOCITY_SCALING_FACTOR}" \
  "arm_max_acceleration_scaling_factor:=${ARM_MAX_ACCELERATION_SCALING_FACTOR}" \
  "spray_duration_sec:=${SPRAY_DURATION_SEC}" \
  "return_home_after_spray:=${RETURN_HOME_AFTER_SPRAY}" \
  "enable_gazebo_arm_mirror:=${ENABLE_GAZEBO_ARM_MIRROR}" \
  "gazebo_arm_mirror_duration_sec:=${GAZEBO_ARM_MIRROR_DURATION_SEC}" \
  "start_task:=${START_TASK}" \
  "arm_control_mode:=${ARM_CONTROL_MODE}" \
  "rl_policy_path:=${RL_POLICY_PATH}" \
  "rl_enable_teacher_fallback:=${RL_ENABLE_TEACHER_FALLBACK}" \
  "rl_max_steps:=${RL_MAX_STEPS}" \
  "rl_timeout_sec:=${RL_TIMEOUT_SEC}" \
  "rl_max_action_delta:=${RL_MAX_ACTION_DELTA}" \
  "rl_control_rate_hz:=${RL_CONTROL_RATE_HZ}" \
  "rl_success_tolerance:=${RL_SUCCESS_TOLERANCE}" \
  "rl_tf_success_tolerance:=${RL_TF_SUCCESS_TOLERANCE}" \
  "rl_trajectory_duration_sec:=${RL_TRAJECTORY_DURATION_SEC}" \
  "rl_trajectory_min_duration_sec:=${RL_TRAJECTORY_MIN_DURATION_SEC}" \
  "rl_trajectory_max_duration_sec:=${RL_TRAJECTORY_MAX_DURATION_SEC}" \
  "rl_trajectory_nominal_joint_speed:=${RL_TRAJECTORY_NOMINAL_JOINT_SPEED}" \
  "rl_trajectory_waypoints:=${RL_TRAJECTORY_WAYPOINTS}" \
  "rl_action_low_pass_alpha:=${RL_ACTION_LOW_PASS_ALPHA}" \
  "arm_mount_x:=${ARM_MOUNT_X}" \
  "arm_mount_y:=${ARM_MOUNT_Y}" \
  "arm_mount_z:=${ARM_MOUNT_Z}" \
  "arm_mount_roll:=${ARM_MOUNT_ROLL}" \
  "arm_mount_pitch:=${ARM_MOUNT_PITCH}" \
  "arm_mount_yaw:=${ARM_MOUNT_YAW}"
