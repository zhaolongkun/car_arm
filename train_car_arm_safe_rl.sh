#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

GUI="${GUI:-true}"
RVIZ="${RVIZ:-false}"
FAST_LIO_MODE="${FAST_LIO_MODE:-stub}"
TRAIN_START_X="${TRAIN_START_X:--3.0}"
TRAIN_START_Y="${TRAIN_START_Y:-0.0}"
TRAIN_START_YAW="${TRAIN_START_YAW:-0.0}"
CAR_SAFETY_RADIUS="${CAR_SAFETY_RADIUS:-0.452548}"
TARGET_CLEARANCE="${TARGET_CLEARANCE:-0.35}"
TRAIN_TARGET_MODE="${TRAIN_TARGET_MODE:-ground_ring}"
GROUND_TARGET_CLEARANCES="${GROUND_TARGET_CLEARANCES:-${TARGET_CLEARANCE}}"
GROUND_TARGET_ANGLES_DEG="${GROUND_TARGET_ANGLES_DEG:--90,-60,-30,0,30,60,90}"
GROUND_TARGET_Z="${GROUND_TARGET_Z:-0.0}"
SPRAY_STANDOFF="${SPRAY_STANDOFF:-0.20}"
ARM_BASE_OFFSET_X="${ARM_BASE_OFFSET_X:-0.0}"
ARM_BASE_OFFSET_Y="${ARM_BASE_OFFSET_Y:-0.0}"
ARM_BASE_HEIGHT="${ARM_BASE_HEIGHT:-0.24}"
if [[ -z "${TRAIN_TARGET_X:-}" ]]; then
  TRAIN_TARGET_X="$(awk -v radius="${CAR_SAFETY_RADIUS}" -v clearance="${TARGET_CLEARANCE}" 'BEGIN {printf "%.6f", radius + clearance}')"
fi
if [[ -z "${LEAK_X:-}" ]]; then
  LEAK_X="$(awk -v start_x="${TRAIN_START_X}" -v target_x="${TRAIN_TARGET_X}" 'BEGIN {printf "%.6f", start_x + target_x}')"
fi
LEAK_Y="${LEAK_Y:-${TRAIN_START_Y}}"
LEAK_Z="${LEAK_Z:-${GROUND_TARGET_Z}}"
TRAIN_TARGET_Y="${TRAIN_TARGET_Y:-0.0}"
TRAIN_TARGET_Z="${TRAIN_TARGET_Z:-0.16}"
USE_LIVE_LEAK_POSE_TARGET="${USE_LIVE_LEAK_POSE_TARGET:-false}"
CAR_MODEL_FILE="${CAR_MODEL_FILE:-version_car_mid360_rebot_b601_dm.sdf}"
SPAWN_CAR="${SPAWN_CAR:-true}"
TRAIN_OUTPUT_DIR="${TRAIN_OUTPUT_DIR:-${WS_DIR}/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo}"
TEACHER_EPISODES="${TEACHER_EPISODES:-80}"
BC_UPDATES="${BC_UPDATES:-500}"
UNFOLD_BEFORE_TARGET="${UNFOLD_BEFORE_TARGET:-true}"
UNFOLD_STEPS="${UNFOLD_STEPS:-48}"
RETURN_LIFT_BEFORE_ESCAPE="${RETURN_LIFT_BEFORE_ESCAPE:-true}"
RETURN_LIFT_STEPS="${RETURN_LIFT_STEPS:-120}"
RETURN_LIFT_DELTA_Z="${RETURN_LIFT_DELTA_Z:-0.45}"
RETURN_LIFT_MIN_Z="${RETURN_LIFT_MIN_Z:-0.45}"
RETURN_HOME_VIA_ESCAPE="${RETURN_HOME_VIA_ESCAPE:-true}"
RETURN_ESCAPE_STEPS="${RETURN_ESCAPE_STEPS:-96}"
RETURN_HOME_VIA_UNFOLD="${RETURN_HOME_VIA_UNFOLD:-false}"
KEEP_SIM_AFTER_TRAIN="${KEEP_SIM_AFTER_TRAIN:-false}"

source_setup() {
  set +u
  source "$1"
  set -u
}

cleanup() {
  "${ROOT_DIR}/stop_car_arm.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_for_topic_publisher() {
  local topic="$1"
  local timeout_sec="$2"
  local start_time
  start_time="$(date +%s)"
  while (( "$(date +%s)" - start_time < timeout_sec )); do
    if ros2 topic info "${topic}" 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_node() {
  local node_name="$1"
  local timeout_sec="$2"
  local start_time
  start_time="$(date +%s)"
  while (( "$(date +%s)" - start_time < timeout_sec )); do
    if ros2 node list 2>/dev/null | grep -Fxq "${node_name}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS 环境文件: ${ROS_SETUP}" >&2
  exit 1
fi

echo "先清理旧仿真..."
"${ROOT_DIR}/stop_car_arm.sh" || true

source_setup "${ROS_SETUP}"
cd "${WS_DIR}"
echo "构建 version_car_sim..."
colcon build --symlink-install --packages-select version_car_sim
source_setup "${WS_DIR}/install/setup.bash"

echo "启动整车 Gazebo 训练场景：小车 + MID360 + reBot B601-DM..."
echo "小车中心地面训练位姿: map(${TRAIN_START_X}, ${TRAIN_START_Y}, ${GROUND_TARGET_Z}), yaw=${TRAIN_START_YAW}"
echo "训练目标模式: ${TRAIN_TARGET_MODE}"
echo "小车安全半径: ${CAR_SAFETY_RADIUS} m，目标外侧净距离列表: ${GROUND_TARGET_CLEARANCES} m"
echo "地面目标角度列表: ${GROUND_TARGET_ANGLES_DEG} deg"
echo "泄漏点在地面 z=${GROUND_TARGET_Z}，spray_tip_link 目标与泄漏点保持喷洒距离: ${SPRAY_STANDOFF} m"
echo "每轮动作循环: home 全零姿态 -> 中等高度六关节展开 -> 喷洒工作点 -> 先抬高末端 -> 侧向撤离雷达/车头区域 -> home 全零姿态 -> 下一个目标点"
echo "安全展开: ${UNFOLD_BEFORE_TARGET}，回收先抬高: ${RETURN_LIFT_BEFORE_ESCAPE}，抬高到 rebot_base_link z>=${RETURN_LIFT_MIN_Z} m，回 home 先侧向撤离: ${RETURN_HOME_VIA_ESCAPE}"
echo "展开步数: ${UNFOLD_STEPS}，抬高步数: ${RETURN_LIFT_STEPS}，撤离步数: ${RETURN_ESCAPE_STEPS}"
echo "Gazebo 代表性目标 marker: map(${LEAK_X}, ${LEAK_Y}, ${LEAK_Z})"
(
  cd "${ROOT_DIR}"
  exec setsid env \
    GUI="${GUI}" \
    RVIZ="${RVIZ}" \
    FAST_LIO_MODE="${FAST_LIO_MODE}" \
    START_TASK=false \
    ARM_CONTROL_MODE=rl \
    START_X="${TRAIN_START_X}" \
    START_Y="${TRAIN_START_Y}" \
    START_YAW="${TRAIN_START_YAW}" \
    SPAWN_CAR="${SPAWN_CAR}" \
    CAR_MODEL_FILE="${CAR_MODEL_FILE}" \
    LEAK_X="${LEAK_X}" \
    LEAK_Y="${LEAK_Y}" \
    LEAK_Z="${LEAK_Z}" \
    WORK_DISTANCE="${TARGET_CLEARANCE}" \
    VEHICLE_SAFETY_RADIUS="${CAR_SAFETY_RADIUS}" \
    SKIP_BUILD=true \
    ./start_car_arm.sh
) &
LAUNCH_PID=$!

echo "等待 Gazebo、ros2_control 和机械臂控制器启动..."
sleep 18
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "联合仿真启动失败：start_car_arm.sh 已提前退出。" >&2
  echo "请先看上方 launch 日志；最常见原因是 ROS/Gazebo 依赖没 source 好，或模型文件没有安装。" >&2
  wait "${LAUNCH_PID}" || true
  exit 1
fi
if ! wait_for_node "/rebot_arm_joint_pose_trajectory" 35; then
  echo "没有检测到 Gazebo 机械臂关节轨迹插件节点 /rebot_arm_joint_pose_trajectory。" >&2
  echo "这说明带 reBot 的联合小车模型还没有正确加载，不能继续训练。" >&2
  exit 1
fi

echo "开始在整车 Gazebo 模型中采集并训练安全策略..."
ros2 run version_car_sim rebot_safe_rl_gazebo_trainer \
  --ros-args \
  -p "output_dir:=${TRAIN_OUTPUT_DIR}" \
  -p "teacher_episodes:=${TEACHER_EPISODES}" \
  -p "unfold_before_target:=${UNFOLD_BEFORE_TARGET}" \
  -p "unfold_steps:=${UNFOLD_STEPS}" \
  -p "return_lift_before_escape:=${RETURN_LIFT_BEFORE_ESCAPE}" \
  -p "return_lift_steps:=${RETURN_LIFT_STEPS}" \
  -p "return_lift_delta_z:=${RETURN_LIFT_DELTA_Z}" \
  -p "return_lift_min_z:=${RETURN_LIFT_MIN_Z}" \
  -p "return_home_via_escape:=${RETURN_HOME_VIA_ESCAPE}" \
  -p "return_escape_steps:=${RETURN_ESCAPE_STEPS}" \
  -p "return_home_via_unfold:=${RETURN_HOME_VIA_UNFOLD}" \
  -p "bc_updates:=${BC_UPDATES}" \
  -p "target_x:=${TRAIN_TARGET_X}" \
  -p "target_y:=${TRAIN_TARGET_Y}" \
  -p "target_z:=${TRAIN_TARGET_Z}" \
  -p "target_mode:=${TRAIN_TARGET_MODE}" \
  -p "car_center_x:=${TRAIN_START_X}" \
  -p "car_center_y:=${TRAIN_START_Y}" \
  -p "car_center_z:=0.0" \
  -p "car_yaw:=${TRAIN_START_YAW}" \
  -p "car_safety_radius:=${CAR_SAFETY_RADIUS}" \
  -p "arm_base_offset_x:=${ARM_BASE_OFFSET_X}" \
  -p "arm_base_offset_y:=${ARM_BASE_OFFSET_Y}" \
  -p "arm_base_height:=${ARM_BASE_HEIGHT}" \
  -p "ground_target_z:=${GROUND_TARGET_Z}" \
  -p "spray_standoff:=${SPRAY_STANDOFF}" \
  -p "ground_target_clearances:=\"${GROUND_TARGET_CLEARANCES}\"" \
  -p "ground_target_angles_deg:=\"${GROUND_TARGET_ANGLES_DEG}\"" \
  -p "use_live_leak_pose_target:=${USE_LIVE_LEAK_POSE_TARGET}"

echo "整车训练完成，策略保存目录：${TRAIN_OUTPUT_DIR}"
echo "部署时可运行："
echo "  ARM_CONTROL_MODE=rl RL_POLICY_PATH=${TRAIN_OUTPUT_DIR}/policy.pt ./start_car_arm.sh"

if [[ "${KEEP_SIM_AFTER_TRAIN}" == "true" ]]; then
  echo "KEEP_SIM_AFTER_TRAIN=true，Gazebo 将保持运行；按 Ctrl-C 后会自动清理。"
  wait "${LAUNCH_PID}" || true
else
  echo "训练结束，正在关闭 Gazebo 训练场景..."
  cleanup
  trap - EXIT
fi
