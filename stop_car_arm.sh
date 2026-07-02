#!/usr/bin/env bash
set -euo pipefail

SELF_PID="$$"
SELF_PGID="$(ps -o pgid= -p "${SELF_PID}" | tr -d ' ')"

MATCH_EXPR='start_car_arm\.sh|train_car_arm_safe_rl\.sh|gas_leak_car_arm_demo\.launch\.py|rebot_arm_motion_debug\.launch\.py|rebot_arm_speed_debug\.launch\.py|gzserver|gzclient|rviz2|fastlio_mapping|move_group|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|ros2_control_node|robot_state_publisher|joint_state_publisher|static_transform_publisher|parameter_bridge|gas_leak_car_arm_task|gas_leak_mobile_manipulator_task|gas_field_simulator|spray_simulator|rebot_arm_minimal_test|rebot_safe_rl_controller|rebot_safe_rl_gazebo_trainer|spawn_entity\.py'

matching_processes() {
  ps -eo pid=,ppid=,pgid=,stat=,etime=,cmd= | awk \
    -v self_pid="${SELF_PID}" \
    -v self_pgid="${SELF_PGID}" \
    -v expr="${MATCH_EXPR}" '
      $1 == self_pid { next }
      $2 == self_pid { next }
      $3 == self_pgid { next }
      $0 ~ /stop_car_arm\.sh/ { next }
      $0 ~ /awk/ { next }
      $0 ~ expr { print }
    '
}

matching_pgids() {
  matching_processes | awk '{ print $3 }' | sort -nu
}

stop_groups() {
  local signal="$1"
  local pgid

  while read -r pgid; do
    [[ -n "${pgid}" ]] || continue
    kill "-${signal}" "-${pgid}" 2>/dev/null || true
  done
}

initial_matches="$(matching_processes || true)"
if [[ -z "${initial_matches}" ]]; then
  echo "没有找到小车+机械臂仿真的残留进程。"
  exit 0
fi

echo "找到这些仿真相关残留进程："
echo "${initial_matches}"

echo
echo "正在恢复被暂停的进程并正常关闭..."
matching_pgids | stop_groups CONT
sleep 0.5
matching_pgids | stop_groups TERM
sleep 2

remaining_matches="$(matching_processes || true)"
if [[ -n "${remaining_matches}" ]]; then
  echo
  echo "还有进程没有正常退出，正在强制结束..."
  echo "${remaining_matches}"
  matching_pgids | stop_groups KILL
  sleep 0.5
fi

final_matches="$(matching_processes || true)"
if [[ -n "${final_matches}" ]]; then
  echo
  echo "仍有残留进程，请把下面内容发给我："
  echo "${final_matches}"
  exit 1
fi

echo "已清理完成，可以重新运行 ./start_car_arm.sh。"
