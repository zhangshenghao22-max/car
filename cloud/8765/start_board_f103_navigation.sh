#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || -z "${1:-}" ]]; then
  echo "usage: bash start_board_f103_navigation.sh <map-name-or-yaml-path>" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${ROOT_DIR}/runtime/cloud_navigation"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_FILE="${RUNTIME_DIR}/navigation.pid"
LOG_FILE="${LOG_DIR}/navigation.log"
MAP_ARG="$1"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

mkdir -p "${LOG_DIR}"

resolve_map() {
  local value="$1"
  if [[ "${value}" = /* && -f "${value}" ]]; then
    printf '%s\n' "${value}"
    return 0
  fi
  if [[ -f "${ROOT_DIR}/lidar_maps/${value}" ]]; then
    printf '%s\n' "${ROOT_DIR}/lidar_maps/${value}"
    return 0
  fi
  if [[ -f "${ROOT_DIR}/lidar_maps/${value}.yaml" ]]; then
    printf '%s\n' "${ROOT_DIR}/lidar_maps/${value}.yaml"
    return 0
  fi
  if [[ -f "${HOME}/Desktop/rock_ws/ros_ws/install/rt_robot_nav2/share/rt_robot_nav2/map/${value}" ]]; then
    printf '%s\n' "${HOME}/Desktop/rock_ws/ros_ws/install/rt_robot_nav2/share/rt_robot_nav2/map/${value}"
    return 0
  fi
  if [[ -f "${HOME}/Desktop/rock_ws/ros_ws/install/rt_robot_nav2/share/rt_robot_nav2/map/${value}.yaml" ]]; then
    printf '%s\n' "${HOME}/Desktop/rock_ws/ros_ws/install/rt_robot_nav2/share/rt_robot_nav2/map/${value}.yaml"
    return 0
  fi
  printf '%s\n' "${value}"
}

MAP_FILE="$(resolve_map "${MAP_ARG}")"

if [[ -s "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}" || true)"
  if [[ -n "${PID}" ]] && kill -0 "${PID}" >/dev/null 2>&1; then
    echo "navigation already running: ${PID}"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

set +u
source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

if [[ -f "${ROOT_DIR}/stop_board_f103_mapping.sh" ]]; then
  CAR_CLOUD_STOP_SILENT=1 bash "${ROOT_DIR}/stop_board_f103_mapping.sh" || true
fi

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

nohup ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=false \
  use_nav:=true \
  use_chassis_controller:=true \
  use_odom_fusion:=false \
  map_file:="${MAP_FILE}" \
  open_rviz:=false \
  >"${LOG_FILE}" 2>&1 &

PID=$!
echo "${PID}" > "${PID_FILE}"
sleep 2

if ! kill -0 "${PID}" >/dev/null 2>&1; then
  echo "navigation failed to start; log follows:" >&2
  tail -80 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  exit 1
fi

echo "navigation started: pid=${PID} map=${MAP_FILE} domain=${ROS_DOMAIN_ID} log=${LOG_FILE}"
