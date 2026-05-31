#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${ROOT_DIR}/runtime/cloud_mapping"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_FILE="${RUNTIME_DIR}/mapping.pid"
LOG_FILE="${LOG_DIR}/mapping.log"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

mkdir -p "${LOG_DIR}"

if [[ -s "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}" || true)"
  if [[ -n "${PID}" ]] && kill -0 "${PID}" >/dev/null 2>&1; then
    echo "mapping already running: ${PID}"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

set +u
source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

if [[ -f "${ROOT_DIR}/stop_board_f103_navigation.sh" ]]; then
  CAR_CLOUD_STOP_SILENT=1 bash "${ROOT_DIR}/stop_board_f103_navigation.sh" || true
fi

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

nohup ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=true \
  use_nav:=false \
  use_chassis_controller:=true \
  use_odom_fusion:=false \
  open_rviz:=false \
  use_auto_mapping:=true \
  >"${LOG_FILE}" 2>&1 &

PID=$!
echo "${PID}" > "${PID_FILE}"
sleep 2

if ! kill -0 "${PID}" >/dev/null 2>&1; then
  echo "mapping failed to start; log follows:" >&2
  tail -80 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  exit 1
fi

echo "mapping started: pid=${PID} domain=${ROS_DOMAIN_ID} log=${LOG_FILE}"
