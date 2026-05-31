#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${ROOT_DIR}/runtime/cloud_mapping"
PID_FILE="${RUNTIME_DIR}/mapping.pid"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

set +u
[[ -f "${ROS_SETUP}" ]] && source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

STOPPED=0
if [[ -s "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}" || true)"
  if [[ -n "${PID}" ]] && kill -0 "${PID}" >/dev/null 2>&1; then
    kill "${PID}" >/dev/null 2>&1 || true
    for _ in $(seq 1 30); do
      if ! kill -0 "${PID}" >/dev/null 2>&1; then
        STOPPED=1
        break
      fi
      sleep 0.2
    done
    if [[ "${STOPPED}" != "1" ]]; then
      kill -TERM "${PID}" >/dev/null 2>&1 || true
      sleep 1
    fi
  fi
  rm -f "${PID_FILE}"
fi

pkill -TERM -f "rt_robot_nav2_complete.launch.py.*use_slam:=true.*use_auto_mapping:=true" >/dev/null 2>&1 || true

if [[ "${CAR_CLOUD_STOP_SILENT:-0}" != "1" ]]; then
  echo "mapping stopped"
fi
