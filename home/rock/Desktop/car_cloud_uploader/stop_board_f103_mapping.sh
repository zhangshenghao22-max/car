#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${ROOT_DIR}/runtime/cloud_mapping"
PID_FILE="${RUNTIME_DIR}/mapping.pid"
SESSION_FILE="${RUNTIME_DIR}/session.json"
STOP_REASON="${1:-${CAR_CLOUD_STOP_REASON:-stop_mapping_command}}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${RUNTIME_DIR}"

write_session_stopped() {
  local reason="$1"
  local pid_value="${2:-0}"
  python3 - "${SESSION_FILE}" "${reason}" "${pid_value}" <<'PY'
import json
import sys
import time

path, reason, pid = sys.argv[1:4]
payload = {
    "desired_mode": "mapping",
    "active": False,
    "status": "stopped",
    "reason": reason,
    "pid": int(pid) if str(pid).isdigit() else 0,
    "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, ensure_ascii=False, indent=2)
PY
}

set +u
[[ -f "${ROS_SETUP}" ]] && source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

STOPPED=0
PID=""
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

pkill -TERM -f "rt_robot_nav2_complete.launch.py.*use_slam:=true.*use_nav:=false" >/dev/null 2>&1 || true
sleep 1

# Some ROS2 launch children can survive if the launch process is killed from
# outside its process group. Clean only mapping-related nodes; keep F103 alive.
pkill -TERM -f "/install/rplidar_ros/lib/rplidar_ros/rplidar_node" >/dev/null 2>&1 || true
pkill -TERM -f "/install/slam_gmapping/lib/slam_gmapping/slam_gmapping" >/dev/null 2>&1 || true
pkill -TERM -f "/opt/ros/foxy/lib/slam_toolbox/(async_slam_toolbox_node|sync_slam_toolbox_node|localization_slam_toolbox_node)" >/dev/null 2>&1 || true
pkill -TERM -f "/install/chassis_controller/lib/chassis_controller/(frontier_explorer|auto_mapping_driver|obstacle_avoidance|odometry_to_tf|odometry_subscriber)" >/dev/null 2>&1 || true
pkill -TERM -f "/install/imu_ros2_device/lib/imu_ros2_device/ybimu_driver" >/dev/null 2>&1 || true
pkill -TERM -f "/install/imu_filter_madgwick/lib/imu_filter_madgwick/imu_filter_madgwick_node" >/dev/null 2>&1 || true
pkill -TERM -f "/opt/ros/foxy/lib/(rviz2/rviz2|robot_state_publisher/robot_state_publisher|joint_state_publisher/joint_state_publisher|tf2_ros/static_transform_publisher)" >/dev/null 2>&1 || true

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

if [[ "${CAR_CLOUD_STOP_SILENT:-0}" != "1" ]]; then
  echo "mapping stopped"
fi
write_session_stopped "${STOP_REASON}" "${PID:-0}"
