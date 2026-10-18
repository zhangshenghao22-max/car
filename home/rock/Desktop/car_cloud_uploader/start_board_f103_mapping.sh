#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${ROOT_DIR}/runtime/cloud_mapping"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_FILE="${RUNTIME_DIR}/mapping.pid"
SESSION_FILE="${RUNTIME_DIR}/session.json"
LOG_FILE="${LOG_DIR}/mapping.log"
MAPPING_MODE="${1:-${CAR_CLOUD_MAPPING_MODE:-frontier}}"
SLAM_BACKEND="${2:-${CAR_CLOUD_SLAM_BACKEND:-toolbox}}"
URDF_FILE="${CAR_CLOUD_MAPPING_URDF:-c26_double_layer_robot.urdf}"
USE_AUTO_MAPPING="true"
REQUIRE_FRONTIER_STATUS="1"

case "${MAPPING_MODE}" in
  frontier|forward)
    ;;
  manual)
    USE_AUTO_MAPPING="false"
    REQUIRE_FRONTIER_STATUS="0"
    MAPPING_MODE="frontier"
    ;;
  *)
    echo "unsupported mapping mode: ${MAPPING_MODE}; expected frontier, forward, or manual" >&2
    exit 2
    ;;
esac

case "${SLAM_BACKEND}" in
  toolbox|slam_toolbox)
    SLAM_BACKEND="toolbox"
    ;;
  gmapping)
    ;;
  *)
    echo "unsupported slam backend: ${SLAM_BACKEND}; expected toolbox or gmapping" >&2
    exit 2
    ;;
esac

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

mkdir -p "${LOG_DIR}"

write_session() {
  local active="$1"
  local status="$2"
  local reason="$3"
  local pid_value="${4:-0}"
  python3 - "${SESSION_FILE}" "${active}" "${status}" "${reason}" "${pid_value}" "${MAPPING_MODE}" "${SLAM_BACKEND}" "${URDF_FILE}" <<'PY'
import json
import sys
import time

path, active, status, reason, pid, mode, backend, urdf = sys.argv[1:9]
payload = {
    "desired_mode": "mapping",
    "active": active == "1",
    "status": status,
    "reason": reason,
    "pid": int(pid) if str(pid).isdigit() else 0,
    "mode": mode,
    "slam_backend": backend,
    "urdf_file": urdf,
    "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, ensure_ascii=False, indent=2)
PY
}

publish_stop() {
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel_cmd geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

cleanup_stale_mapping() {
  if [[ -s "${PID_FILE}" ]]; then
    local old_pid
    old_pid="$(cat "${PID_FILE}" || true)"
    if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" >/dev/null 2>&1; then
      kill "${old_pid}" >/dev/null 2>&1 || true
      sleep 0.5
    fi
    rm -f "${PID_FILE}"
  fi

  pkill -TERM -f "rt_robot_nav2_complete.launch.py.*use_slam:=true.*use_nav:=false" >/dev/null 2>&1 || true
  pkill -TERM -f "/install/rplidar_ros/lib/rplidar_ros/rplidar_node" >/dev/null 2>&1 || true
  pkill -TERM -f "/opt/ros/foxy/lib/slam_toolbox/(async_slam_toolbox_node|sync_slam_toolbox_node|localization_slam_toolbox_node)" >/dev/null 2>&1 || true
  pkill -TERM -f "/install/slam_gmapping/lib/slam_gmapping/slam_gmapping" >/dev/null 2>&1 || true
  pkill -TERM -f "/install/chassis_controller/lib/chassis_controller/(frontier_explorer|auto_mapping_driver|obstacle_avoidance|odometry_to_tf|odometry_subscriber)" >/dev/null 2>&1 || true
  pkill -TERM -f "/install/imu_ros2_device/lib/imu_ros2_device/ybimu_driver" >/dev/null 2>&1 || true
  pkill -TERM -f "/install/imu_filter_madgwick/lib/imu_filter_madgwick/imu_filter_madgwick_node" >/dev/null 2>&1 || true
  pkill -TERM -f "/opt/ros/foxy/lib/(rviz2/rviz2|robot_state_publisher/robot_state_publisher|joint_state_publisher/joint_state_publisher|tf2_ros/static_transform_publisher)" >/dev/null 2>&1 || true
  sleep 1
}

topic_has_message() {
  local topic="$1"
  local timeout_s="${2:-2}"
  local qos_args=("--no-arr" "--no-str")
  if [[ "${topic}" == "/map" ]]; then
    qos_args=("--qos-reliability" "reliable" "--qos-durability" "transient_local" "--no-arr" "--no-str")
  elif [[ "${topic}" == "/scan" ]]; then
    qos_args=("--qos-reliability" "best_effort" "--qos-durability" "volatile" "--no-arr" "--no-str")
  fi
  local tmp_file
  tmp_file="$(mktemp)"
  timeout "${timeout_s}" ros2 topic echo "${topic}" "${qos_args[@]}" >"${tmp_file}" 2>/dev/null || true
  if grep -Eq '[[:alnum:]_]' "${tmp_file}" >/dev/null 2>&1; then
    rm -f "${tmp_file}"
    return 0
  fi
  rm -f "${tmp_file}"
  return 1
}

topic_has_publisher() {
  local topic="$1"
  local info
  info="$(timeout 2 ros2 topic info "${topic}" -v 2>/dev/null || true)"
  echo "${info}" | grep -Eq "Publisher count:[[:space:]]*[1-9]"
}

tf_available() {
  local timeout_s="${1:-2}"
  timeout "${timeout_s}" ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -Eq "Translation:|At time"
}

laser_tf_available() {
  local timeout_s="${1:-2}"
  timeout "${timeout_s}" ros2 run tf2_ros tf2_echo base_link laser 2>&1 | grep -Eq "Translation:|At time"
}

frontier_status_available() {
  timeout 1 ros2 topic list 2>/dev/null | grep -Eq '^/frontier_explorer/status$'
}

print_failure_context() {
  echo "mapping launch verification failed" >&2
  echo "--- ros nodes ---" >&2
  timeout 4 ros2 node list >&2 || true
  echo "--- ros topics ---" >&2
  timeout 4 ros2 topic list >&2 || true
  echo "--- /scan info ---" >&2
  timeout 4 ros2 topic info /scan -v >&2 || true
  echo "--- /map info ---" >&2
  timeout 4 ros2 topic info /map -v >&2 || true
  echo "--- tf base_link -> laser ---" >&2
  timeout 4 ros2 run tf2_ros tf2_echo base_link laser >&2 2>&1 || true
  echo "--- tf map -> base_footprint ---" >&2
  timeout 4 ros2 run tf2_ros tf2_echo map base_footprint >&2 2>&1 || true
  echo "--- mapping log tail ---" >&2
  tail -120 "${LOG_FILE}" >&2 || true
}

if [[ -s "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}" || true)"
  if [[ -n "${PID}" ]] && kill -0 "${PID}" >/dev/null 2>&1; then
    echo "stale mapping pid found, restarting: ${PID}"
  fi
fi

set +u
source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

if [[ -f "${ROOT_DIR}/stop_board_f103_navigation.sh" ]]; then
  CAR_CLOUD_STOP_SILENT=1 bash "${ROOT_DIR}/stop_board_f103_navigation.sh" || true
fi

publish_stop
cleanup_stale_mapping
publish_stop

: > "${LOG_FILE}"
write_session 1 "starting" "start_mapping" 0

nohup ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py \
  use_slam:=true \
  use_nav:=false \
  use_chassis_controller:=true \
  use_odom_fusion:=false \
  open_rviz:=false \
  use_auto_mapping:="${USE_AUTO_MAPPING}" \
  auto_mapping_mode:="${MAPPING_MODE}" \
  slam_backend:="${SLAM_BACKEND}" \
  urdf_file:="${URDF_FILE}" \
  >"${LOG_FILE}" 2>&1 &

PID=$!
echo "${PID}" > "${PID_FILE}"
write_session 1 "running" "launch_started" "${PID}"
sleep 2

if ! kill -0 "${PID}" >/dev/null 2>&1; then
  echo "mapping failed to start; log follows:" >&2
  tail -80 "${LOG_FILE}" >&2 || true
  rm -f "${PID_FILE}"
  write_session 0 "failed" "launch_process_exited" "${PID}"
  exit 1
fi

if [[ "${REQUIRE_FRONTIER_STATUS}" == "1" ]]; then
  VERIFY_OK=0
  for _ in $(seq 1 18); do
    if ! kill -0 "${PID}" >/dev/null 2>&1; then
      break
    fi
    VERIFY_NODES="$(timeout 2 ros2 node list 2>/dev/null || true)"
    if echo "${VERIFY_NODES}" | grep -Eq '(^|/)frontier_explorer$' &&
       topic_has_publisher /frontier_explorer/status &&
       topic_has_publisher /scan; then
      VERIFY_OK=1
      break
    fi
    sleep 1
  done
  if [[ "${VERIFY_OK}" != "1" ]]; then
    {
      echo "mapping launch verification warning: frontier status or scan publisher not ready"
      print_failure_context
    } >>"${LOG_FILE}" 2>&1 || true
    write_session 1 "verification_warning" "frontier_or_scan_not_ready" "${PID}"
  fi
fi

echo "mapping command executed: pid=${PID} domain=${ROS_DOMAIN_ID} mode=${MAPPING_MODE} auto=${USE_AUTO_MAPPING} slam_backend=${SLAM_BACKEND} urdf=${URDF_FILE} log=${LOG_FILE}"
