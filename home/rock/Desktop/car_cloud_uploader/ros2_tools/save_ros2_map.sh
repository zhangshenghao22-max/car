#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_DIR="${CAR_CLOUD_MAP_DIR:-${ROOT_DIR}/lidar_maps}"
RAW_NAME="${1:-cloud_map_$(date +%Y%m%d_%H%M%S)}"
MAP_NAME="$(printf '%s' "${RAW_NAME}" | tr -c 'A-Za-z0-9_.-' '_' | sed -E 's/[.](yaml|pgm|png)$//')"

if [[ -z "${MAP_NAME}" || "${MAP_NAME}" == "_" ]]; then
  MAP_NAME="cloud_map_$(date +%Y%m%d_%H%M%S)"
fi

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash}"
MAP_TOPIC="${CAR_CLOUD_MAP_TOPIC:-/map}"
SAVE_TIMEOUT="${CAR_CLOUD_SAVE_MAP_TIMEOUT:-20}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${MAP_DIR}"

set +u
source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

BASE_PATH="${MAP_DIR}/${MAP_NAME}"

if ! [[ "${SAVE_TIMEOUT}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  SAVE_TIMEOUT=20
fi
SAVE_TIMEOUT_INT="${SAVE_TIMEOUT%.*}"
if ! [[ "${SAVE_TIMEOUT_INT}" =~ ^[0-9]+$ ]]; then
  SAVE_TIMEOUT_INT=20
fi
if (( SAVE_TIMEOUT_INT < 1 )); then
  SAVE_TIMEOUT_INT=20
fi

topic_has_message() {
  local topic="$1"
  local timeout_s="$2"
  local tmp_file
  tmp_file="$(mktemp)"
  timeout "${timeout_s}" ros2 topic echo "${topic}" \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --no-arr \
    --no-str \
    >"${tmp_file}" 2>/dev/null || true
  if grep -Eq '[[:alnum:]_]' "${tmp_file}" >/dev/null 2>&1; then
    rm -f "${tmp_file}"
    return 0
  fi
  rm -f "${tmp_file}"
  return 1
}

print_map_diagnostics() {
  echo "--- ros nodes ---" >&2
  timeout 4 ros2 node list >&2 || true
  echo "--- /map info ---" >&2
  timeout 4 ros2 topic info "${MAP_TOPIC}" -v >&2 || true
  echo "--- /map hz ---" >&2
  timeout 8 ros2 topic hz "${MAP_TOPIC}" >&2 || true
}

map_has_publisher() {
  local info
  info="$(timeout 4 ros2 topic info "${MAP_TOPIC}" -v 2>/dev/null || true)"
  echo "${info}" | grep -Eq "Publisher count:[[:space:]]*[1-9]"
}

stamp_initial_pose() {
  local yaml_path="${BASE_PATH}.yaml"
  local stamper="${ROOT_DIR}/ros2_tools/stamp_map_initial_pose.py"
  if [[ ! -f "${yaml_path}" ]]; then
    echo "initial pose not saved: yaml missing ${yaml_path}" >&2
    return 1
  fi
  if [[ ! -f "${stamper}" ]]; then
    echo "initial pose stamper missing: ${stamper}" >&2
    return 1
  fi
  if ! timeout "$(( SAVE_TIMEOUT_INT + 5 ))" python3 "${stamper}" "${yaml_path}" --timeout "${SAVE_TIMEOUT}"; then
    echo "initial pose stamper failed; map file is still saved" >&2
    return 1
  fi
  return 0
}

finish_success() {
  if ! stamp_initial_pose; then
    echo "warning: initial pose was not saved cleanly; map file remains usable with manual 2D Pose Estimate" >&2
  fi
  echo "map saved: ${BASE_PATH}.yaml"
  exit 0
}

if ! map_has_publisher; then
  echo "current mapping is not running or ${MAP_TOPIC} has no publisher; cannot save map" >&2
  echo "当前未建图或 ${MAP_TOPIC} 未发布，无法保存地图" >&2
  print_map_diagnostics
  exit 1
fi

FALLBACK="${ROOT_DIR}/ros2_tools/save_ros2_map_fallback.py"
FALLBACK_LOG="$(mktemp)"
if [[ -f "${FALLBACK}" ]]; then
  echo "saving ${MAP_TOPIC} with fallback transient-local saver..."
  if timeout "$(( SAVE_TIMEOUT_INT + 5 ))" python3 "${FALLBACK}" --topic "${MAP_TOPIC}" --base-path "${BASE_PATH}" --timeout "${SAVE_TIMEOUT}" >"${FALLBACK_LOG}" 2>&1; then
    cat "${FALLBACK_LOG}"
    rm -f "${FALLBACK_LOG}"
    if [[ -f "${BASE_PATH}.yaml" ]]; then
      finish_success
    fi
    echo "fallback saver returned ok but yaml is missing: ${BASE_PATH}.yaml" >&2
  else
    echo "fallback saver failed; trying map_saver_cli" >&2
    cat "${FALLBACK_LOG}" >&2 || true
  fi
else
  echo "fallback saver missing: ${FALLBACK}; trying map_saver_cli" >&2
fi
rm -f "${FALLBACK_LOG}"

MAP_SAVER_LOG="$(mktemp)"
MAP_SAVER_OK=0
timeout "$(( SAVE_TIMEOUT_INT + 10 ))" ros2 run nav2_map_server map_saver_cli \
  -f "${BASE_PATH}" \
  --ros-args \
  -p map_topic:="${MAP_TOPIC}" \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:="${SAVE_TIMEOUT}" \
  >"${MAP_SAVER_LOG}" 2>&1 && MAP_SAVER_OK=1

if [[ "${MAP_SAVER_OK}" != "1" || ! -f "${BASE_PATH}.yaml" ]]; then
  echo "map_saver_cli failed or did not create yaml" >&2
  cat "${MAP_SAVER_LOG}" >&2 || true
  print_map_diagnostics
  rm -f "${MAP_SAVER_LOG}"
  exit 1
fi
rm -f "${MAP_SAVER_LOG}"

if [[ ! -f "${BASE_PATH}.yaml" ]]; then
  echo "map yaml was not created: ${BASE_PATH}.yaml" >&2
  exit 1
fi

finish_success
