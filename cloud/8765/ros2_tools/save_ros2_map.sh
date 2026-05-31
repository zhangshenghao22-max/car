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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${MAP_DIR}"

set +u
source "${ROS_SETUP}"
[[ -f "${ROS_WS_SETUP}" ]] && source "${ROS_WS_SETUP}"
set -u

BASE_PATH="${MAP_DIR}/${MAP_NAME}"
ros2 run nav2_map_server map_saver_cli \
  -f "${BASE_PATH}" \
  --ros-args \
  -p map_subscribe_transient_local:=true

if [[ ! -f "${BASE_PATH}.yaml" ]]; then
  echo "map yaml was not created: ${BASE_PATH}.yaml" >&2
  exit 1
fi

echo "map saved: ${BASE_PATH}.yaml"
