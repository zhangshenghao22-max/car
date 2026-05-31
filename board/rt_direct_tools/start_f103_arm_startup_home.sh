#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-/home/rock/Desktop/rock_ws/ros_ws/install/setup.bash}"
ARM_TOPIC="${ARM_TOPIC:-/arm_cmd}"
WAIT_SUBSCRIBER="${WAIT_SUBSCRIBER:-20}"
HOME_DURATION_MS="${HOME_DURATION_MS:-2000}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export PYTHONUNBUFFERED=1

if [ -f "${ROS_SETUP}" ]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
fi
if [ -f "${ROS_WS_SETUP}" ]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS_WS_SETUP}"
  set -u
fi

mkdir -p "${ROOT_DIR}/logs"
cd "${ROOT_DIR}"
exec python3 "${ROOT_DIR}/ros2_tools/arm_cmd_cli.py" \
  --topic "${ARM_TOPIC}" \
  --wait-subscriber "${WAIT_SUBSCRIBER}" \
  home \
  --duration "${HOME_DURATION_MS}"
