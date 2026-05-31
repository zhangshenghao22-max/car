#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="${ROOT_DIR}/runtime/board_uploader"
PID_FILE="${LOG_DIR}/board_linux_uploader.pid"
LOG_FILE="${LOG_DIR}/board_linux_uploader.log"

ROS_SETUP="/opt/ros/foxy/setup.bash"
ROS_WS_SETUP="${HOME}/Desktop/rock_ws/ros_ws/install/setup.bash"

export CAR_CLOUD_SERVER_URL="${CAR_CLOUD_SERVER_URL:-http://115.159.33.216:8765}"
export CAR_CLOUD_UPLOAD_TOKEN="${CAR_CLOUD_UPLOAD_TOKEN:-car-cloud-upload}"
BOARD_URL="${CAR_CLOUD_BOARD_URL:-}"
export CAR_CLOUD_BOARD_ID="${CAR_CLOUD_BOARD_ID:-rk3588-f103-board}"
export CAR_CLOUD_BOARD_LABEL="${CAR_CLOUD_BOARD_LABEL:-RK3588 F103 Board}"
STATE_INTERVAL="${CAR_CLOUD_STATE_INTERVAL:-1.0}"
FRAME_INTERVAL="${CAR_CLOUD_FRAME_INTERVAL:-2.0}"
TELEOP_INTERVAL="${CAR_CLOUD_TELEOP_INTERVAL:-0.15}"
TELEOP_TOPIC="${CAR_CLOUD_TELEOP_TOPIC:-/cmd_vel_cmd}"

mkdir -p "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}" || true)"
  if [[ -n "${PID}" ]] && kill -0 "${PID}" 2>/dev/null; then
    echo "board uploader already running: ${PID}"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

set +u
source "${ROS_SETUP}"
source "${ROS_WS_SETUP}"
set -u

CMD=(
  python3
  "${ROOT_DIR}/board_linux_uploader.py"
  --server-url "${CAR_CLOUD_SERVER_URL}"
  --upload-token "${CAR_CLOUD_UPLOAD_TOKEN}"
  --board-id "${CAR_CLOUD_BOARD_ID}"
  --board-label "${CAR_CLOUD_BOARD_LABEL}"
  --state-interval "${STATE_INTERVAL}"
  --frame-interval "${FRAME_INTERVAL}"
  --teleop-interval "${TELEOP_INTERVAL}"
  --teleop-topic "${TELEOP_TOPIC}"
)

if [[ -n "${BOARD_URL}" ]]; then
  CMD+=(--board-url "${BOARD_URL}")
fi

nohup "${CMD[@]}" >>"${LOG_FILE}" 2>&1 &
echo $! > "${PID_FILE}"
sleep 2
cat "${PID_FILE}"
