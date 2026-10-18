#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${BUS_SERVO_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61032904-if00}"
BAUDRATE="${BUS_SERVO_BAUDRATE:-115200}"
CMD_TOPIC="${BUS_SERVO_CMD_TOPIC:-/bus_servo_cmd}"
STATE_TOPIC="${BUS_SERVO_STATE_TOPIC:-/bus_servo_state}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
LOG_DIR="${ROOT_DIR}/logs"
PID_FILE="${LOG_DIR}/bus_servo_bridge.pid"
LOG_FILE="${LOG_DIR}/bus_servo_bridge.log"

mkdir -p "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "bus servo bridge already running: pid=${old_pid}"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

if [[ ! -e "${PORT}" ]]; then
  echo "ERROR: bus servo port not found: ${PORT}" >&2
  exit 2
fi

if command -v fuser >/dev/null 2>&1 && fuser "${PORT}" >/dev/null 2>&1; then
  echo "ERROR: bus servo port is busy: ${PORT}" >&2
  fuser -v "${PORT}" || true
  exit 3
fi

(
  export ROS_DOMAIN_ID
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
  unset LD_PRELOAD
  set +u
  source /opt/ros/foxy/setup.bash
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    source "${ROOT_DIR}/install/setup.bash"
  fi
  set -u
  exec python3 "${ROOT_DIR}/ros2_tools/bus_servo_ros2_bridge.py" \
    --port "${PORT}" \
    --baudrate "${BAUDRATE}" \
    --cmd-topic "${CMD_TOPIC}" \
    --state-topic "${STATE_TOPIC}"
) >"${LOG_FILE}" 2>&1 &

pid=$!
echo "${pid}" > "${PID_FILE}"
echo "started bus servo bridge pid=${pid} port=${PORT} baudrate=${BAUDRATE} domain=${ROS_DOMAIN_ID}"
echo "log: ${LOG_FILE}"
