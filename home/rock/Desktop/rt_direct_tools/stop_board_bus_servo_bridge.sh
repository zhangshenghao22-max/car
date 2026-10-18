#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
PID_FILE="${LOG_DIR}/bus_servo_bridge.pid"

if [[ -f "${PID_FILE}" ]]; then
  pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    sleep 0.5
    if kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
    echo "stopped bus servo bridge pid=${pid}"
  fi
  rm -f "${PID_FILE}"
else
  echo "bus servo bridge pid file not found; checking anchored process"
fi

pkill -f "^python3 .*/ros2_tools/bus_servo_ros2_bridge.py" 2>/dev/null || true
