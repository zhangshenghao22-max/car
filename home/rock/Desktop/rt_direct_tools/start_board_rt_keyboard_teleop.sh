#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
PYTHON_BIN="python3"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"

if [ -x "$VENV_DIR/bin/python" ]; then
  PYTHON_BIN="$VENV_DIR/bin/python"
fi

if [ "${RT_KEYBOARD_MODE:-ros}" = "serial" ]; then
  exec sudo -E "$PYTHON_BIN" "$ROOT_DIR/board_rt_serial_keyboard_teleop.py" \
    --port "${RT_SHELL_PORT:-/dev/ttyUSB0}" \
    --baud "${RT_SHELL_BAUD:-1500000}" \
    "$@"
fi

source /opt/ros/foxy/setup.bash
if [ -f /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash ]; then
  source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -f "$HOME/Desktop/rock_ws/fastdds_no_shm.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/Desktop/rock_ws/fastdds_no_shm.xml"
fi

exec "$PYTHON_BIN" "$ROOT_DIR/board_keyboard_teleop.py" \
  --cmd-vel-topic "$CMD_VEL_TOPIC" \
  --qos-reliability "${KEYBOARD_QOS_RELIABILITY:-best_effort}" \
  --linear-velocity-step "${RT_KEYBOARD_LINEAR_STEP:-0.03}" \
  --angular-velocity-step "${RT_KEYBOARD_ANGULAR_STEP:-0.05}" \
  --max-linear-x "${RT_KEYBOARD_MAX_LINEAR_X:-0.20}" \
  --max-linear-y "${RT_KEYBOARD_MAX_LINEAR_Y:-0.20}" \
  --max-angular-z "${RT_KEYBOARD_MAX_ANGULAR_Z:-0.30}" \
  --publish-mode hold \
  --hold-rate-hz "${RT_KEYBOARD_HOLD_RATE_HZ:-10}" \
  "$@"
