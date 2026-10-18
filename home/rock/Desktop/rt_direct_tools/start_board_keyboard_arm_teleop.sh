#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
PYTHON_BIN="python3"
ROS_SETUP="/opt/ros/foxy/setup.bash"
ROS_WS_SETUP="$HOME/Desktop/rock_ws/ros_ws/install/setup.bash"
MICROROS_WS_SETUP="$HOME/Desktop/rock_ws/microros_ws/install/setup.bash"

if [ -x "$VENV_DIR/bin/python" ]; then
  PYTHON_BIN="$VENV_DIR/bin/python"
fi

if [ -f "$ROS_SETUP" ]; then
  . "$ROS_SETUP" >/dev/null 2>&1 || true
fi
if [ -f "$ROS_WS_SETUP" ]; then
  . "$ROS_WS_SETUP" >/dev/null 2>&1 || true
fi
if [ -f "$MICROROS_WS_SETUP" ]; then
  . "$MICROROS_WS_SETUP" >/dev/null 2>&1 || true
fi

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -f "$HOME/Desktop/rock_ws/fastdds_no_shm.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/Desktop/rock_ws/fastdds_no_shm.xml"
fi
export PYTHONUNBUFFERED=1

cd "$ROOT_DIR"
exec "$PYTHON_BIN" "$ROOT_DIR/board_keyboard_arm_teleop.py" "$@"
