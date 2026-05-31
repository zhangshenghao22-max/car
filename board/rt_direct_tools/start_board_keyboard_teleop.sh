#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
PYTHON_BIN="python3"
ROS_SETUP="/opt/ros/foxy/setup.bash"
ROS_WS_SETUP="$HOME/Desktop/rock_ws/ros_ws/install/setup.bash"

if [ -x "$VENV_DIR/bin/python" ]; then
  PYTHON_BIN="$VENV_DIR/bin/python"
fi

if [ -f "$ROS_SETUP" ]; then
  . "$ROS_SETUP" >/dev/null 2>&1 || true
fi
if [ -f "$ROS_WS_SETUP" ]; then
  . "$ROS_WS_SETUP" >/dev/null 2>&1 || true
fi

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

cd "$ROOT_DIR"

# Older helper commands used --wait-subscriber, but board_keyboard_teleop.py
# does not need or accept it. Keep the wrapper backward-compatible.
FILTERED_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --wait-subscriber)
      shift
      [ "$#" -gt 0 ] && shift
      ;;
    --wait-subscriber=*)
      shift
      ;;
    *)
      FILTERED_ARGS+=("$1")
      shift
      ;;
  esac
done

exec "$PYTHON_BIN" "$ROOT_DIR/board_keyboard_teleop.py" "${FILTERED_ARGS[@]}"
