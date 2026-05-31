#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-$HOME/Desktop/rock_ws/ros_ws/install/setup.bash}"
F103_PORT="${F103_PORT:-/dev/f103}"
F103_BAUD="${F103_BAUD:-115200}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
EXTRA_CMD_TOPIC="${EXTRA_CMD_TOPIC:-/cmd_vel_cmd}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
STATE_TOPIC="${STATE_TOPIC:-/f103_state}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs}"
PID_FILE="${PID_FILE:-$ROOT_DIR/f103_usb_chassis.pid}"

mkdir -p "$LOG_DIR"

if pgrep -f 'microros_chassis' >/dev/null 2>&1; then
  echo "WARN: microros_chassis process is running; stop RT direct chassis to avoid /cmd_vel conflicts." >&2
fi
if pgrep -f 'f103_usb_twist_panel.py' >/dev/null 2>&1; then
  echo "ERROR: f103_usb_twist_panel.py is running and may own the F103 serial port." >&2
  exit 1
fi
if [ -s "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" >/dev/null 2>&1; then
  echo "F103 USB chassis bridge already running, pid=$(cat "$PID_FILE")"
  exit 0
fi

if [ -f "$ROS_SETUP" ]; then
  set +u
  source "$ROS_SETUP"
  [ -f "$ROS_WS_SETUP" ] && source "$ROS_WS_SETUP"
  set -u
else
  echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

if [ -e "$F103_PORT" ]; then
  chmod a+rw "$F103_PORT" >/dev/null 2>&1 || true
else
  echo "WARN: $F103_PORT not found; bridge will try auto-detecting F103 serial." >&2
fi

nohup python3 "$ROOT_DIR/ros2_tools/f103_usb_ros2_bridge.py" \
  --port "$F103_PORT" \
  --baudrate "$F103_BAUD" \
  --cmd-topic "$CMD_TOPIC" \
  --extra-cmd-topic "$EXTRA_CMD_TOPIC" \
  --odom-topic "$ODOM_TOPIC" \
  --state-topic "$STATE_TOPIC" \
  > "$LOG_DIR/f103_usb_bridge.log" 2>&1 &
PID=$!
echo "$PID" > "$PID_FILE"
echo "Started F103 USB chassis bridge pid=$PID port=$F103_PORT baud=$F103_BAUD cmd=$CMD_TOPIC odom=$ODOM_TOPIC"
echo "Log: $LOG_DIR/f103_usb_bridge.log"
