#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-$HOME/Desktop/rock_ws/ros_ws/install/setup.bash}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
PID_FILE="${PID_FILE:-$ROOT_DIR/f103_usb_chassis.pid}"

if [ -f "$ROS_SETUP" ]; then
  set +u
  source "$ROS_SETUP"
  [ -f "$ROS_WS_SETUP" ] && source "$ROS_WS_SETUP"
  set -u
else
  echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if command -v ros2 >/dev/null 2>&1; then
  timeout 2 ros2 topic pub --once "$CMD_TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
fi

if [ -s "$PID_FILE" ]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" >/dev/null 2>&1; then
    kill "$PID" >/dev/null 2>&1 || true
    sleep 0.3
    kill -9 "$PID" >/dev/null 2>&1 || true
    echo "Stopped F103 USB chassis bridge pid=$PID"
  fi
  rm -f "$PID_FILE"
fi

pkill -f "$ROOT_DIR/ros2_tools/f103_usb_ros2_bridge.py" >/dev/null 2>&1 || true
echo "F103 USB chassis bridge stopped. Radar/IMU/mapping processes were not touched."
