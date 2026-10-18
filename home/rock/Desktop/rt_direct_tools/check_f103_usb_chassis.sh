#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-$HOME/Desktop/rock_ws/ros_ws/install/setup.bash}"
F103_PORT="${F103_PORT:-/dev/f103}"
F103_BAUD="${F103_BAUD:-115200}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"

if [ -f "$ROS_SETUP" ]; then
  set +u
  source "$ROS_SETUP"
  [ -f "$ROS_WS_SETUP" ] && source "$ROS_WS_SETUP"
  set -u
else
  echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

echo "== files =="
ls -l "$ROOT_DIR/f103_usb_twist_panel.py" "$ROOT_DIR/ros2_tools/f103_usb_ros2_bridge.py" "$ROOT_DIR/ros2_tools/ros_probe.py"
echo "== serial =="
if [ -e "$F103_PORT" ]; then
  ls -l "$F103_PORT"
else
  echo "WARN: $F103_PORT not found; available candidates:"
  ls -l /dev/f103 /dev/f103_usb /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
fi

echo "== processes =="
pgrep -af 'f103_usb_ros2_bridge.py|f103_usb_twist_panel.py|microros_chassis|minicom' || true

echo "== ROS endpoints =="
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic info "$CMD_TOPIC" -v || true
  timeout 3 ros2 topic hz "$ODOM_TOPIC" || true
else
  echo "ros2 command not available in this shell"
fi

echo "== F103 raw probe =="
python3 "$ROOT_DIR/f103_usb_twist_panel.py" --port "$F103_PORT" --baudrate "$F103_BAUD" --ping --status --raw '$ODOM:ON!' || true
