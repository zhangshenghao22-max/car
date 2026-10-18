#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-$HOME/Desktop/rock_ws/ros_ws/install/setup.bash}"
F103_PORT="${F103_PORT:-/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0}"
F103_BAUD="${F103_BAUD:-115200}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
EXTRA_CMD_TOPIC="${EXTRA_CMD_TOPIC:-/cmd_vel_cmd}"
MAPPING_CMD_TOPIC="${MAPPING_CMD_TOPIC:-/cmd_vel_mapping}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_raw}"
STATE_TOPIC="${STATE_TOPIC:-/f103_state}"
CMD_VX_SIGN="${CMD_VX_SIGN:-1}"
CMD_WZ_SIGN="${CMD_WZ_SIGN:-1}"
CMD_ANGULAR_SCALE="${CMD_ANGULAR_SCALE:-1.0}"
CMD_WZ_TRIM_PER_VX="${CMD_WZ_TRIM_PER_VX:-0.0}"
CMD_OUTPUT_MODE="${CMD_OUTPUT_MODE:-dcr}"
CMD_WHEEL_SIGNS="${CMD_WHEEL_SIGNS:-1,1,1,1}"
MAX_LINEAR_MM_S="${MAX_LINEAR_MM_S:-300}"
MAX_ANGULAR_MRAD_S="${MAX_ANGULAR_MRAD_S:-500}"
MAX_MOVING_ANGULAR_MRAD_S="${MAX_MOVING_ANGULAR_MRAD_S:-500}"
INPLACE_WHEEL_MIN_MM_S="${INPLACE_WHEEL_MIN_MM_S:-45,45,45,45}"
MAPPING_INPLACE_WHEEL_MIN_MM_S="${MAPPING_INPLACE_WHEEL_MIN_MM_S:-30,30,30,30}"
DCR_CHANGE_THRESHOLD_MM_S="${DCR_CHANGE_THRESHOLD_MM_S:-10}"
DCR_MAX_UPDATE_HZ="${DCR_MAX_UPDATE_HZ:-3}"
DCR_MAX_STALE_SEC="${DCR_MAX_STALE_SEC:-1.0}"
STARTUP_FEEDBACK_MM_S="${STARTUP_FEEDBACK_MM_S:-2}"
STARTUP_FEEDBACK_FRAMES="${STARTUP_FEEDBACK_FRAMES:-2}"
STARTUP_BOOST_WHEEL_MM_S="${STARTUP_BOOST_WHEEL_MM_S:-45}"
STARTUP_BOOST_DURATION_SEC="${STARTUP_BOOST_DURATION_SEC:-2.50}"
INPLACE_STARTUP_BOOST_WHEEL_MM_S="${INPLACE_STARTUP_BOOST_WHEEL_MM_S:-60}"
MAPPING_INPLACE_STARTUP_BOOST_WHEEL_MM_S="${MAPPING_INPLACE_STARTUP_BOOST_WHEEL_MM_S:-45}"
MAPPING_INPLACE_STARTUP_BOOST_DURATION_SEC="${MAPPING_INPLACE_STARTUP_BOOST_DURATION_SEC:-0.45}"
INPLACE_STARTUP_FEEDBACK_MM_S="${INPLACE_STARTUP_FEEDBACK_MM_S:-12}"
INPLACE_STARTUP_FEEDBACK_FRAMES="${INPLACE_STARTUP_FEEDBACK_FRAMES:-6}"
INPLACE_STARTUP_BALANCE_RATIO="${INPLACE_STARTUP_BALANCE_RATIO:-0.45}"
STARTUP_RETRY_PERIOD="${STARTUP_RETRY_PERIOD:-1.2}"
STARTUP_RETRY_MAX="${STARTUP_RETRY_MAX:-5}"
STATUS_PERIOD_SEC="${STATUS_PERIOD_SEC:-0.05}"
ODOM_VX_SIGN="${ODOM_VX_SIGN:-1}"
ODOM_LINEAR_SCALE="${ODOM_LINEAR_SCALE:-1.71}"
ODOM_ANGULAR_SCALE="${ODOM_ANGULAR_SCALE:-2.356484}"
ODOM_WZ_SIGN="${ODOM_WZ_SIGN:-1}"
ODOM_WHEEL_SIGNS="${ODOM_WHEEL_SIGNS:-1,1,-1,-1}"
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

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/rock/Desktop/rock_ws/fastdds_no_shm.xml}"
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
  --mapping-cmd-topic "$MAPPING_CMD_TOPIC" \
  --odom-topic "$ODOM_TOPIC" \
  --state-topic "$STATE_TOPIC" \
  --arm-topic /arm_cmd \
  --max-linear-mm-s "$MAX_LINEAR_MM_S" \
  --max-angular-mrad-s "$MAX_ANGULAR_MRAD_S" \
  --max-moving-angular-mrad-s "$MAX_MOVING_ANGULAR_MRAD_S" \
  --cmd-vx-sign "$CMD_VX_SIGN" \
  --cmd-wz-sign "$CMD_WZ_SIGN" \
  --cmd-angular-scale "$CMD_ANGULAR_SCALE" \
  --cmd-wz-trim-per-vx "$CMD_WZ_TRIM_PER_VX" \
  --cmd-output-mode "$CMD_OUTPUT_MODE" \
  --cmd-wheel-signs "$CMD_WHEEL_SIGNS" \
  --inplace-wheel-min-mm-s "$INPLACE_WHEEL_MIN_MM_S" \
  --mapping-inplace-wheel-min-mm-s "$MAPPING_INPLACE_WHEEL_MIN_MM_S" \
  --dcr-change-threshold-mm-s "$DCR_CHANGE_THRESHOLD_MM_S" \
  --dcr-max-update-hz "$DCR_MAX_UPDATE_HZ" \
  --dcr-max-stale-sec "$DCR_MAX_STALE_SEC" \
  --startup-feedback-mm-s "$STARTUP_FEEDBACK_MM_S" \
  --startup-feedback-frames "$STARTUP_FEEDBACK_FRAMES" \
  --startup-boost-wheel-mm-s "$STARTUP_BOOST_WHEEL_MM_S" \
  --startup-boost-duration-sec "$STARTUP_BOOST_DURATION_SEC" \
  --inplace-startup-boost-wheel-mm-s "$INPLACE_STARTUP_BOOST_WHEEL_MM_S" \
  --mapping-inplace-startup-boost-wheel-mm-s "$MAPPING_INPLACE_STARTUP_BOOST_WHEEL_MM_S" \
  --mapping-inplace-startup-boost-duration-sec "$MAPPING_INPLACE_STARTUP_BOOST_DURATION_SEC" \
  --inplace-startup-feedback-mm-s "$INPLACE_STARTUP_FEEDBACK_MM_S" \
  --inplace-startup-feedback-frames "$INPLACE_STARTUP_FEEDBACK_FRAMES" \
  --inplace-startup-balance-ratio "$INPLACE_STARTUP_BALANCE_RATIO" \
  --startup-retry-period "$STARTUP_RETRY_PERIOD" \
  --startup-retry-max "$STARTUP_RETRY_MAX" \
  --status-period "$STATUS_PERIOD_SEC" \
  --odom-vx-sign "$ODOM_VX_SIGN" \
  --odom-linear-scale "$ODOM_LINEAR_SCALE" \
  --odom-angular-scale "$ODOM_ANGULAR_SCALE" \
  --odom-wz-sign "$ODOM_WZ_SIGN" \
  --odom-wheel-signs "$ODOM_WHEEL_SIGNS" \
  > "$LOG_DIR/f103_usb_bridge.log" 2>&1 &
PID=$!
echo "$PID" > "$PID_FILE"
echo "Started F103 USB chassis bridge pid=$PID port=$F103_PORT baud=$F103_BAUD cmd=$CMD_TOPIC odom=$ODOM_TOPIC"
echo "Log: $LOG_DIR/f103_usb_bridge.log"
