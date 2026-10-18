#!/bin/bash
set -euo pipefail

ROOT=/home/rock/Desktop/car_cloud_uploader
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/rock/Desktop/rock_ws/fastdds_no_shm.xml}"

set +u
source /opt/ros/foxy/setup.bash
if [ -f /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash ]; then
  source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash
fi
set -u

exec python3 "$ROOT/ros2_tools/cabinet_camera_overlay.py" \
  --config "$ROOT/config/cabinet_camera_overlay.yaml"
