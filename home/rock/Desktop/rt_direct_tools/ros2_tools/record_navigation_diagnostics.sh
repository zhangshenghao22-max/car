#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/foxy/setup.bash
source /home/rock/Desktop/rock_ws/ros_ws/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/rock/Desktop/rock_ws/fastdds_no_shm.xml}"

root="${1:-/home/rock/Desktop/rt_direct_tools/logs/navigation_diagnostics}"
stamp="$(date +%Y%m%d_%H%M%S)"
output="${root}/nav_${stamp}"
mkdir -p "$root"

exec ros2 bag record -o "$output" \
  /cmd_vel \
  /cmd_vel_cmd \
  /odom_raw \
  /odom \
  /imu/data_calibrated \
  /f103_state \
  /f103_motion_fault \
  /plan \
  /tf \
  /tf_static
